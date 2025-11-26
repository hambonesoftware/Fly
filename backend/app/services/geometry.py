"""
Geometry service for the flythrough project.

This module is responsible for loading CAD models from disk, tessellating
them into a mesh representation and computing their axis‑aligned bounding
boxes.  In earlier phases the implementation used the `pythonocc-core`
bindings directly.  For Phase 5 the backend has been refactored to use
`cadquery`, which itself leverages the Open CASCADE kernel but exposes a
much simpler API.  When CadQuery cannot be imported (for example, in
limited testing environments) the service falls back to a simple stub
cube so that the rest of the application can continue to function.  The
public API mirrors the Phase 0/Phase 1 contract while hiding the
implementation details of the geometry backend.

Functions defined here:

- ``get_mesh_for_model(model_id)`` – return a ``MeshResponse`` for the
  specified model ID.  If CadQuery is available this loads the CAD file,
  tessellates it and computes normals and a bounding box.  Otherwise
  it falls back to returning a unit cube.
- ``load_shape_for_model(model_id)`` – load a CAD file via CadQuery
  importers and return a shape object.  Returns ``None`` if CadQuery
  cannot be imported.
- ``tessellate_shape_to_mesh(shape, linear_deflection, angular_deflection)``
  – tessellate a shape into flat arrays of vertices, indices and
  normals.
- ``compute_bounding_box(shape)`` – compute an axis‑aligned bounding
  box directly from a shape using CadQuery, returning (min_xyz, max_xyz).

When CadQuery is not installed the tessellation and bounding box helpers
raise ``ImportError`` and the public ``get_mesh_for_model`` falls back to
the stub cube.
"""

from __future__ import annotations

import logging
import math
import time
from collections import OrderedDict
from dataclasses import dataclass

logger = logging.getLogger(__name__)

from typing import List, Tuple, Iterable, Dict, Optional, Tuple as TyTuple

# Additional imports for mesh caching and precomputation
from pathlib import Path
from .models_store import (
    get_model_record,
    get_mesh_cache,
    upsert_mesh_cache,
    MeshCacheRecord,
    get_mesh_cache_for_binary,
    upsert_mesh_cache_for_binary,
    get_binary_file_by_id,
    update_models_status_for_binary,
)
from .mesh_cache import save_mesh_cache, load_mesh_cache
from .raster_utils import (
    _compute_distance_field_from_occupancy,
    _marching_squares_distance_field,
)

from ..api.models import MeshResponse, MeshBBox

from .slicing import SlicePlane  # ensure slicing module is imported to avoid circular import

# Third‑party numerical helper.  We import numpy here rather than at the call
# sites to avoid repetitive imports.  The mesh silhouette generator relies on
# vectorised operations for performance, but falls back to pure Python when
# numpy is unavailable.
try:
    import numpy as np  # type: ignore  # noqa: N816 – keep lowercase for numpy conventions
except Exception:
    np = None  # type: ignore  # fallback will be handled at runtime


def _orient_loops_ccw(
    loops_3d: list[list[tuple[float, float, float]]],
    ax0: int,
    ax1: int,
) -> list[list[tuple[float, float, float]]]:
    """Return copies of *loops_3d* with counter-clockwise winding.

    The raster perimeter pipeline yields polygon loops without a guaranteed
    orientation. Some downstream consumers (for example, polygon
    triangulators) assume outer rings are counter-clockwise. This helper
    projects each loop onto the active plane axes and reverses the vertex
    order when the signed area is negative. Degenerate loops are left
    unchanged.
    """

    from .slicing import _polygon_area_2d  # Local import to avoid cycles

    oriented: list[list[tuple[float, float, float]]] = []
    for loop in loops_3d:
        if len(loop) < 3:
            oriented.append(loop)
            continue
        try:
            uv_loop = [(pt[ax0], pt[ax1]) for pt in loop]
            area = _polygon_area_2d(uv_loop)
        except Exception:
            oriented.append(loop)
            continue
        if area < 0.0:
            oriented.append(list(reversed(loop)))
        else:
            oriented.append(loop)
    return oriented

########################################################################################
# Raster‑based perimeter extraction
#
# The raster perimeter engine collapses the tessellated mesh onto a specified
# orthographic plane, rasterises the projected triangles into a 2D occupancy grid,
# optionally applies simple morphological operations to close small gaps, runs a
# marching‑squares contour extraction to produce candidate polygon loops and
# performs containment filtering to discard interior islands.  This approach
# complements the existing vector/edge‑based perimeter finder and is more
# robust to tiny holes and numerical noise in the mesh.  Each function below
# operates entirely on the cached mesh and does not require re‑tessellation
# of the original CAD model.  See documentation in the repo root for usage.

def collapse_vertices_to_plane_with_axes(vertices: 'np.ndarray', plane: str) -> tuple['np.ndarray', int, int]:
    """
    Collapse an array of 3D vertices onto one of the principal planes.

    Given a collection of vertices of shape (N, 3) and a plane identifier
    ('xy', 'xz' or 'yz'), this helper zeroes out the coordinate orthogonal to
    the selected plane.  It returns the collapsed array and the indices of
    the two active axes.  For example, collapsing to the XZ plane
    zeroes out the Y coordinate and returns axis indices (0, 2).

    Args:
        vertices: Array of shape (N, 3) of world‑space vertex positions.
        plane: A string in {"xy", "xz", "yz"} identifying the target plane.

    Returns:
        A tuple (collapsed, axis0, axis1) where ``collapsed`` is a copy of
        ``vertices`` with one coordinate forced to 0, and ``axis0``/``axis1``
        give the indices of the remaining two axes.

    Raises:
        ValueError: If ``plane`` is not one of the recognised identifiers.
    """
    if np is None:
        # If numpy is unavailable the caller should avoid using this helper
        raise RuntimeError("collapse_vertices_to_plane requires numpy")
    pl = (plane or 'xy').strip().lower()
    collapsed = vertices.copy()
    if pl == 'xy':
        # Zero out Z
        collapsed[:, 2] = 0.0
        return collapsed, 0, 1
    elif pl == 'xz':
        collapsed[:, 1] = 0.0
        return collapsed, 0, 2
    elif pl == 'yz':
        collapsed[:, 0] = 0.0
        return collapsed, 1, 2
    else:
        raise ValueError(f"Unsupported plane: {plane}")




def compute_raster_perimeter_polygons(
    model_id: str,
    plane: str = 'xy',
    detail: str | None = None,
    island_mode: str = 'single',
    outline_stitch_percent: float | None = None,
    clearance_radius: float = 0.0,
) -> tuple[list[list[tuple[float, float, float]]], list[float], dict]:
    """
    Compute perimeter polygons using a raster grid and marching squares.

    This function implements a full mesh→raster→contour pipeline.  It loads the
    cached mesh for ``model_id``, collapses the vertices onto the specified
    plane, constructs a 2D occupancy grid based on triangle coverage, applies
    simple morphology to close small holes, extracts contours via marching
    squares, filters out tiny and contained polygons and returns only the
    outer islands.  The detail parameter controls the raster resolution: low
    yields a coarse grid, high yields a fine grid.  The ``outline_stitch_percent``
    argument is accepted for API compatibility but does not influence the
    raster engine; its value is recorded in the metadata. When
    ``clearance_radius`` is positive, the occupancy grid of all bodies is first
    collapsed into a single mask, a distance field is computed from that union
    and an iso-contour at the requested clearance is extracted to realise a
    union-first Minkowski boundary.

    Args:
        model_id: Identifier of the model to process.
        plane: Projection plane: 'xy', 'xz' or 'yz'.
        detail: Level of detail hint: 'low', 'medium', 'high' or 'auto'.  When
            None, defaults to 'auto'.  Determines the grid cell size.
        island_mode: 'single' to return only the largest outer polygon, or
            'multi' to return all outermost polygons.
        outline_stitch_percent: Unused in the raster algorithm but recorded in
            the returned metadata for backward compatibility.

    Returns:
        loops_3d: List of loops where each loop is a list of (x, y, z) points
            lying in the selected plane.  Loops are ordered and closed.
        areas: List of absolute polygon areas corresponding to each loop.
        meta: Dictionary containing diagnostics such as cell size, grid
            dimensions, occupancy counts and elapsed timings.  The
            ``stitchTolerance`` field records the effective sampling length
            (equal to ``cellSize``).

    Raises:
        ValueError: If an unsupported plane or detail level is specified.
    """
    global_start = time.perf_counter()
    pl = (plane or 'xy').strip().lower()
    if pl not in {'xy', 'xz', 'yz'}:
        raise ValueError(f"Unsupported plane: {plane}")
    # Normalise detail
    detail_norm = (detail or 'auto').strip().lower()
    if detail_norm not in {'low', 'medium', 'high', 'auto'}:
        detail_norm = 'auto'
    # Load mesh
    try:
        mesh_resp = get_mesh_for_model(model_id)
    except Exception as exc:
        logger.error("RasterPerimeter[%s]: failed to load mesh: %s", model_id, exc)
        raise
    verts_flat = mesh_resp.vertices or []
    idx_flat = mesh_resp.indices or []
    if not verts_flat or not idx_flat:
        logger.warning("RasterPerimeter[%s]: empty mesh", model_id)
        return [], [], {
            'stitchTolerance': None,
            'cellSize': None,
            'gridWidth': 0,
            'gridHeight': 0,
            'islandMode': island_mode,
            'totalLoops': 0,
            'totalPoints': 0,
            'timings': {},
        }
    # Convert flat arrays to numpy for convenience
    try:
        if np is None:
            raise RuntimeError("numpy is required for raster perimeter computation")
        verts = np.array(verts_flat, dtype=float).reshape(-1, 3)
        indices = np.array(idx_flat, dtype=int).reshape(-1, 3)
    except Exception as exc:
        logger.error("RasterPerimeter[%s]: failed to reshape mesh arrays: %s", model_id, exc)
        raise
    timings: dict[str, float] = {}
    # Collapse vertices to plane
    t0 = time.perf_counter()
    collapsed, ax0, ax1 = collapse_vertices_to_plane_with_axes(verts, pl)
    collapse_elapsed = time.perf_counter() - t0
    timings['collapse'] = collapse_elapsed
    # Compute bounding box and characteristic length
    try:
        bbox_min = mesh_resp.bbox.min if mesh_resp.bbox else [float(np.min(collapsed[:, ax0])), float(np.min(collapsed[:, ax1]))]
        bbox_max = mesh_resp.bbox.max if mesh_resp.bbox else [float(np.max(collapsed[:, ax0])), float(np.max(collapsed[:, ax1]))]
        dx = bbox_max[0] - bbox_min[0]
        dy = bbox_max[1] - bbox_min[1]
        dz = bbox_max[2] - bbox_min[2]
        characteristic = max(dx, dy, dz) or 1.0
    except Exception:
        characteristic = 1.0
    # Determine cell size from detail
    if detail_norm == 'low':
        cell_size = characteristic / 256.0
    elif detail_norm == 'medium':
        cell_size = characteristic / 512.0
    elif detail_norm == 'high':
        cell_size = characteristic / 1024.0
    else:  # auto
        # Choose a reasonable grid resolution based on triangle count and model size
        tri_count = len(indices)
        # Aim for roughly 512 cells along the longest dimension for typical models
        cell_size = characteristic / max(256.0, min(1024.0, float(math.sqrt(tri_count))))
    if cell_size <= 0.0:
        cell_size = characteristic / 512.0
    # Compute 2D bounding box on the active axes
    min_u = float(np.min(collapsed[:, ax0]))
    max_u = float(np.max(collapsed[:, ax0]))
    min_v = float(np.min(collapsed[:, ax1]))
    max_v = float(np.max(collapsed[:, ax1]))
    # Add one cell of padding around the bbox to ensure contours close
    padding = cell_size
    min_u -= padding
    min_v -= padding
    max_u += padding
    max_v += padding
    width = int(math.ceil((max_u - min_u) / cell_size)) + 2
    height = int(math.ceil((max_v - min_v) / cell_size)) + 2
    timings['grid_setup'] = 0.0  # placeholder; negligible time
    logger.debug(
        "RasterPerimeter[%s]: plane=%s detail=%s bbox_u=[%.3f, %.3f] bbox_v=[%.3f, %.3f] cell_size=%.5f grid=(%d,%d)",
        model_id, pl, detail_norm, min_u, max_u, min_v, max_v, cell_size, width, height,
    )
    # Initialise occupancy grid
    t0 = time.perf_counter()
    grid = np.zeros((height, width), dtype=np.uint8)
    # Rasterise each triangle
    tri_count = len(indices)
    for tri_idx, (i0, i1, i2) in enumerate(indices):
        v0 = collapsed[i0]
        v1 = collapsed[i1]
        v2 = collapsed[i2]
        # Extract 2D coordinates
        x0, y0 = v0[ax0], v0[ax1]
        x1, y1 = v1[ax0], v1[ax1]
        x2, y2 = v2[ax0], v2[ax1]
        # Compute triangle bounding box in world coords
        tri_min_u = min(x0, x1, x2)
        tri_max_u = max(x0, x1, x2)
        tri_min_v = min(y0, y1, y2)
        tri_max_v = max(y0, y1, y2)
        # Convert to grid coordinates (clamped)
        col0 = max(0, int((tri_min_u - min_u) / cell_size))
        col1 = min(width - 1, int((tri_max_u - min_u) / cell_size))
        row0 = max(0, int((tri_min_v - min_v) / cell_size))
        row1 = min(height - 1, int((tri_max_v - min_v) / cell_size))
        if col0 > col1 or row0 > row1:
            continue
        # Precompute values for barycentric test
        denom = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if denom == 0.0:
            continue  # Degenerate triangle
        for row in range(row0, row1 + 1):
            # Compute world y coordinate of cell centre
            cy = min_v + (row + 0.5) * cell_size
            for col in range(col0, col1 + 1):
                cx = min_u + (col + 0.5) * cell_size
                # Barycentric coordinates
                w0 = ( (y1 - y2) * (cx - x2) + (x2 - x1) * (cy - y2) ) / denom
                if w0 < 0.0 or w0 > 1.0:
                    continue
                w1 = ( (y2 - y0) * (cx - x2) + (x0 - x2) * (cy - y2) ) / denom
                if w1 < 0.0 or w1 > 1.0:
                    continue
                w2 = 1.0 - w0 - w1
                if w2 < 0.0 or w2 > 1.0:
                    continue
                grid[row, col] = 1
    raster_elapsed = time.perf_counter() - t0
    timings['raster'] = raster_elapsed
    occupied_count = int(grid.sum())
    logger.debug(
        "RasterPerimeter[%s]: rasterised %d triangles, occupied cells=%d in %.2f s",
        model_id, tri_count, occupied_count, raster_elapsed,
    )
    # Morphological operations: simple closing (dilate then erode)
    morph_start = time.perf_counter()
    apply_morph = False
    # Determine morphological behaviour based on detail
    if detail_norm == 'low':
        apply_morph = True
        iterations = 1
    elif detail_norm == 'medium':
        apply_morph = True
        iterations = 1
    else:
        apply_morph = False
    if apply_morph:
        for _ in range(iterations):
            # Dilation
            padded = np.pad(grid, 1, mode='constant', constant_values=0)
            dil = np.zeros_like(grid)
            # OR across 3x3 neighbourhood
            for dy in range(3):
                for dx in range(3):
                    dil |= padded[dy:dy + height, dx:dx + width]
            # Erosion
            padded = np.pad(dil, 1, mode='constant', constant_values=1)
            ero = np.ones_like(grid)
            for dy in range(3):
                for dx in range(3):
                    ero &= padded[dy:dy + height, dx:dx + width]
            grid = ero.astype(np.uint8)
        morph_elapsed = time.perf_counter() - morph_start
        timings['morphology'] = morph_elapsed
        logger.debug(
            "RasterPerimeter[%s]: morphology applied (%d iterations) in %.2f s",
            model_id, iterations, morph_elapsed,
        )
    else:
        timings['morphology'] = 0.0
    # Union-first Minkowski boundary via distance field then iso-contour
    ms_start = time.perf_counter()
    if clearance_radius > 0.0:
        distance_field = _compute_distance_field_from_occupancy(grid, cell_size)
        loops_uv = _marching_squares_distance_field(
            distance_field,
            threshold=clearance_radius,
            cell_size=cell_size,
            min_u=min_u,
            min_v=min_v,
        )
    else:
        # Backwards-compatible outline extraction when no clearance is requested
        loops_uv = _marching_squares_distance_field(
            grid.astype(float),
            threshold=0.5,
            cell_size=cell_size,
            min_u=min_u,
            min_v=min_v,
        )
    ms_elapsed = time.perf_counter() - ms_start
    timings['marching_squares'] = ms_elapsed
    logger.debug(
        "RasterPerimeter[%s]: marching squares produced %d loops in %.2f s",
        model_id, len(loops_uv), ms_elapsed,
    )
    # Convert loops to 3D coordinates and compute areas
    loops_3d: list[list[tuple[float, float, float]]] = []
    areas: list[float] = []
    # Determine collapsed axis index (the one not equal to ax0 or ax1)
    collapsed_axis = 3 - (ax0 + ax1)
    for loop_uv in loops_uv:
        # Convert to world coordinates, use cell_size scale and base offsets already included in uv coordinates
        pts3d: list[tuple[float, float, float]] = []
        for u, v in loop_uv:
            coord = [0.0, 0.0, 0.0]
            coord[ax0] = u
            coord[ax1] = v
            coord[collapsed_axis] = 0.0
            pts3d.append((coord[0], coord[1], coord[2]))
        # Compute area using shoelace on (u,v)
        area_sum = 0.0
        for i in range(len(loop_uv)):
            x0, y0 = loop_uv[i]
            x1, y1 = loop_uv[(i + 1) % len(loop_uv)]
            area_sum += x0 * y1 - x1 * y0
        area = 0.5 * area_sum
        loops_3d.append(pts3d)
        areas.append(abs(area))
    # Filter tiny loops
    filtered_loops: list[list[tuple[float, float, float]]] = []
    filtered_areas: list[float] = []
    # Minimum area threshold scales with cell_size squared and detail
    if detail_norm == 'low':
        area_thresh = (cell_size * cell_size) * 4.0
    elif detail_norm == 'medium':
        area_thresh = (cell_size * cell_size) * 2.0
    else:
        area_thresh = (cell_size * cell_size) * 0.5
    # Sort loops by descending area
    sorted_indices = sorted(range(len(loops_3d)), key=lambda i: areas[i], reverse=True)
    contained_flags = [False] * len(loops_3d)
    # Point‑in‑polygon test using ray casting
    def _point_in_poly(poly: list[tuple[float, float]], point: tuple[float, float]) -> bool:
        inside = False
        n = len(poly)
        px, py = point
        for i in range(n):
            x0, y0 = poly[i]
            x1, y1 = poly[(i + 1) % n]
            if ((y0 > py) != (y1 > py)) and (px < (x1 - x0) * (py - y0) / (y1 - y0 + 1e-16) + x0):
                inside = not inside
        return inside
    # Precompute 2D loops for containment tests
    loops2d = [ [(p[ax0], p[ax1]) for p in l] for l in loops_3d ]
    for i_idx, i in enumerate(sorted_indices):
        # Skip small loops
        if areas[i] < area_thresh:
            contained_flags[i] = True
            continue
        # Test if loop i is contained in any larger loop
        for j_idx in range(i_idx):
            j = sorted_indices[j_idx]
            if contained_flags[j]:
                continue
            # Pick a representative point (first vertex)
            rep = loops2d[i][0]
            if _point_in_poly(loops2d[j], rep):
                contained_flags[i] = True
                break
    for idx in sorted_indices:
        if not contained_flags[idx] and areas[idx] >= area_thresh:
            filtered_loops.append(loops_3d[idx])
            filtered_areas.append(areas[idx])
    # Apply island mode: return all or just largest
    if island_mode != 'multi' and filtered_loops:
        # Return only largest by area
        max_idx = max(range(len(filtered_loops)), key=lambda i: filtered_areas[i])
        filtered_loops = [filtered_loops[max_idx]]
        filtered_areas = [filtered_areas[max_idx]]
    # Normalise winding so outer loops follow CCW orientation on the active plane
    filtered_loops = _orient_loops_ccw(filtered_loops, ax0, ax1)
    total_points = sum(len(loop) for loop in filtered_loops)
    # Build metadata
    meta: dict[str, float | str | int | None | dict[str, float]] = {
        'stitchTolerance': cell_size,
        'cellSize': cell_size,
        'gridWidth': width,
        'gridHeight': height,
        'islandMode': island_mode,
        'totalLoops': len(filtered_loops),
        'totalPoints': total_points,
        'timings': timings,
        'clearanceRadius': max(0.0, clearance_radius),
    }
    # Record unused outline stitch percent for completeness
    meta['outlineStitchPercent'] = outline_stitch_percent
    timings['total'] = time.perf_counter() - global_start
    return filtered_loops, filtered_areas, meta


@dataclass(frozen=True)
class HLRPerimeterCacheKey:
    """Key used for caching HLR-derived perimeter polylines.

    The key is intentionally small and only depends on parameters that
    materially affect the outline: the model identifier, projection
    plane, stitching tolerance and the resolved world-space sampling
    step length.
    """
    model_id: str
    plane: str
    stitch_tolerance: float
    world_step: float


# Simple in-memory LRU cache for HLR perimeters.  This mirrors the
# approach used by the slice and mesh caches but is intentionally
# lightweight – outlines are cheap to store compared to full meshes.
_HLR_PERIMETER_CACHE: "OrderedDict[HLRPerimeterCacheKey, list[tuple[float, float, float]]]" = OrderedDict()
_MAX_HLR_CACHE_ENTRIES = 32


def _get_hlr_perimeter_from_cache(
    key: HLRPerimeterCacheKey,
) -> list[tuple[float, float, float]] | None:
    """Return a cached HLR perimeter for ``key`` if available.

    Accessing an entry moves it to the end of the ``OrderedDict`` so the
    cache behaves as a least-recently-used (LRU) store.
    """
    points = _HLR_PERIMETER_CACHE.pop(key, None)
    if points is not None:
        _HLR_PERIMETER_CACHE[key] = points
    return points


def _store_hlr_perimeter_in_cache(
    key: HLRPerimeterCacheKey,
    points: list[tuple[float, float, float]],
) -> None:
    """Insert a perimeter polyline into the cache and enforce the LRU
    size limit."""
    _HLR_PERIMETER_CACHE[key] = points
    _HLR_PERIMETER_CACHE.move_to_end(key)
    while len(_HLR_PERIMETER_CACHE) > _MAX_HLR_CACHE_ENTRIES:
        _HLR_PERIMETER_CACHE.popitem(last=False)


# Detect whether CadQuery is available.  If it is importable we set
# CADQUERY_AVAILABLE to True and import the module; otherwise we fall
# back to stub behaviour.  We alias ``TopoDS_Shape`` to ``object``
# for type hints to avoid importing OCC types.
try:
    import cadquery as cq
    CADQUERY_AVAILABLE = True
    # Try to alias the CadQuery Shape class for type hints.  If not
    # available we simply use ``object`` as a placeholder.
    try:
        from cadquery.occ_impl.shapes import Shape as CqShape  # type: ignore
        TopoDS_Shape = CqShape  # type: ignore  # noqa: N816 – mimic OCC naming
    except Exception:
        TopoDS_Shape = object  # type: ignore
    logger.info("CadQuery available; real CAD import/tessellation will be used")
except Exception as exc:
    CADQUERY_AVAILABLE = False
    TopoDS_Shape = object  # type: ignore
    logger.warning(
        "CadQuery not available, falling back to stub cube meshes. Reason: %r",
        exc,
    )


def get_mesh_for_model(model_id: str) -> MeshResponse:
    """Return a triangulated mesh for a given model.

    This implementation first checks for a precomputed mesh cache keyed
    by the content hash of the uploaded model.  If a cached mesh is
    available it is loaded from disk and returned immediately.  If no
    cache exists and OpenCascade is available, the function invokes
    ``precompute_mesh_for_model`` synchronously to generate the cache
    and then loads it.  If precomputation is not possible (e.g. OCC
    missing) or any step fails, the stub cube fallback is returned.

    Args:
        model_id: Identifier of the uploaded model.

    Returns:
        MeshResponse: A mesh response containing vertices, indices,
        normals and bounding box information.
    """
    # Attempt to load the cached mesh using the binary reference on the model.
    record = get_model_record(model_id)
    if record is not None:
        # Retrieve a cache based on binary_file_id and LOD 0
        cache = get_mesh_cache_for_binary(record.binary_file_id, lod=0)
        if cache is not None:
            cache_path = Path(cache.mesh_path)
            try:
                if cache_path.exists():
                    # Load cached mesh data from disk
                    vertices, indices, normals, bbox_min, bbox_max = load_mesh_cache(cache_path)
                    bbox = MeshBBox(min=bbox_min, max=bbox_max)
                    logger.info(
                        "Loaded mesh from cache for model_id=%s (binary_file_id=%s)",
                        model_id,
                        record.binary_file_id,
                    )
                    return MeshResponse(
                        modelId=model_id,
                        vertices=vertices,
                        indices=indices,
                        normals=normals,
                        bbox=bbox,
                    )
            except Exception as exc:
                # If loading fails, log and proceed to generate the cache or fall back
                logger.exception(
                    "Failed to load mesh cache for model_id=%s; will recompute. Reason: %r",
                    model_id,
                    exc,
                )
        # No cache found or failed to load: if CadQuery available, attempt precomputation
        if CADQUERY_AVAILABLE:
            try:
                precompute_mesh_for_model(model_id, lod=0)
                # After precomputation, attempt to load again
                cache = get_mesh_cache_for_binary(record.binary_file_id, lod=0)
                if cache is not None:
                    cache_path = Path(cache.mesh_path)
                    if cache_path.exists():
                        vertices, indices, normals, bbox_min, bbox_max = load_mesh_cache(cache_path)
                        bbox = MeshBBox(min=bbox_min, max=bbox_max)
                        return MeshResponse(
                            modelId=model_id,
                            vertices=vertices,
                            indices=indices,
                            normals=normals,
                            bbox=bbox,
                        )
            except Exception as exc:
                logger.exception(
                    "Error precomputing mesh for model_id=%s; falling back to stub. Reason: %r",
                    model_id,
                    exc,
                )
    # Final fallback: return stub cube
    logger.warning(
        "get_mesh_for_model(%s): returning stub cube mesh (no cache and CadQuery unavailable or failure)",
        model_id,
    )
    vertices: List[float] = [
        -0.5,
        -0.5,
        -0.5,
        0.5,
        -0.5,
        -0.5,
        0.5,
        0.5,
        -0.5,
        -0.5,
        0.5,
        -0.5,
        -0.5,
        -0.5,
        0.5,
        0.5,
        -0.5,
        0.5,
        0.5,
        0.5,
        0.5,
        -0.5,
        0.5,
        0.5,
    ]
    indices: List[int] = [
        0,
        1,
        2,
        2,
        3,
        0,
        4,
        5,
        6,
        6,
        7,
        4,
        0,
        3,
        7,
        7,
        4,
        0,
        1,
        5,
        6,
        6,
        2,
        1,
        0,
        1,
        5,
        5,
        4,
        0,
        3,
        2,
        6,
        6,
        7,
        3,
    ]
    normals: List[float] = [0.0] * len(vertices)
    bbox = _compute_bbox(vertices)
    return MeshResponse(
        modelId=model_id,
        vertices=vertices,
        indices=indices,
        normals=normals,
        bbox=bbox,
    )


def get_mesh_and_bbox_for_model(model_id: str) -> tuple[list[float], list[int], MeshBBox, bool]:
    """Return a mesh and its bounding box along with a stub indicator.

    This helper consolidates mesh and bounding box retrieval for slicing
    purposes.  It wraps :func:`get_mesh_for_model` and exposes the
    underlying flat vertex and index arrays, the bounding box object
    (as a :class:`MeshBBox`), and a boolean flag indicating whether the
    returned mesh is a stub cube.  The stub flag is set when the
    OpenCascade geometry backend is unavailable.

    Debug logs are emitted when the ``SLICE_DEBUG`` environment
    variable is set.  The log includes the model identifier, counts
    of vertices and indices, and whether the mesh is a stub.  Large
    structures such as the vertex array itself are not logged.

    Args:
        model_id: Identifier of the uploaded model whose mesh should
            be retrieved.

    Returns:
        A tuple ``(vertices, indices, bbox, is_stub)``.  ``vertices``
        is a flat list of floats (x, y, z …), ``indices`` is a flat
        list of ints forming triangles, ``bbox`` is the mesh’s
        :class:`MeshBBox` and ``is_stub`` is ``True`` if the mesh is
        the fallback cube.
    """
    import os

    mesh = get_mesh_for_model(model_id)
    # Determine whether this mesh is a stub based on CadQuery availability.
    # When CadQuery is not available the fallback cube is returned, so we
    # mark the result accordingly.  If CadQuery is available we assume the
    # mesh corresponds to a real tessellated model.
    is_stub = not CADQUERY_AVAILABLE
    verts = mesh.vertices or []
    idx = mesh.indices or []
    bbox = mesh.bbox
    # Emit debug log if requested
    if os.getenv("SLICE_DEBUG"):
        logger.debug(
            "Mesh access for slicing: model_id=%s verts=%d indices=%d stub=%s",
            model_id,
            len(verts),
            len(idx),
            is_stub,
        )
    return verts, idx, bbox, is_stub


def slice_model_mesh(model_id: str, plane: SlicePlane) -> list:
    """High‑level helper to slice a model's mesh with a given plane.

    This function retrieves the mesh and bounding box for ``model_id`` using
    :func:`get_mesh_and_bbox_for_model` and then delegates to
    :func:`~app.services.slicing.intersect_mesh_with_plane` to compute
    intersection segments.  To improve performance on repeated calls
    with identical parameters, results are cached via
    :mod:`app.services.slice_cache` using a key composed of the model
    identifier, plane type, plane offset and level of detail.  If
    caching is unavailable (for example, due to import errors) the
    function behaves as before.

    If the mesh is a stub cube (which occurs when OpenCascade is
    unavailable) or if there are no indices, an empty list is
    returned and a debug message is logged.

    Args:
        model_id: The identifier of the model to slice.
        plane: The slice plane previously constructed via
            :func:`~app.services.slicing.make_slice_plane`.

    Returns:
        A list of slice segments (instances of
        :class:`~app.services.slicing.SliceSegment`) representing the
        intersection of the mesh with the plane.  Returns an empty list
        when slicing is not applicable.
    """
    # Import intersect_mesh_with_plane lazily to avoid circular imports
    from .slicing import intersect_mesh_with_plane
    # Attempt to import cache helpers.  If unavailable, proceed without caching.
    try:
        from .slice_cache import (
            SliceCacheKey,
            get_slice_segments_from_cache,
            put_slice_segments_in_cache,
        )
    except Exception:
        SliceCacheKey = None  # type: ignore
        get_slice_segments_from_cache = None  # type: ignore
        put_slice_segments_in_cache = None  # type: ignore

    verts, idx, bbox, is_stub = get_mesh_and_bbox_for_model(model_id)
    if is_stub or not idx:
        logger.info(
            "slice_model_mesh: stub or empty mesh for %s; returning no segments",
            model_id,
        )
        return []
    # Try cache lookup
    segments = None
    if SliceCacheKey is not None and get_slice_segments_from_cache is not None:
        key = SliceCacheKey(
            model_id=model_id,
            plane_type=plane.plane_type,
            offset=plane.offset,
            lod=0,
        )
        segments = get_slice_segments_from_cache(key)
        if segments is not None:
            logger.debug(
                "slice_model_mesh: cache hit for model=%s plane=%s offset=%s",
                model_id,
                plane.plane_type,
                plane.offset,
            )
            return segments
    # Compute intersection
    segments = intersect_mesh_with_plane(verts, idx, plane)
    # Store in cache if possible
    if SliceCacheKey is not None and put_slice_segments_in_cache is not None:
        key = SliceCacheKey(
            model_id=model_id,
            plane_type=plane.plane_type,
            offset=plane.offset,
            lod=0,
        )
        put_slice_segments_in_cache(key, segments)
    return segments


def load_shape_for_model(model_id: str) -> Optional[TopoDS_Shape]:
    """Load a STEP or IGES file into a CadQuery shape.

    This helper retrieves the on-disk CAD file associated with ``model_id`` and
    imports it using CadQuery's importers. Only STEP (".step"/".stp") files
    are officially supported by CadQuery at the time of writing. IGES files
    (".iges"/".igs") are attempted via the STEP importer as a best effort.

    If CadQuery is not available this function logs and returns ``None``.
    """
    if not CADQUERY_AVAILABLE:
        logger.debug(
            "load_shape_for_model(%s): CadQuery not available, returning None",
            model_id,
        )
        return None

    start_time = time.perf_counter()
    logger.debug("load_shape_for_model(%s): begin", model_id)

    # Retrieve the path to the stored model file
    from .storage import get_model_file_path  # local import to avoid cycles

    file_path = get_model_file_path(model_id)
    ext = file_path.suffix.lower()
    logger.debug(
        "load_shape_for_model(%s): resolved file path=%s (ext=%s)",
        model_id,
        file_path,
        ext,
    )

    try:
        import cadquery as cq  # type: ignore

        import_start = time.perf_counter()
        # Import using CadQuery's importers
        if ext in (".step", ".stp"):
            shape = cq.importers.importStep(str(file_path))
        elif ext in (".iges", ".igs"):
            # CadQuery does not provide a dedicated IGES importer; we attempt
            # to read IGES as STEP. If this fails, an exception will propagate.
            shape = cq.importers.importStep(str(file_path))
        else:
            from fastapi import HTTPException
            raise HTTPException(
                status_code=400,
                detail=f"Unsupported file extension: {ext}",
            )

        import_elapsed = time.perf_counter() - import_start
        logger.debug(
            "load_shape_for_model(%s): CadQuery import finished in %.2f s (type=%s)",
            model_id,
            import_elapsed,
            type(shape),
        )
    except Exception as exc:
        logger.exception(
            "Failed to import CAD model %s via CadQuery. Reason: %r",
            file_path,
            exc,
        )
        raise

    total_elapsed = time.perf_counter() - start_time
    logger.debug(
        "load_shape_for_model(%s): completed in %.2f s",
        model_id,
        total_elapsed,
    )
    return shape


def tessellate_shape_to_mesh(
    shape: TopoDS_Shape,
    linear_deflection: float = 0.5,
    angular_deflection: float = 0.5,
) -> TyTuple[List[float], List[int], List[float]]:
    """Tessellate a CadQuery shape into vertices, indices and normals.

    This function uses CadQuery's built‑in tessellation to obtain a
    triangulated representation of the shape.  The returned mesh consists
    of flat vertex and index arrays.  Normals are not currently
    calculated and a zero vector is emitted for each vertex as a
    placeholder.  The ``linear_deflection`` and ``angular_deflection``
    parameters are passed through to the underlying tessellation routine
    where supported; if the CadQuery version only accepts a single
    tolerance parameter then ``linear_deflection`` is used.

    Args:
        shape: The CadQuery shape to tessellate.  Workplane objects
            will be converted to their underlying shape via ``val()``.
        linear_deflection: Linear deflection for meshing; lower values
            produce finer meshes.
        angular_deflection: Angular deflection for meshing.  Not all
            versions of CadQuery expose this parameter.

    Returns:
        A tuple of (vertices, indices, normals) where ``vertices`` is a
        flat list of x,y,z floats, ``indices`` is a flat list of vertex
        indices forming triangles and ``normals`` is a flat list of
        per‑vertex normals (all zeros at present).

    Raises:
        ImportError: If CadQuery is not available.
    """
    if not CADQUERY_AVAILABLE:
        raise ImportError("CadQuery is not available for tessellation")
    # Unwrap Workplane objects to obtain the underlying Shape
    try:
        # If the object has a val() method (e.g. Workplane), call it to get the Shape
        if hasattr(shape, "val"):
            cq_shape = shape.val()
        else:
            cq_shape = shape  # assume it's already a Shape
        # Attempt to tessellate with both linear and angular deflection parameters
        vertices_data: Iterable[Tuple[float, float, float]]
        triangles_data: Iterable[Tuple[int, int, int]]
        try:
            vertices_data, triangles_data = cq_shape.tessellate(
                linear_deflection, angular_deflection
            )
        except TypeError:
            # Fallback for versions that accept a single tolerance parameter
            vertices_data, triangles_data = cq_shape.tessellate(linear_deflection)
    except Exception as exc:
        logger.exception("Failed to tessellate shape via CadQuery. Reason: %r", exc)
        raise
    # Flatten vertices and triangles
    vertices: List[float] = []
    indices: List[int] = []
    normals: List[float] = []
    for vx, vy, vz in vertices_data:
        vertices.extend([float(vx), float(vy), float(vz)])
        normals.extend([0.0, 0.0, 0.0])
    for tri in triangles_data:
        # CadQuery returns 0‑based indices
        indices.extend([int(tri[0]), int(tri[1]), int(tri[2])])
    return vertices, indices, normals


def compute_bounding_box(shape: TopoDS_Shape) -> TyTuple[List[float], List[float]]:
    """Compute an axis‑aligned bounding box for a CadQuery shape.

    Args:
        shape: The shape whose bounding box is to be computed.  If the
            object implements ``val()``, the underlying shape will be used.

    Returns:
        A tuple ``(min_xyz, max_xyz)`` containing the minimum and maximum
        x, y and z coordinates.

    Raises:
        ImportError: If CadQuery is not available.
    """
    if not CADQUERY_AVAILABLE:
        raise ImportError("CadQuery is not available for bounding box computation")
    # Unwrap Workplane objects
    if hasattr(shape, "val"):
        cq_shape = shape.val()
    else:
        cq_shape = shape
    try:
        bbox = cq_shape.BoundingBox()
        # CadQuery bounding boxes expose xmin/xmax/ymin/ymax/zmin/zmax attributes
        min_xyz = [float(bbox.xmin), float(bbox.ymin), float(bbox.zmin)]
        max_xyz = [float(bbox.xmax), float(bbox.ymax), float(bbox.zmax)]
    except Exception as exc:
        logger.exception("Failed to compute bounding box via CadQuery. Reason: %r", exc)
        raise
    return min_xyz, max_xyz


def precompute_mesh_for_model(model_id: str, lod: int = 0) -> None:
    """Generate and cache a triangulated mesh for the given model.

    This helper loads the CAD shape for ``model_id``, tessellates it
    using coarse meshing parameters for the specified level of detail,
    computes the bounding box, saves the mesh to disk and updates
    ``MeshCacheRecord`` in the database.  If OpenCascade is not
    available or the shape cannot be loaded, the function logs a
    warning and sets the model status to ``failed``.

    Args:
        model_id: Identifier of the uploaded model.
        lod: Level of detail index (0 = coarse).  Currently only 0
            (coarse) is supported; additional LODs may be added in
            future phases.
    """
    # Ensure CadQuery is available; if not, we cannot precompute
    if not CADQUERY_AVAILABLE:
        logger.warning(
            "precompute_mesh_for_model(%s): CadQuery unavailable, cannot precompute",
            model_id,
        )
        return
    # Lookup model metadata
    record = get_model_record(model_id)
    if record is None:
        logger.warning(
            "precompute_mesh_for_model(%s): model record not found; skipping",
            model_id,
        )
        return
    # Attempt to load the shape from disk
    try:
        shape = load_shape_for_model(model_id)
    except Exception as exc:
        logger.exception(
            "precompute_mesh_for_model(%s): failed to load shape; skipping. Reason: %r",
            model_id,
            exc,
        )
        update_models_status_for_binary(record.binary_file_id, "failed", str(exc))
        return
    if shape is None:
        logger.warning(
            "precompute_mesh_for_model(%s): shape is None; skipping",
            model_id,
        )
        update_models_status_for_binary(record.binary_file_id, "failed", "Shape is None")
        return
    # Tessellate with coarse parameters for LOD 0.  Smaller linear and
    # angular deflection values produce finer meshes but are slower; for
    # quick previews we use relatively large values.
    try:
        vertices, indices, normals = tessellate_shape_to_mesh(
            shape,
            linear_deflection=5.0,
            angular_deflection=0.7,
        )
        # Compute bounding box using CadQuery.  We avoid re‑tessellating by
        # computing the bounding box directly on the shape where possible.  If
        # that fails we fall back to computing the box from the vertices.
        try:
            bbox_min, bbox_max = compute_bounding_box(shape)
        except Exception:
            # Fall back to computing the bounding box from the tessellated vertices
            bb = _compute_bbox(vertices)
            bbox_min, bbox_max = bb.min, bb.max
    except Exception as exc:
        logger.exception(
            "precompute_mesh_for_model(%s): tessellation failed; skipping. Reason: %r",
            model_id,
            exc,
        )
        update_models_status_for_binary(record.binary_file_id, "failed", str(exc))
        return
    # Determine cache directory and file path.  Use project root /storage/meshes
    base_dir = Path(__file__).resolve().parents[3]
    mesh_dir = base_dir / "storage" / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    # Use the binary file hash for naming the cache; this ensures all models referencing
    # the same binary will share the same cache file.
    binary = get_binary_file_by_id(record.binary_file_id)
    if binary is None:
        logger.error(
            "precompute_mesh_for_model(%s): binary file not found for id %s", model_id, record.binary_file_id
        )
        update_models_status_for_binary(record.binary_file_id, "failed", "Binary file not found")
        return
    cache_path = mesh_dir / f"{binary.file_hash}_lod{lod}.npz"
    # Save mesh to disk
    try:
        save_mesh_cache(cache_path, vertices, indices, normals, bbox_min, bbox_max)
    except Exception as exc:
        logger.exception(
            "precompute_mesh_for_model(%s): failed to save mesh cache; skipping. Reason: %r",
            model_id,
            exc,
        )
        update_models_status_for_binary(record.binary_file_id, "failed", str(exc))
        return
    # Build and upsert cache record keyed by binary_file_id
    # Compute counts before creating the cache record.  We compute these here
    # to avoid accessing detached ORM attributes after the session is committed.
    num_vertices = len(vertices) // 3
    num_triangles = len(indices) // 3
    cache_record = MeshCacheRecord(
        binary_file_id=record.binary_file_id,
        lod=lod,
        mesh_path=str(cache_path),
        vertex_count=num_vertices,
        triangle_count=num_triangles,
        bbox_min_x=bbox_min[0],
        bbox_min_y=bbox_min[1],
        bbox_min_z=bbox_min[2],
        bbox_max_x=bbox_max[0],
        bbox_max_y=bbox_max[1],
        bbox_max_z=bbox_max[2],
    )
    try:
        upsert_mesh_cache_for_binary(cache_record)
        logger.info(
            "precompute_mesh_for_model(%s): mesh cached at %s (verts=%d, tris=%d)",
            model_id,
            cache_path,
            num_vertices,
            num_triangles,
        )
        # Update status to ready for all models referencing this binary
        update_models_status_for_binary(record.binary_file_id, "ready", None)
    except Exception as exc:
        logger.exception(
            "precompute_mesh_for_model(%s): failed to update cache metadata. Reason: %r",
            model_id,
            exc,
        )
        update_models_status_for_binary(record.binary_file_id, "failed", str(exc))




def _bbox_perimeter_fallback(model_id: str, plane: str) -> list[tuple[float, float, float]]:
    """Return a simple rectangular perimeter from the CAD bounding box.

    This is used as a last‑resort fallback when HLR cannot provide a
    projected outline.  It keeps the rest of the pipeline functional so
    the UI still has something to draw.
    """
    if not CADQUERY_AVAILABLE:
        # If CadQuery is not available there is nothing sensible we can do
        # here – the caller should already have handled this case.
        raise HTTPException(
            status_code=503,
            detail="CadQuery not available; cannot compute perimeter fallback",
        )

    shape = load_shape_for_model(model_id)
    if shape is None:
        raise HTTPException(status_code=404, detail="Model shape not found")

    min_xyz, max_xyz = compute_bounding_box(shape)
    min_x, min_y, min_z = min_xyz
    max_x, max_y, max_z = max_xyz

    if plane.lower() == "xy":
        z = min_z
        pts = [
            (min_x, min_y, z),
            (max_x, min_y, z),
            (max_x, max_y, z),
            (min_x, max_y, z),
            (min_x, min_y, z),
        ]
    elif plane.lower() == "xz":
        y = min_y
        pts = [
            (min_x, y, min_z),
            (max_x, y, min_z),
            (max_x, y, max_z),
            (min_x, y, max_z),
            (min_x, y, min_z),
        ]
    else:  # "yz" or anything else – treat as yz
        x = min_x
        pts = [
            (x, min_y, min_z),
            (x, max_y, min_z),
            (x, max_y, max_z),
            (x, min_y, max_z),
            (x, min_y, min_z),
        ]
    return pts


def compute_hlr_perimeter_for_model(
    model_id: str,
    plane: str = "xy",
    stitch_tolerance: float = 1.0e-3,
    world_step: float | None = None,
    detail: str | None = None,
) -> list[tuple[float, float, float]]:
    """
    Compute an approximate perimeter of the model by projecting it onto the
    given principal plane using OCC's hidden-line removal (HLR).

    Historically this function performed the full HLR, stitching and sampling
    pipeline internally and returned a single loop.  Starting with
    version 0.9.7 this behaviour is implemented via
    ``compute_hlr_perimeter_loops`` which returns all detected loops.
    ``compute_hlr_perimeter_for_model`` now delegates to
    ``compute_hlr_perimeter_loops`` and picks the largest loop by area to
    maintain backwards compatibility.  See ``compute_hlr_perimeter_loops``
    for additional details.

    Sampling density scales with model size instead of "N points per edge".

    Returns an ordered list of 3D points lying in the projection plane.  If
    HLR is not available or fails, falls back to a simple bounding‑box
    rectangle.
    """
    # Delegate to multi-loop computation and choose the largest loop for
    # backwards compatibility.  We compute a stitching percentage based
    # on the provided tolerance and the model's bounding box diagonal so
    # that existing callers continue to behave as before.
    from .slicing import _polygon_area_2d, make_slice_plane, project_point_to_plane_uv

    if not CADQUERY_AVAILABLE:
        raise HTTPException(
            status_code=503,
            detail="CadQuery / OCC backend not available for perimeter computation",
        )

    # Load CadQuery shape for bbox & compute characteristic length
    shape = load_shape_for_model(model_id)
    if shape is None:
        raise HTTPException(status_code=404, detail="Model shape not found")
    try:
        min_xyz, max_xyz = compute_bounding_box(shape)
        dx = max_xyz[0] - min_xyz[0]
        dy = max_xyz[1] - min_xyz[1]
        dz = max_xyz[2] - min_xyz[2]
        diag = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
    except Exception:
        diag = 1.0
    # Derive world_step similar to the old implementation
    if world_step is None:
        world_step = diag / 200.0
        world_step = max(diag / 1000.0, min(world_step, diag / 50.0))
    # Adjust world_step based on detail
    detail_norm = (detail or "auto").strip().lower()
    if detail_norm == "low":
        world_step *= 2.0
    elif detail_norm == "high":
        world_step *= 0.5
    # Convert tolerance into a stitching percentage relative to diag
    stitch_fraction = 0.0
    if stitch_tolerance is not None:
        try:
            stitch_fraction = max(0.0, min(float(stitch_tolerance) / diag, 1.0))
        except Exception:
            stitch_fraction = 0.0
    stitch_percent = stitch_fraction * 100.0
    loops = compute_hlr_perimeter_loops(
        model_id=model_id,
        plane=plane,
        stitch_percent=stitch_percent if stitch_tolerance is not None else None,
        world_step=world_step,
    )
    if not loops:
        return _bbox_perimeter_fallback(model_id, plane)
    # If a single loop, return it
    if len(loops) == 1:
        return loops[0]
    # Choose the loop with the largest projected area
    slice_plane = make_slice_plane(plane, 0.0)
    max_area = None
    chosen = None
    for pts3d in loops:
        uv = [project_point_to_plane_uv(p, slice_plane) for p in pts3d]
        try:
            area = abs(_polygon_area_2d(uv))
        except Exception:
            area = 0.0
        if max_area is None or area > max_area:
            max_area = area
            chosen = pts3d
    return chosen if chosen else loops[0]


def _sample_wire_points_3d(wire, world_step: float) -> list[tuple[float, float, float]]:
    """Sample a TopoDS_Wire into a list of 3D points using a uniform step.

    The sampling strategy matches the one used in
    ``compute_hlr_perimeter_for_model``: edges are sampled with
    ``GCPnts_UniformAbscissa`` using a world‑space step length.  The
    returned list is closed if the wire is closed.

    Args:
        wire: The TopoDS_Wire to sample.
        world_step: Desired step length in world units.

    Returns:
        A list of (x, y, z) tuples representing the sampled points along
        the wire.
    """
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_EDGE
    from OCP.BRep import BRep_Tool
    from OCP.GeomAdaptor import GeomAdaptor_Curve
    from OCP.GCPnts import GCPnts_UniformAbscissa

    points: list[tuple[float, float, float]] = []
    exp = TopExp_Explorer(wire, TopAbs_EDGE)
    while exp.More():
        edge = exp.Current()
        exp.Next()
        try:
            curve, first, last = BRep_Tool.Curve(edge)
        except Exception:
            continue
        if curve is None:
            continue
        try:
            adaptor = GeomAdaptor_Curve(curve, first, last)
        except Exception:
            continue
        try:
            absc = GCPnts_UniformAbscissa(adaptor, world_step)
        except Exception:
            try:
                absc = GCPnts_UniformAbscissa(adaptor, 10)
            except Exception:
                continue
        nb = absc.NbPoints()
        for i in range(1, nb + 1):
            try:
                param = absc.Parameter(i)
                pnt = adaptor.Value(param)
            except Exception:
                continue
            xyz = (pnt.X(), pnt.Y(), pnt.Z())
            if not points or xyz != points[-1]:
                points.append(xyz)
    if points and points[0] != points[-1]:
        points.append(points[0])
    return points


def compute_hlr_perimeter_loops(
    model_id: str,
    plane: str = "xy",
    stitch_percent: float | None = None,
    world_step: float | None = None,
    min_area_fraction: float = 1.0e-4,
) -> list[list[tuple[float, float, float]]]:
    """
    Return multiple closed loops from the HLR projection of a model.

    Each loop is a list of (x, y, z) points lying in the projection plane.
    The number of loops is controlled by ``stitch_percent``:

    * 0%  -> almost no stitching (many islands)
    * 100% -> aggressive stitching (tends toward a single silhouette)
    """
    global_start = time.perf_counter()
    logger.debug(
        "HLR[%s]: start compute_hlr_perimeter_loops plane=%s "
        "stitch_percent=%s world_step=%s min_area_fraction=%s",
        model_id,
        plane,
        stitch_percent,
        world_step,
        min_area_fraction,
    )

    if not CADQUERY_AVAILABLE:
        logger.debug(
            "HLR[%s]: CadQuery not available, using bbox fallback", model_id
        )
        raise HTTPException(
            status_code=503,
            detail="CadQuery / OCC backend not available for perimeter computation",
        )

    # Load CadQuery shape and unwrap to OCC shape
    shape_load_start = time.perf_counter()
    shape = load_shape_for_model(model_id)
    if shape is None:
        logger.error("HLR[%s]: shape is None", model_id)
        raise HTTPException(status_code=404, detail="Model shape not found")

    load_elapsed = time.perf_counter() - shape_load_start
    logger.debug(
        "HLR[%s]: load_shape_for_model finished in %.2f s (type=%s)",
        model_id,
        load_elapsed,
        type(shape),
    )

    occ_candidate = shape
    try:
        if hasattr(occ_candidate, "val"):
            val_obj = occ_candidate.val()
            if val_obj is not None:
                occ_candidate = val_obj
    except Exception as exc:
        logger.debug(
            "HLR[%s]: val() unwrap failed, continuing with original object: %r",
            model_id,
            exc,
        )

    occ_shape = getattr(occ_candidate, "wrapped", occ_candidate)

    # Compute bounding box and characteristic length
    try:
        min_xyz, max_xyz = compute_bounding_box(shape)
        dx = max_xyz[0] - min_xyz[0]
        dy = max_xyz[1] - min_xyz[1]
        dz = max_xyz[2] - min_xyz[2]
        diag = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
        logger.debug(
            "HLR[%s]: bbox dx=%.3f dy=%.3f dz=%.3f diag=%.3f",
            model_id,
            dx,
            dy,
            dz,
            diag,
        )
    except Exception as exc:
        logger.warning(
            "HLR[%s]: compute_bounding_box failed (%r), "
            "falling back to diag=1.0",
            model_id,
            exc,
        )
        diag = 1.0

    # Derive sampling step
    if world_step is None:
        ws = diag / 200.0
        ws = max(diag / 1000.0, min(ws, diag / 50.0))
        world_step = ws

    # Stitch tolerance as fraction of characteristic length
    if stitch_percent is None:
        stitch_fraction = 0.02
    else:
        stitch_fraction = max(0.0, min(stitch_percent / 100.0, 1.0))
    stitch_tolerance = stitch_fraction * diag

    logger.debug(
        "HLR[%s]: world_step=%.5f stitch_percent=%s stitch_tolerance=%.5f",
        model_id,
        world_step,
        stitch_percent,
        stitch_tolerance,
    )

    # Import OCC types
    try:
        from OCP.Bnd import Bnd_Box
        from OCP.BRepBndLib import BRepBndLib
        from OCP.HLRAlgo import HLRAlgo_Projector
        from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
        from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopAbs import TopAbs_EDGE, TopAbs_WIRE
        from OCP.ShapeAnalysis import ShapeAnalysis_FreeBounds
        from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
        from OCP.BRepGProp import BRepGProp
        from OCP.GProp import GProp_GProps
    except Exception as exc:
        logger.warning(
            "HLR[%s]: HLR imports failed (%r), using bbox perimeter",
            model_id,
            exc,
        )
        return [_bbox_perimeter_fallback(model_id, plane)]

    # Build OCC bounding box for projector placement
    box = Bnd_Box()
    BRepBndLib.Add(occ_shape, box)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()

    if plane.lower() == "xy":
        origin = gp_Pnt((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, zmax + diag * 0.1)
        normal = gp_Dir(0.0, 0.0, -1.0)
    elif plane.lower() == "xz":
        origin = gp_Pnt((xmin + xmax) * 0.5, ymax + diag * 0.1, (zmin + zmax) * 0.5)
        normal = gp_Dir(0.0, -1.0, 0.0)
    else:
        origin = gp_Pnt(xmax + diag * 0.1, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5)
        normal = gp_Dir(-1.0, 0.0, 0.0)

    logger.debug(
        "HLR[%s]: projector origin=(%.3f, %.3f, %.3f) plane=%s",
        model_id,
        origin.X(),
        origin.Y(),
        origin.Z(),
        plane,
    )

    # Run HLR algorithm
    hlr_start = time.perf_counter()
    projector = HLRAlgo_Projector(gp_Ax2(origin, normal))
    algo = HLRBRep_Algo()
    algo.Add(occ_shape)
    logger.debug("HLR[%s]: algo.Add completed", model_id)

    algo.Projector(projector)
    logger.debug("HLR[%s]: algo.Projector set", model_id)

    update_start = time.perf_counter()
    algo.Update()
    algo.Hide()
    update_elapsed = time.perf_counter() - update_start
    logger.debug(
        "HLR[%s]: algo.Update + Hide took %.2f s",
        model_id,
        update_elapsed,
    )

    hlr_shapes = HLRBRep_HLRToShape(algo)
    hlr_elapsed = time.perf_counter() - hlr_start
    logger.debug(
        "HLR[%s]: HLR total (Add + Update + ToShape) took %.2f s",
        model_id,
        hlr_elapsed,
    )

    comp = hlr_shapes.OutLineVCompound()
    if comp.IsNull():
        comp = hlr_shapes.VCompound()

    # Collect edges from HLR result
    edges: list = []
    edge_start = time.perf_counter()
    exp_e = TopExp_Explorer(comp, TopAbs_EDGE)
    while exp_e.More():
        edges.append(exp_e.Current())
        exp_e.Next()
    edge_elapsed = time.perf_counter() - edge_start
    logger.debug(
        "HLR[%s]: collected %d edges in %.2f s",
        model_id,
        len(edges),
        edge_elapsed,
    )

    if not edges:
        logger.warning("HLR[%s]: no edges, using bbox perimeter", model_id)
        return [_bbox_perimeter_fallback(model_id, plane)]

    # Connect edges into wires using the stitch tolerance
    fb = ShapeAnalysis_FreeBounds()
    connect_start = time.perf_counter()
    try:
        wires_compound = fb.ConnectEdgesToWires(edges, stitch_tolerance, True)
    except Exception as exc:
        logger.warning(
            "HLR[%s]: ConnectEdgesToWires failed (%r), using bbox perimeter",
            model_id,
            exc,
        )
        return [_bbox_perimeter_fallback(model_id, plane)]
    connect_elapsed = time.perf_counter() - connect_start
    logger.debug(
        "HLR[%s]: ConnectEdgesToWires took %.2f s",
        model_id,
        connect_elapsed,
    )

    # Filter wires to remove tiny loops and sample points
    area_threshold = (diag * diag) * float(min_area_fraction)
    loops: list[list[tuple[float, float, float]]] = []
    total_points = 0
    wire_count = 0

    loop_start = time.perf_counter()
    exp_w = TopExp_Explorer(wires_compound, TopAbs_WIRE)
    while exp_w.More():
        wire = exp_w.Current()
        exp_w.Next()
        wire_count += 1

        try:
            face = BRepBuilderAPI_MakeFace(wire).Face()
            props = GProp_GProps()
            BRepGProp.SurfaceProperties(face, props)
            area = abs(props.Mass())
        except Exception:
            area = 0.0

        if area < area_threshold:
            continue

        pts = _sample_wire_points_3d(wire, world_step)
        if pts and len(pts) >= 4:
            loops.append(pts)
            total_points += len(pts)

    loop_elapsed = time.perf_counter() - loop_start
    logger.debug(
        "HLR[%s]: processed %d wires, kept %d loops "
        "(total_points=%d) in %.2f s (area_threshold=%.6f)",
        model_id,
        wire_count,
        len(loops),
        total_points,
        loop_elapsed,
        area_threshold,
    )

    if not loops:
        logger.warning(
            "HLR[%s]: no valid loops after filtering, using bbox perimeter",
            model_id,
        )
        return [_bbox_perimeter_fallback(model_id, plane)]

    total_elapsed = time.perf_counter() - global_start
    logger.debug(
        "HLR[%s]: compute_hlr_perimeter_loops finished with %d loops "
        "in %.2f s",
        model_id,
        len(loops),
        total_elapsed,
    )
    return loops


def _compute_bbox(vertices: List[float]) -> MeshBBox:
    """Compute an axis‑aligned bounding box from a flat vertex list.

    Args:
        vertices: A flat list of x, y, z coordinates.

    Returns:
        MeshBBox with min and max coordinates.
    """
    xs = vertices[0::3]
    ys = vertices[1::3]
    zs = vertices[2::3]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)
    return MeshBBox(min=[min_x, min_y, min_z], max=[max_x, max_y, max_z])


# -----------------------------------------------------------------------------
# Mesh‑based silhouette extraction
#
# The HLR‑based perimeter computation used in earlier versions relied on Open
# CASCADE's hidden line removal functionality.  While robust, that approach
# suffers from slow performance on large models because it must operate on the
# full BRep representation.  To improve the responsiveness of outline and
# perimeter queries the following helper functions implement a purely
# mesh‑based silhouette generator.  The algorithm collapses the tessellated
# mesh along one axis, identifies boundary edges in the resulting 2D projection
# and stitches them into closed loops.  Because the computation is linear in
# the number of triangles it scales well to large models.

def collapse_vertices_to_plane(vertices: "np.ndarray", plane: str) -> "np.ndarray":
    """
    Collapse an array of 3D vertices onto a principal plane.

    Given an ``(N, 3)`` array of vertex coordinates, this helper returns a
    copy in which one coordinate is zeroed out depending on the target plane:

    * ``plane == 'xy'`` → z‑coordinates become 0
    * ``plane == 'xz'`` → y‑coordinates become 0
    * ``plane == 'yz'`` → x‑coordinates become 0

    Args:
        vertices: An ``(N, 3)`` array of vertex positions.
        plane: One of ``'xy'``, ``'xz'`` or ``'yz'`` (case insensitive).

    Returns:
        A new ``(N, 3)`` array with the appropriate component set to zero.

    Raises:
        ValueError: If an unsupported plane is requested.
    """
    if np is None:
        # Fallback: operate on nested lists.  We simply deep copy and
        # zero out the appropriate coordinate on each row.
        collapsed = [list(v) for v in vertices]
        pl = plane.lower()
        if pl == "xy":
            for v in collapsed:
                v[2] = 0.0
        elif pl == "xz":
            for v in collapsed:
                v[1] = 0.0
        elif pl == "yz":
            for v in collapsed:
                v[0] = 0.0
        else:
            raise ValueError(f"Unsupported plane: {plane}")
        return collapsed  # type: ignore[return-value]
    else:
        collapsed = vertices.copy()
        pl = plane.lower()
        if pl == "xy":
            collapsed[:, 2] = 0.0
        elif pl == "xz":
            collapsed[:, 1] = 0.0
        elif pl == "yz":
            collapsed[:, 0] = 0.0
        else:
            raise ValueError(f"Unsupported plane: {plane}")
        return collapsed


def compute_mesh_silhouette_loops(
    model_id: str,
    plane: str = "xy",
    outline_stitch_percent: float | None = None,
    height_offset: float = 0.0,
    *,
    min_area_fraction: float = 1.0e-4,
) -> list[list[tuple[float, float, float]]]:
    """
    Compute silhouette loops of a model by collapsing its mesh onto a plane.

    This function accesses the cached mesh for ``model_id``, projects it onto
    the specified orthographic plane by zeroing out the orthogonal coordinate,
    and then extracts the outer boundary edges in 2D.  The boundary edges
    are stitched into closed loops which are returned as lists of 3D points
    lying exactly in the chosen plane at ``height_offset``.  The algorithm
    avoids any dependency on OpenCASCADE or BReps and scales roughly with
    the number of triangles in the tessellated mesh.

    Args:
        model_id: Identifier of the model whose mesh silhouette is to be
            computed.
        plane: One of ``'xy'``, ``'xz'`` or ``'yz'``.  Defaults to ``'xy'``.
        outline_stitch_percent: Percentage (0–100) controlling how aggressively
            to stitch nearby boundary vertices into single points.  A higher
            value merges islands that are within a larger fraction of the
            model's diagonal.  When ``None`` or 0, a very small tolerance
            relative to the model size is used.
        height_offset: Constant coordinate along the orthogonal axis at which
            to place the returned points.  For example, when ``plane == 'xy'``
            the returned z‑coordinate will be ``height_offset`` for all points.
        min_area_fraction: Minimum fraction of the model's diagonal squared
            used to filter out degenerate loops.  Loops with absolute area
            below ``min_area_fraction * diag ** 2`` are discarded.

    Returns:
        A list of loops, where each loop is a list of ``(x, y, z)`` points
        defining a closed polygon lying in the specified plane.  The loops are
        ordered arbitrarily.  An empty list indicates that no silhouette
        could be extracted (e.g. degenerate mesh).
    """
    start_time = time.perf_counter()
    pl = (plane or "xy").strip().lower()
    # Load mesh.  This call returns cached vertices/indices if available.
    try:
        mesh_resp = get_mesh_for_model(model_id)
    except Exception as exc:
        logger.error("mesh silhouette[%s]: failed to load mesh: %s", model_id, exc)
        return []
    vertices_flat = mesh_resp.vertices
    indices_flat = mesh_resp.indices
    # Quick sanity check
    if not vertices_flat or not indices_flat:
        return []
    # Compute bounding box and characteristic length (diagonal)
    try:
        bbox = mesh_resp.bbox
        dx = bbox.max[0] - bbox.min[0]
        dy = bbox.max[1] - bbox.min[1]
        dz = bbox.max[2] - bbox.min[2]
        diag = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
    except Exception:
        diag = 1.0
    # Determine stitch tolerance in world units
    if outline_stitch_percent is not None:
        try:
            frac = max(0.0, min(float(outline_stitch_percent), 100.0)) / 100.0
        except Exception:
            frac = 0.0
    else:
        frac = 0.0
    stitch_tol = max(frac * diag, diag * 1.0e-6)
    # Reshape vertices into (N, 3) array or list
    vcount = len(vertices_flat) // 3
    if np is not None:
        try:
            verts = np.array(vertices_flat, dtype=float).reshape((vcount, 3))
        except Exception:
            # Fallback to Python lists if numpy conversion fails
            verts = [
                (vertices_flat[i * 3], vertices_flat[i * 3 + 1], vertices_flat[i * 3 + 2])
                for i in range(vcount)
            ]
    else:
        verts = [
            (vertices_flat[i * 3], vertices_flat[i * 3 + 1], vertices_flat[i * 3 + 2])
            for i in range(vcount)
        ]
    # Reshape indices into (M, 3)
    tcount = len(indices_flat) // 3
    tris = [
        (indices_flat[i * 3], indices_flat[i * 3 + 1], indices_flat[i * 3 + 2])
        for i in range(tcount)
    ]
    # Collapse vertices onto the selected plane
    if np is not None and isinstance(verts, np.ndarray):
        collapsed = collapse_vertices_to_plane(verts, pl)
        # Extract 2D coordinates depending on plane
        if pl == "xy":
            coords2d = collapsed[:, :2]
        elif pl == "xz":
            coords2d = collapsed[:, [0, 2]]
        else:  # yz
            coords2d = collapsed[:, 1:]
        coords3d_collapsed = collapsed
    else:
        # When verts is a list of tuples
        collapsed_list: list[list[float]] = collapse_vertices_to_plane(verts, pl)  # type: ignore[arg-type]
        coords3d_collapsed = collapsed_list
        coords2d = []  # type: ignore[assignment]
        if pl == "xy":
            coords2d = [(p[0], p[1]) for p in collapsed_list]
        elif pl == "xz":
            coords2d = [(p[0], p[2]) for p in collapsed_list]
        else:
            coords2d = [(p[1], p[2]) for p in collapsed_list]
    # Phase timing: collapse completed
    collapse_elapsed = time.perf_counter() - start_time
    # Build map of undirected edges and count occurrences
    edge_counts: dict[tuple[int, int], int] = {}
    for tri in tris:
        i0, i1, i2 = tri
        edges = ((i0, i1), (i1, i2), (i2, i0))
        for e in edges:
            # Normalise undirected edge as (min, max)
            a, b = (e[0], e[1])
            if a > b:
                a, b = b, a
            key = (a, b)
            edge_counts[key] = edge_counts.get(key, 0) + 1
    # Identify boundary edges (those referenced exactly once)
    boundary_edges: list[tuple[int, int]] = [e for e, c in edge_counts.items() if c == 1]
    edge_elapsed = time.perf_counter() - start_time - collapse_elapsed
    # Early exit if no boundary edges
    if not boundary_edges:
        logger.debug(
            "mesh silhouette[%s]: no boundary edges found (plane=%s)", model_id, pl
        )
        return []
    # Quantise and stitch vertices into adjacency graph
    adj: dict[int, set[int]] = {}
    key_map: dict[tuple[int, int], int] = {}  # maps quantised key to node id
    node_coords: dict[int, tuple[float, float, float]] = {}  # node id -> 3D point
    next_node_id = 0
    for (v_a, v_b) in boundary_edges:
        for vi in (v_a, v_b):
            # Determine 2D coordinate of the vertex
            if np is not None and isinstance(coords2d, np.ndarray):
                x2, y2 = coords2d[vi]
            else:
                x2, y2 = coords2d[vi]
            # Quantise to stitch tolerance
            # Use rounding to nearest multiple of stitch_tol
            if stitch_tol > 0:
                qx = round(x2 / stitch_tol)
                qy = round(y2 / stitch_tol)
            else:
                qx = round(x2, 12)
                qy = round(y2, 12)
            key = (int(qx), int(qy))
            if key not in key_map:
                node_id = next_node_id
                key_map[key] = node_id
                next_node_id += 1
                # Store full 3D collapsed coordinate; copy to ensure float type
                if np is not None and isinstance(coords3d_collapsed, np.ndarray):
                    x3, y3, z3 = coords3d_collapsed[vi].tolist()
                else:
                    x3, y3, z3 = coords3d_collapsed[vi]
                # Apply height offset along orthogonal axis
                if pl == "xy":
                    z3 = height_offset
                elif pl == "xz":
                    y3 = height_offset
                else:  # yz
                    x3 = height_offset
                node_coords[node_id] = (float(x3), float(y3), float(z3))
                adj[node_id] = set()
        # Now link the two vertices via their stitched ids
        if np is not None and isinstance(coords2d, np.ndarray):
            x2a, y2a = coords2d[v_a]
            x2b, y2b = coords2d[v_b]
            if stitch_tol > 0:
                qxa = round(x2a / stitch_tol)
                qya = round(y2a / stitch_tol)
                qxb = round(x2b / stitch_tol)
                qyb = round(y2b / stitch_tol)
            else:
                qxa = round(x2a, 12)
                qya = round(y2a, 12)
                qxb = round(x2b, 12)
                qyb = round(y2b, 12)
            id_a = key_map[(int(qxa), int(qya))]
            id_b = key_map[(int(qxb), int(qyb))]
        else:
            x2a, y2a = coords2d[v_a]
            x2b, y2b = coords2d[v_b]
            if stitch_tol > 0:
                qxa = round(x2a / stitch_tol)
                qya = round(y2a / stitch_tol)
                qxb = round(x2b / stitch_tol)
                qyb = round(y2b / stitch_tol)
            else:
                qxa = round(x2a, 12)
                qya = round(y2a, 12)
                qxb = round(x2b, 12)
                qyb = round(y2b, 12)
            id_a = key_map[(int(qxa), int(qya))]
            id_b = key_map[(int(qxb), int(qyb))]
        # Add undirected adjacency
        adj[id_a].add(id_b)
        adj[id_b].add(id_a)
    # Stitching adjacency built
    stitch_elapsed = time.perf_counter() - start_time - collapse_elapsed - edge_elapsed
    # Extract loops by walking adjacency graph
    visited: set[int] = set()
    loops: list[list[tuple[float, float, float]]] = []
    for node in list(adj.keys()):
        if node in visited:
            continue
        # Start a new path
        path: list[int] = []
        current = node
        prev = None  # previous node to avoid immediate backtracking
        while True:
            path.append(current)
            visited.add(current)
            neighbours = adj[current]
            # Choose next neighbour that is not the previous; if only one, go that way
            next_nodes = [n for n in neighbours if n != prev]
            if not next_nodes:
                break  # dead end
            next_node = next_nodes[0]
            prev = current
            current = next_node
            if current == path[0]:
                # Closed loop
                path.append(current)
                break
            # Prevent infinite loop: bail if path is too long
            if len(path) > len(adj) * 2:
                break
        # Skip degenerate paths (less than 3 unique vertices)
        if len(path) < 4:
            continue
        # Convert node ids to 3D points
        pts3d = [node_coords[nid] for nid in path]
        loops.append(pts3d)
    # Filter loops by area
    kept_loops: list[list[tuple[float, float, float]]] = []
    area_threshold = (min_area_fraction * diag * diag)
    for pts3d in loops:
        # Compute area in 2D projected plane
        if not pts3d:
            continue
        # Project to 2D
        uv = []
        for (x3, y3, z3) in pts3d:
            if pl == "xy":
                uv.append((x3, y3))
            elif pl == "xz":
                uv.append((x3, z3))
            else:
                uv.append((y3, z3))
        # Compute signed area using shoelace formula
        area = 0.0
        m = len(uv)
        for i in range(m - 1):
            x0, y0 = uv[i]
            x1, y1 = uv[i + 1]
            area += x0 * y1 - x1 * y0
        area *= 0.5
        if abs(area) >= area_threshold:
            # Ensure closed loop ends at start
            if pts3d[0] != pts3d[-1]:
                pts3d = pts3d + [pts3d[0]]
            kept_loops.append(pts3d)
    total_elapsed = time.perf_counter() - start_time
    logger.debug(
        "mesh silhouette[%s]: plane=%s v=%d t=%d edges=%d loops=%d (collapse=%.3f s, edge=%.3f s, stitch=%.3f s, total=%.3f s)",
        model_id,
        pl,
        vcount,
        tcount,
        len(boundary_edges),
        len(kept_loops),
        collapse_elapsed,
        edge_elapsed,
        stitch_elapsed,
        total_elapsed,
    )
    return kept_loops

# -----------------------------------------------------------------------------
# Mesh‑based perimeter extraction
#
# While ``compute_mesh_silhouette_loops`` projects the tessellated mesh onto a
# principal plane and returns all boundary loops, it does not distinguish
# between outer and inner polygons.  In some cases the projected geometry
# contains holes or islands fully enclosed by larger loops.  The perimeter
# finder must discard any such interior polygons and return only the
# outermost islands.  This helper wraps the silhouette generator and
# performs an additional containment pass to remove interior polygons.
#
# The algorithm proceeds as follows:
#   1. Use ``compute_mesh_silhouette_loops`` to obtain candidate loops
#      collapsed onto the specified plane.  The loops are lists of
#      ``(x, y, z)`` points lying in the plane at a fixed height.
#   2. Project each loop into 2D by selecting the two active axes of the
#      plane (XY→(x,y), XZ→(x,z), YZ→(y,z)).  Compute the signed area of
#      each polygon using the shoelace formula.  Loops with negligible
#      area are discarded by the underlying silhouette helper and are not
#      considered here.
#   3. Sort the remaining loops by descending absolute area.  The largest
#      polygons correspond to the outermost islands.
#   4. For each pair of polygons (A, B) where A is larger than B, test
#      whether a representative point from B lies inside A using a
#      ray‑casting point‑in‑polygon test.  If B is found to be strictly
#      inside A, mark B as contained.  Continue until all smaller
#      polygons are processed.
#   5. Discard all polygons marked as contained.  Optionally return only
#      the single largest loop when ``island_mode != 'multi'``.

def compute_mesh_perimeter_polygons(
    model_id: str,
    plane: str = "xy",
    outline_stitch_percent: float | None = None,
    island_mode: str = "single",
    *,
    min_area_fraction: float = 1.0e-4,
) -> list[list[tuple[float, float, float]]]:
    """
    Compute robust perimeter polygons of a model by collapsing its mesh to a plane,
    stitching boundary edges into clean loops and discarding interior polygons.

    This function implements a full mesh→2D polygon pipeline.  It uses the
    cached tessellation of the model to avoid expensive BRep operations,
    projects the mesh onto the selected plane, extracts boundary edges, stitches
    them into ordered loops, filters out degenerate and very small edges, then
    performs a containment check to drop interior polygons.  The result is
    suitable for perimeter path planning and visualisation.

    Args:
        model_id: Identifier of the model to process.
        plane: Projection plane: 'xy', 'xz' or 'yz'.  Default 'xy'.
        outline_stitch_percent: Percentage controlling vertex snap tolerance
            (0–100).  A higher value merges more nearby vertices into the same
            node.  When None or 0, a minimal tolerance relative to the model
            size is used.
        island_mode: 'single' to return only the largest outer polygon, or
            'multi' to return all outermost polygons.  Default 'single'.
        min_area_fraction: Minimum fraction of the squared model diagonal
            below which polygons are ignored.  Polygons with area less than
            ``min_area_fraction * diag ** 2`` are discarded.  Default 1e-4.

    Returns:
        A list of loops, each a list of (x, y, z) points lying in the
        selected plane.  An empty list is returned if no perimeter polygons
        are found.
    """
    start_time = time.perf_counter()
    # Normalise plane
    pl = (plane or "xy").strip().lower()
    if pl not in {"xy", "xz", "yz"}:
        raise ValueError(f"Unsupported plane: {plane}")
    # Load mesh
    try:
        mesh_resp = get_mesh_for_model(model_id)
    except Exception as exc:
        logger.error("perimeter polygons[%s]: failed to load mesh: %s", model_id, exc)
        return []
    verts_flat = mesh_resp.vertices or []
    idx_flat = mesh_resp.indices or []
    # Quick sanity check
    if not verts_flat or not idx_flat:
        return []
    # Compute bounding box and characteristic length
    try:
        bbox = mesh_resp.bbox
        dx = bbox.max[0] - bbox.min[0]
        dy = bbox.max[1] - bbox.min[1]
        dz = bbox.max[2] - bbox.min[2]
        characteristic = max(dx, dy, dz)
        diag = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
    except Exception:
        characteristic = 1.0
        diag = 1.0
    # Determine stitch tolerance and minimum edge length
    if outline_stitch_percent is not None:
        try:
            frac = max(0.0, min(float(outline_stitch_percent), 100.0)) / 100.0
        except Exception:
            frac = 0.0
    else:
        frac = 0.0
    stitch_tol = max(frac * characteristic, characteristic * 1.0e-6)
    # Define a tiny edge length threshold relative to model size
    min_edge_len = characteristic * 1.0e-3  # 0.1% of characteristic length
    # Log scaling parameters
    logger.debug(
        "perimeter polygons[%s]: plane=%s stitch_percent=%s → stitch_tol=%.6g min_edge_len=%.6g", 
        model_id, pl, outline_stitch_percent, stitch_tol, min_edge_len
    )
    # Reshape vertices into (N,3)
    vcount = len(verts_flat) // 3
    if np is not None:
        try:
            verts_arr = np.array(verts_flat, dtype=float).reshape((vcount, 3))
        except Exception:
            verts_arr = None
    else:
        verts_arr = None
    # Build Python list fallback
    if verts_arr is None:
        verts = [
            (verts_flat[i * 3], verts_flat[i * 3 + 1], verts_flat[i * 3 + 2])
            for i in range(vcount)
        ]
    # Determine active axes (i0,i1) and collapsed axis index
    if pl == "xy":
        i0, i1, collapsed_idx = 0, 1, 2
    elif pl == "xz":
        i0, i1, collapsed_idx = 0, 2, 1
    else:  # yz
        i0, i1, collapsed_idx = 1, 2, 0
    # Collapse vertices to plane
    if verts_arr is not None:
        collapsed = verts_arr.copy()
        collapsed[:, collapsed_idx] = 0.0
        coords3d = collapsed  # Nx3 array
        coords2d = collapsed[:, [i0, i1]]  # Nx2 array
    else:
        collapsed_list = []  # list of 3 floats
        coords2d = []  # type: ignore
        for v in verts:
            # convert to list to allow mutation
            p = list(v)
            p[collapsed_idx] = 0.0
            collapsed_list.append(tuple(p))
            coords2d.append((p[i0], p[i1]))
        coords3d = collapsed_list  # type: ignore
    # Reshape indices into list of triangles
    tcount = len(idx_flat) // 3
    tris = [
        (idx_flat[i * 3], idx_flat[i * 3 + 1], idx_flat[i * 3 + 2])
        for i in range(tcount)
    ]
    # Build undirected edge count map and length map
    edge_counts: dict[tuple[int, int], int] = {}
    # Pre-calc 2D coords for Python list or numpy
    # For speed, we can index coords2d differently depending on type
    for tri in tris:
        i0t, i1t, i2t = tri
        # edges of triangle
        tri_edges = ((i0t, i1t), (i1t, i2t), (i2t, i0t))
        for e in tri_edges:
            a, b = e
            if a > b:
                a, b = b, a
            key = (a, b)
            edge_counts[key] = edge_counts.get(key, 0) + 1
    # Identify boundary edges and filter by length
    boundary_edges: list[tuple[int, int]] = []
    # compute edges length quickly
    if np is not None and isinstance(coords2d, np.ndarray):
        # vectorised difference is heavy; we compute individually
        for (a, b), count in edge_counts.items():
            if count != 1:
                continue
            x0, y0 = coords2d[a]
            x1, y1 = coords2d[b]
            dx_edge = x1 - x0
            dy_edge = y1 - y0
            if (dx_edge * dx_edge + dy_edge * dy_edge) >= (min_edge_len * min_edge_len):
                boundary_edges.append((a, b))
    else:
        for (a, b), count in edge_counts.items():
            if count != 1:
                continue
            x0, y0 = coords2d[a]
            x1, y1 = coords2d[b]
            dx_edge = x1 - x0
            dy_edge = y1 - y0
            if (dx_edge * dx_edge + dy_edge * dy_edge) >= (min_edge_len * min_edge_len):
                boundary_edges.append((a, b))
    # Early return if no boundary edges
    if not boundary_edges:
        logger.debug(
            "perimeter polygons[%s]: no boundary edges after filtering", model_id
        )
        return []
    # Build adjacency using quantised nodes
    node_key_to_id: Dict[tuple[int, int], int] = {}
    node_coords3d: Dict[int, tuple[float, float, float]] = {}
    adj: Dict[int, set[int]] = {}
    next_node_id = 0
    # Precompute quantisation helper
    def quantise(x: float, y: float) -> tuple[int, int]:
        # Avoid division by zero; stitch_tol > 0
        return (int(round(x / stitch_tol)), int(round(y / stitch_tol)))
    # Build nodes and adjacency
    for (va, vb) in boundary_edges:
        # Process both endpoints
        for vi in (va, vb):
            # Determine 2D coords
            if np is not None and isinstance(coords2d, np.ndarray):
                x2, y2 = coords2d[vi]
            else:
                x2, y2 = coords2d[vi]
            qx, qy = quantise(x2, y2)
            key = (qx, qy)
            if key not in node_key_to_id:
                nid = next_node_id
                node_key_to_id[key] = nid
                next_node_id += 1
                # Determine full 3D coordinate from collapsed coords
                if np is not None and isinstance(coords3d, np.ndarray):
                    x3, y3, z3 = coords3d[vi].tolist()
                else:
                    x3, y3, z3 = coords3d[vi]
                node_coords3d[nid] = (float(x3), float(y3), float(z3))
                adj[nid] = set()
        # Link endpoints in adjacency
        # Determine quantised node ids for this edge
        # We re-quantise again to map indices to node ids
        if np is not None and isinstance(coords2d, np.ndarray):
            xa, ya = coords2d[va]
            xb, yb = coords2d[vb]
        else:
            xa, ya = coords2d[va]
            xb, yb = coords2d[vb]
        ka = (int(round(xa / stitch_tol)), int(round(ya / stitch_tol)))
        kb = (int(round(xb / stitch_tol)), int(round(yb / stitch_tol)))
        id_a = node_key_to_id[ka]
        id_b = node_key_to_id[kb]
        adj[id_a].add(id_b)
        adj[id_b].add(id_a)
    # Build set to mark used edges (undirected). We key edges as (min_id,max_id).
    used_edges: set[tuple[int, int]] = set()
    loops_raw: list[list[int]] = []
    # Walk adjacency to build loops
    for start_node in list(adj.keys()):
        # For each neighbour of start_node, attempt to start a loop if the edge is unused
        for nb in list(adj[start_node]):
            edge_key = (start_node, nb) if start_node < nb else (nb, start_node)
            if edge_key in used_edges:
                continue
            # Start new loop
            path: list[int] = [start_node]
            prev = start_node
            current = nb
            used_edges.add(edge_key)
            # Use local set for edges used in this loop to avoid reuse; but we rely on global used_edges
            while True:
                path.append(current)
                # Determine next neighbours excluding previous
                neighbours = adj[current]
                # Identify candidate next edges that are unused
                found_next = False
                for next_node in neighbours:
                    if next_node == prev:
                        continue
                    # Determine if edge current-next_node is unused
                    ek = (current, next_node) if current < next_node else (next_node, current)
                    if ek in used_edges:
                        continue
                    # Use this edge
                    used_edges.add(ek)
                    prev, current = current, next_node
                    found_next = True
                    break
                if not found_next:
                    break
                # If we returned to start, loop closed
                if current == start_node:
                    path.append(start_node)
                    break
                # Safety: limit path length
                if len(path) > len(adj) * 2:
                    break
            # Accept only closed loops with at least 4 points (start repeated)
            if len(path) >= 4 and path[0] == path[-1]:
                loops_raw.append(path)
    # Convert node index loops to 3D point loops and filter by area
    loops3d: list[list[tuple[float, float, float]]] = []
    # Determine area threshold relative to model size
    area_threshold = (min_area_fraction * diag * diag)
    # Compute area of each loop and discard degenerate
    for node_loop in loops_raw:
        # Build list of 3D points
        pts = [node_coords3d[nid] for nid in node_loop]
        # Compute area in 2D projection
        uv = []
        for (x3, y3, z3) in pts:
            if pl == "xy":
                uv.append((x3, y3))
            elif pl == "xz":
                uv.append((x3, z3))
            else:
                uv.append((y3, z3))
        # Compute signed area via shoelace
        area_val = 0.0
        m = len(uv)
        for i in range(m - 1):
            x0, y0 = uv[i]
            x1, y1 = uv[i + 1]
            area_val += x0 * y1 - x1 * y0
        area_val *= 0.5
        if abs(area_val) >= area_threshold:
            loops3d.append(pts)
    # Early return if nothing kept
    if not loops3d:
        return []
    # At this stage loops3d may contain holes; perform containment filtering
    # Build metadata for loops: store index, 3d points, 2d uv, signed area, abs area
    loops_info: list[dict[str, object]] = []
    # Use simple area function for 2D
    for pts in loops3d:
        if pl == "xy":
            uv = [(p[0], p[1]) for p in pts]
        elif pl == "xz":
            uv = [(p[0], p[2]) for p in pts]
        else:
            uv = [(p[1], p[2]) for p in pts]
        # Compute signed area
        a_val = 0.0
        n_uv = len(uv)
        for i in range(n_uv - 1):
            x0, y0 = uv[i]
            x1, y1 = uv[i + 1]
            a_val += x0 * y1 - x1 * y0
        a_val *= 0.5
        loops_info.append({
            "pts3d": pts,
            "uv": uv,
            "area": a_val,
            "abs_area": abs(a_val),
        })
    # Sort loops by absolute area descending
    loops_info.sort(key=lambda d: d["abs_area"], reverse=True)  # type: ignore[index]
    n_loops = len(loops_info)
    # Point-in-polygon helper (ray casting). Returns True if pt is inside polygon
    def _point_in_polygon(pt: tuple[float, float], poly: list[tuple[float, float]]) -> bool:
        x, y = pt
        inside = False
        m = len(poly)
        for i in range(m):
            x0, y0 = poly[i]
            x1, y1 = poly[(i + 1) % m]
            # Exclude boundary points: if point lies exactly on edge, treat as outside
            denom = (y1 - y0) * (x - x0) - (x1 - x0) * (y - y0)
            if abs(denom) < 1e-12:
                if min(x0, x1) - 1e-12 <= x <= max(x0, x1) + 1e-12 and min(y0, y1) - 1e-12 <= y <= max(y0, y1) + 1e-12:
                    return False
            if (y0 > y) != (y1 > y):
                x_int = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
                if x_int > x:
                    inside = not inside
        return inside
    contained_flags: list[bool] = [False] * n_loops
    # For each loop, mark contained loops
    for i in range(n_loops):
        if contained_flags[i]:
            continue
        outer_uv = loops_info[i]["uv"]  # type: ignore[index]
        # Use first vertex of inner polygon as test point
        for j in range(i + 1, n_loops):
            if contained_flags[j]:
                continue
            inner_uv = loops_info[j]["uv"]  # type: ignore[index]
            if not inner_uv:
                continue
            test_pt = inner_uv[0]
            try:
                if _point_in_polygon(test_pt, outer_uv):
                    contained_flags[j] = True
            except Exception:
                continue
    # Collect outer loops
    outer_loops: list[list[tuple[float, float, float]]] = []
    for info, is_contained in zip(loops_info, contained_flags):
        if not is_contained:
            outer_loops.append(info["pts3d"])  # type: ignore[index]
    # Respect island_mode
    if island_mode != "multi" and outer_loops:
        return [outer_loops[0]]
    return outer_loops