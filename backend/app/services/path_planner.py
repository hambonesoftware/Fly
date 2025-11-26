"""
Path planning utilities for the flythrough project.

This module provides helper functions to generate and validate
flythrough paths.  In Phase 3 we implement an orbit-style path
generator that uses the model's bounding box and a simple distance
based validator.  Earlier phases used a fixed circular path; that
function remains available for reference but is no longer used in
route handlers.

Functions:
    generate_circular_path(radius: float, num_samples: int) -> (points, length)
        Generate a simple circle in the XY plane.  Used in Phase 1.
    generate_orbit_path(bbox_min: tuple[float, float, float], bbox_max: tuple[float, float, float],
        camera_radius: float, num_samples: int, factor: float) -> list[PathPoint]
        Generate a horizontal circular orbit around the model bounding box.
    validate_path(points: list[PathPoint], bbox_min: tuple[float, float, float],
        bbox_max: tuple[float, float, float], camera_radius: float) -> tuple[bool, float, list[dict]]
        Validate the clearance of a path relative to the model bounding box.
"""

from __future__ import annotations

import math
from typing import List, Tuple, Iterable

from ..api.models import PathPoint

import os
import logging

# Import slicing helpers for unified plane handling and perimeter computation
from .slicing import (
    make_slice_plane,
    compute_slice_perimeter_loop,
    offset_polygon_2d,
    simplify_polyline_rdp,
    choose_simplification_tolerance,
    fit_uniform_closed_bspline,
    sample_bspline,
    lift_uv_to_3d,
    project_point_to_plane_uv,
    _polygon_area_2d,
)

from .geometry import (
    compute_mesh_silhouette_loops,
    compute_mesh_perimeter_polygons,
    compute_raster_perimeter_polygons,
    get_mesh_for_model,
)

import time

# -----------------------------------------------------------------------------
# Geometry helpers for self‑intersection detection
#
# The perimeter path generator offsets a base outline outward to maintain a
# clearance around the model.  In some cases (e.g. highly concave shapes or
# excessive offsets) the offset polygon can self‑intersect.  When this
# happens the path becomes invalid and can lead to confusing results.  To
# mitigate this we detect self‑intersections in 2D and iteratively shrink
# the offset until a valid simple polygon is produced.  The helper functions
# below implement a basic segment intersection test and polygon self‑
# intersection check.  They operate on sequences of (x, y) tuples.

def _orientation(p: tuple[float, float], q: tuple[float, float], r: tuple[float, float]) -> int:
    """Return the orientation of the ordered triplet (p, q, r).

    The function returns:

    * 0 if the points are colinear
    * 1 if they are oriented clockwise
    * 2 if they are oriented counter‑clockwise

    Based on the sign of the cross product of vectors pq and qr.
    """
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if abs(val) < 1e-12:
        return 0
    return 1 if val > 0 else 2


def _on_segment(p: tuple[float, float], q: tuple[float, float], r: tuple[float, float]) -> bool:
    """Return True if point q lies on the segment pr."""
    return (min(p[0], r[0]) - 1e-12 <= q[0] <= max(p[0], r[0]) + 1e-12) and (
        min(p[1], r[1]) - 1e-12 <= q[1] <= max(p[1], r[1]) + 1e-12
    )


def _segments_intersect(
    p1: tuple[float, float],
    q1: tuple[float, float],
    p2: tuple[float, float],
    q2: tuple[float, float],
) -> bool:
    """Check whether two line segments p1q1 and p2q2 intersect.

    Uses the orientation method and handles colinear cases.  Endpoints are
    considered touching.  This function treats the segments as closed.
    """
    o1 = _orientation(p1, q1, p2)
    o2 = _orientation(p1, q1, q2)
    o3 = _orientation(p2, q2, p1)
    o4 = _orientation(p2, q2, q1)
    # General case
    if o1 != o2 and o3 != o4:
        return True
    # Special cases for colinear points
    if o1 == 0 and _on_segment(p1, p2, q1):
        return True
    if o2 == 0 and _on_segment(p1, q2, q1):
        return True
    if o3 == 0 and _on_segment(p2, p1, q2):
        return True
    if o4 == 0 and _on_segment(p2, q1, q2):
        return True
    return False


def _polygon_self_intersects(poly: list[tuple[float, float]]) -> bool:
    """Return True if the closed polygon defined by `poly` self‑intersects.

    The input should be an ordered list of vertices (x, y).  The first and
    last vertices may or may not be the same; this function will treat the
    polygon as closed either way.  Adjacent edges and the first/last edge
    pair are ignored when testing for intersections.
    """
    n = len(poly)
    if n < 4:
        # A triangle cannot self‑intersect
        return False
    # Ensure closed loop by referencing indices modulo n
    for i1 in range(n):
        p1 = poly[i1]
        q1 = poly[(i1 + 1) % n]
        for i2 in range(i1 + 1, n):
            # Adjacent edges share a vertex; skip
            if (i1 == i2) or ((i1 + 1) % n == i2) or (i1 == (i2 + 1) % n):
                continue
            p2 = poly[i2]
            q2 = poly[(i2 + 1) % n]
            if _segments_intersect(p1, q1, p2, q2):
                return True
    return False


# -----------------------------------------------------------------------------
# Rolling‑circle perimeter path helper

def generate_rolling_circle_perimeter_path(
    model_id: str,
    plane: str,
    height_offset: float,
    circle_radius: float,
    island_mode: str,
    perimeter_detail: str | None,
    base_clearance: float,
    samples_per_side: int,
) -> list[list[PathPoint]]:
    """
    Generate one or more closed camera paths corresponding to the locus
    of a circle of radius ``circle_radius`` rolling around the model perimeter.

    The locus of the circle centre is equivalent to offsetting the outer
    perimeter by ``circle_radius + base_clearance``.  This helper
    leverages the raster perimeter extraction pipeline to obtain the
    base loops, projects them into the chosen 2D plane, offsets them
    outward by the effective clearance and fits a closed B‑spline
    through the simplified offset polygon.  Detailed logging is
    performed to aid debugging and performance tuning.

    Args:
        model_id: Identifier of the model whose perimeter is to be used.
        plane: Slice plane name ('xy', 'xz' or 'yz').
        height_offset: Offset along the plane normal where the slice is taken.
        circle_radius: Radius of the camera circle to roll along the perimeter.
        island_mode: 'single' to pick the largest island or 'multi' to return all.
        perimeter_detail: Detail level controlling simplification and sampling.
        base_clearance: Additional clearance to add beyond the circle radius.
        samples_per_side: Legacy sampling resolution used to determine
            minimum sample counts.

    Returns:
        A list of lists of PathPoint objects, each sublist representing a
        closed path.  Returns an empty list if no perimeter could be computed.
    """
    logger = logging.getLogger(__name__)
    prefix = "[RollingCirclePath]"
    # Start overall timer
    t_start = time.perf_counter()
    # Construct the slice plane for projection and lifting
    slice_plane = make_slice_plane(plane, height_offset)
    # Fetch outer perimeter loops using the raster helper.  This returns
    # a tuple (loops, areas, meta).  We are interested in the loops only.
    loops_3d: list[list[tuple[float, float, float]]]
    try:
        result = compute_raster_perimeter_polygons(
            model_id=model_id,
            plane=plane,
            detail=(perimeter_detail or "auto"),
            island_mode=island_mode,
            outline_stitch_percent=None,
        )
        # Unpack loops; older versions may return just loops
        if isinstance(result, tuple) and len(result) >= 1:
            loops_3d = result[0]  # type: ignore[assignment]
        else:
            loops_3d = result  # type: ignore[assignment]
    except Exception as exc:
        logger.error(f"{prefix} Error obtaining raster perimeter polygons: {exc}")
        loops_3d = []
    t_perim = time.perf_counter()
    logger.info(
        f"{prefix} perimeter extraction: {len(loops_3d)} loops in {t_perim - t_start:.4f}s"
    )
    if not loops_3d:
        return []
    # Project loops into UV coordinates on the slice plane.  Compute signed
    # areas to determine orientation.  Retain only valid loops (>=3 points,
    # non‑zero area).  Orient outer loops counter‑clockwise for consistency.
    loops_uv: list[list[tuple[float, float]]] = []
    loops_area: list[float] = []
    for pts3d in loops_3d:
        try:
            uv = [project_point_to_plane_uv(p, slice_plane) for p in pts3d]
        except Exception:
            continue
        if len(uv) < 3:
            continue
        try:
            a = _polygon_area_2d(uv)
        except Exception:
            continue
        if abs(a) < 1e-12:
            continue
        # Enforce CCW orientation for outer loops (positive area)
        if a < 0.0:
            uv = list(reversed(uv))
            a = -a
        loops_uv.append(uv)
        loops_area.append(a)
    t_proj = time.perf_counter()
    logger.info(
        f"{prefix} projection and orientation: {len(loops_uv)} loops in {t_proj - t_perim:.4f}s"
    )
    if not loops_uv:
        return []
    # If island_mode is not multi, retain only the largest loop by area
    if island_mode != "multi":
        idx_max = max(range(len(loops_uv)), key=lambda i: abs(loops_area[i]))
        loops_uv = [loops_uv[idx_max]]
        loops_area = [loops_area[idx_max]]
    # Determine effective offset distance.  Negative radii or clearances
    # collapse to zero.  This distance defines how far from the perimeter
    # the circle centre should travel.
    offset_distance = max(0.0, circle_radius) + max(0.0, base_clearance)
    # Normalise detail for simplification and sampling
    detail_norm = (perimeter_detail or "auto").strip().lower()
    if detail_norm not in {"low", "medium", "high", "auto"}:
        detail_norm = "auto"
    all_paths: list[list[PathPoint]] = []
    # Process each loop independently
    for uv_loop, area_val in zip(loops_uv, loops_area):
        # Offset the polygon outward
        t_off0 = time.perf_counter()
        try:
            offset_uv = offset_polygon_2d(uv_loop, offset_distance, area_hint=area_val)
        except Exception as exc:
            logger.error(f"{prefix} error offsetting polygon: {exc}")
            continue
        t_off1 = time.perf_counter()
        logger.info(
            f"{prefix} offsetting: {len(uv_loop)}→{len(offset_uv)} pts in {t_off1 - t_off0:.4f}s (distance={offset_distance:.4f})"
        )
        # Choose simplification tolerance based on detail
        base_tol = choose_simplification_tolerance(offset_uv)
        if detail_norm == "low":
            tol = base_tol * 2.0
        elif detail_norm == "high":
            tol = base_tol * 0.5
        else:
            tol = base_tol
        t_simp0 = time.perf_counter()
        try:
            simple_uv = simplify_polyline_rdp(offset_uv, tol)
        except Exception as exc:
            logger.error(f"{prefix} error simplifying polygon: {exc}")
            continue
        t_simp1 = time.perf_counter()
        logger.info(
            f"{prefix} simplify: {len(offset_uv)}→{len(simple_uv)} pts in {t_simp1 - t_simp0:.4f}s (tol={tol:.6f})"
        )
        # Determine sample count based on detail and original resolution
        if detail_norm == "low":
            num_samples = max(4 * len(simple_uv), samples_per_side * 2)
        elif detail_norm == "high":
            num_samples = max(4 * len(simple_uv), samples_per_side * 8)
        else:
            num_samples = max(4 * len(simple_uv), samples_per_side * 4)
        # Fit a closed B‑spline and sample uniformly
        t_spline0 = time.perf_counter()
        try:
            spline = fit_uniform_closed_bspline(simple_uv)
            sampled_uv = sample_bspline(spline, num_samples)
        except Exception as exc:
            logger.error(f"{prefix} error fitting/sampling spline: {exc}")
            continue
        t_spline1 = time.perf_counter()
        logger.info(
            f"{prefix} spline fit+sample: {num_samples} samples in {t_spline1 - t_spline0:.4f}s"
        )
        # Lift sampled UV coordinates back into 3D
        t_lift0 = time.perf_counter()
        try:
            pts3d = lift_uv_to_3d(sampled_uv, slice_plane)
        except Exception as exc:
            logger.error(f"{prefix} error lifting UV to 3D: {exc}")
            continue
        t_lift1 = time.perf_counter()
        logger.info(
            f"{prefix} lifting: {len(sampled_uv)} pts in {t_lift1 - t_lift0:.4f}s"
        )
        path_points = [PathPoint(x=p[0], y=p[1], z=p[2]) for p in pts3d]
        all_paths.append(path_points)
    # Total time taken
    t_end = time.perf_counter()
    logger.info(f"{prefix} total time: {t_end - t_start:.4f}s, paths={len(all_paths)}")
    return all_paths

# Strategy identifiers used by the path planner.  These constants
# allow the API layer to remain agnostic about the concrete strings
# used internally and provide a single source of truth.  Keeping the
# strategy names here helps avoid typos when wiring the backend and
# frontend together.
ORBIT_STRATEGY: str = "orbit"
PERIMETER_STRATEGY: str = "perimeter"


def generate_circular_path(radius: float = 2.0, num_samples: int = 64) -> Tuple[List[PathPoint], float]:
    """Generate a circular path in the XY plane.

    Args:
        radius: Radius of the circle.
        num_samples: Number of points to sample along the circle.

    Returns:
        A tuple of (points, length) where `points` is a list of
        PathPoint objects and `length` is the approximate perimeter of
        the circle.
    """
    points: List[PathPoint] = []
    for i in range(num_samples):
        angle = 2 * math.pi * i / num_samples
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = 0.0
        points.append(PathPoint(x=x, y=y, z=z))
    length = 2 * math.pi * radius
    return points, length


def generate_orbit_path(
    bbox_min: Iterable[float],
    bbox_max: Iterable[float],
    camera_radius: float,
    num_samples: int = 128,
    factor: float = 1.5,
) -> List[PathPoint]:
    """Generate a circular orbit path around the model bounding box.

    The path lies in the horizontal plane (constant z) and encircles
    the model at a radius proportional to its largest dimension.

    Args:
        bbox_min: The minimum x, y, z of the model's bounding box.
        bbox_max: The maximum x, y, z of the model's bounding box.
        camera_radius: Minimum clearance distance from the model (unused in this simple implementation).
        num_samples: Number of points to sample along the path.
        factor: Multiplier applied to the largest dimension to determine orbit radius.

    Returns:
        A list of PathPoint objects defining the orbit.
    """
    min_x, min_y, min_z = bbox_min
    max_x, max_y, max_z = bbox_max
    # Compute centre and size of the bounding box
    cx = (min_x + max_x) / 2.0
    cy = (min_y + max_y) / 2.0
    cz = (min_z + max_z) / 2.0
    sx = max_x - min_x
    sy = max_y - min_y
    sz = max_z - min_z
    # Largest dimension
    max_dim = max(sx, sy, sz)
    # Determine orbit radius: scale by factor and add camera radius
    R = max_dim * factor + camera_radius
    points: List[PathPoint] = []
    for i in range(num_samples):
        angle = 2 * math.pi * i / num_samples
        x = cx + R * math.cos(angle)
        y = cy + R * math.sin(angle)
        z = cz  # keep z constant at model centre
        points.append(PathPoint(x=x, y=y, z=z))
    return points


def generate_perimeter_path(
    bbox_min: tuple[float, float, float],
    bbox_max: tuple[float, float, float],
    offset: float,
    samples_per_side: int = 32,
    bulge_factor: float = 0.0,
    smoothness: float = 0.5,
    plane: str = "xy",
    height_offset: float = 0.0,
    model_id: str | None = None,
    detail: str | None = None,
    use_spline_override: bool | None = None,
    use_model_outline: bool | None = None,
    *,
    outline_stitch_percent: float | None = None,
    island_mode: str = "single",
    outline_stitch_tolerance: float | None = None,
    outline_detail: str | None = None,
    # New parameters for version 0.9.11: rolling‑circle perimeter generation
    perimeter_use_rolling_circle: bool = False,
    perimeter_circle_diameter: float | None = None,
) -> list[PathPoint] | list[list[PathPoint]]:
    """Generate a perimeter path around a model cross‑section on an arbitrary plane.

    This function generalises the perimeter path generator so that it can
    operate on any of the principal planes (XY, XZ or YZ) and slice the
    model at an arbitrary offset along the orthogonal axis.  The path
    maintains approximately ``offset`` units of clearance around the
    model's cross‑section on the chosen plane.  Bulging and smoothing
    parameters behave as in earlier phases.

    Args:
        bbox_min: (min_x, min_y, min_z) of the model's axis‑aligned
            bounding box.
        bbox_max: (max_x, max_y, max_z) of the model's axis‑aligned
            bounding box.
        offset: Clearance to maintain around the model in the plane.
        samples_per_side: Legacy sampling resolution used for the
            fallback bounding‑box path.  For slice‑based paths this
            influences the number of samples along the perimeter.
        bulge_factor: Additional outward bulge applied to selected edges.
            Only used when falling back to the bounding‑box path.
        smoothness: Spline interpolation factor (0.0–1.0).  For the
            slice‑based path this value controls whether a B‑spline is
            fitted (smoothness > 0) or the simplified polygon is used
            directly.  For the bounding‑box fallback this controls the
            Catmull–Rom smoothing factor.
        plane: Which plane to use ('xy', 'xz' or 'yz').
        height_offset: Constant coordinate of the selected plane.
        model_id: Optional identifier of the model; if provided and a
            slice perimeter loop can be computed, the slice‑based path
            pipeline will be used.  Otherwise a bounding‑box path is
            generated.

    Returns:
        A list of PathPoint objects describing the perimeter path.
    """
    import math

    # Initialise metadata container on the function for later retrieval by the API layer.
    # This attribute is mutated below when a slice‑based or bounding‑box path is generated.
    # Consumers such as the API layer can inspect ``generate_perimeter_path._last_meta``
    # after calling this function to enrich the response metadata.
    generate_perimeter_path._last_meta = None  # type: ignore[attr-defined]

    # Attempt to compute a slice‑based or outline‑based perimeter path when a model_id is provided.
    # This pipeline either slices the model mesh (legacy behaviour) or uses the
    # HLR‑derived model outline as the base curve, then offsets the outer
    # perimeter, simplifies it, optionally fits a B‑spline and lifts the result
    # back into 3D.  If any step fails or no loop is found, the code falls back
    # to the legacy bounding‑box perimeter implementation below.
    if model_id is not None:
        try:
            slice_plane = make_slice_plane(plane, height_offset)
            # ------------------------------------------------------------------
            # Rolling‑circle perimeter mode
            #
            # When the request indicates that a rolling‑circle path should be
            # generated and the model outline is being used, attempt to compute
            # the locus of a circle rolling around the perimeter.  This path
            # corresponds to the perimeter offset by the circle radius plus
            # clearance.  If successful, return the generated path(s) and
            # populate metadata accordingly.  On failure, fall through to the
            # standard perimeter generation pipeline below.
            if use_model_outline and perimeter_use_rolling_circle:
                try:
                    # Determine circle radius.  Use half of the provided
                    # diameter when specified; otherwise choose a default
                    # proportionate to the model size.
                    if perimeter_circle_diameter is not None and perimeter_circle_diameter > 0.0:
                        circle_radius = perimeter_circle_diameter * 0.5
                    else:
                        try:
                            mesh_for_size = get_mesh_for_model(model_id)
                            bbox_size = mesh_for_size.bbox
                            dx = bbox_size.max[0] - bbox_size.min[0]
                            dy = bbox_size.max[1] - bbox_size.min[1]
                            dz = bbox_size.max[2] - bbox_size.min[2]
                            characteristic = max(dx, dy, dz) or 1.0
                            circle_radius = characteristic / 10.0
                        except Exception:
                            circle_radius = 1.0
                    # Generate the rolling‑circle path(s)
                    rc_paths = generate_rolling_circle_perimeter_path(
                        model_id=model_id,
                        plane=plane,
                        height_offset=height_offset,
                        circle_radius=circle_radius,
                        island_mode=island_mode,
                        perimeter_detail=detail,
                        base_clearance=offset,
                        samples_per_side=samples_per_side,
                    )
                    if rc_paths:
                        # Record metadata on the function for API consumption
                        norm_detail = (detail or "auto").strip().lower()
                        if norm_detail not in {"low", "medium", "high", "auto"}:
                            norm_detail = "auto"
                        generate_perimeter_path._last_control_points_3d = None  # type: ignore[attr-defined]
                        generate_perimeter_path._last_meta = {
                            "source": "rolling-circle",
                            "perimeterSource": "outline",
                            "circleRadius": circle_radius,
                            "islandMode": island_mode,
                            "detail": norm_detail,
                            "num_islands": len(rc_paths),
                        }  # type: ignore[attr-defined]
                        # Return paths depending on island mode
                        if island_mode == "multi":
                            return rc_paths
                        else:
                            return rc_paths[0]
                except Exception as exc:
                    # Log and fall back to standard perimeter logic
                    logging.getLogger(__name__).exception(
                        "Rolling‑circle perimeter generation failed: %s", exc
                    )
            # --- Begin new multi-loop perimeter logic ---
            try:
                # Gather base loops for both outline and slice modes
                base_loops_uv: list[list[tuple[float, float]]] = []
                base_loops_area: list[float] = []
                if use_model_outline:
                    # Use the raster‑based perimeter loops instead of the silhouette.  The
                    # perimeter helper collapses the mesh into a raster grid, extracts
                    # contours via marching squares, filters interior islands and returns
                    # the outer loops.  We request all islands here (multi) and let
                    # island_mode below control selection of the largest vs all.
                    try:
                        loops_3d, _areas_unused, _meta_unused = compute_raster_perimeter_polygons(
                            model_id=model_id,
                            plane=plane,
                            detail=detail,
                            island_mode="multi",
                            outline_stitch_percent=outline_stitch_percent,
                        )
                    except Exception:
                        loops_3d = []
                    if loops_3d:
                        from .slicing import _polygon_area_2d  # local import
                        for pts3d in loops_3d:
                            try:
                                uv = [project_point_to_plane_uv(p, slice_plane) for p in pts3d]
                                area_val = _polygon_area_2d(uv)
                                if abs(area_val) > 0.0:
                                    base_loops_uv.append(uv)
                                    base_loops_area.append(area_val)
                            except Exception:
                                continue
                else:
                    loop = compute_slice_perimeter_loop(model_id, slice_plane)
                    if loop is not None and loop.points_2d:
                        base_loops_uv.append(loop.points_2d)
                        base_loops_area.append(loop.area)
                # Proceed only if loops found
                if base_loops_uv:
                    # Single vs multi mode
                    if island_mode != "multi":
                        idx2 = max(range(len(base_loops_uv)), key=lambda i: abs(base_loops_area[i]))
                        base_loops_uv = [base_loops_uv[idx2]]
                        base_loops_area = [base_loops_area[idx2]]
                    # Normalise detail
                    detail_norm_local = (detail or "auto").strip().lower()
                    if detail_norm_local not in {"low", "medium", "high", "auto"}:
                        detail_norm_local = "auto"
                    if use_spline_override is not None:
                        use_spline_local = use_spline_override
                    else:
                        use_spline_local = smoothness > 0.0
                    all_paths: list[list[PathPoint]] = []
                    any_self = False
                    any_adjusted = False
                    base_outline_points_multi: list[list[PathPoint]] = []
                    last_simple_uv: list[tuple[float, float]] | None = None
                    for uv_loop, area_val in zip(base_loops_uv, base_loops_area):
                        try:
                            loop3d = lift_uv_to_3d(uv_loop, slice_plane)
                            base_outline_pts = [PathPoint(x=p[0], y=p[1], z=p[2]) for p in loop3d]
                            base_outline_points_multi.append(base_outline_pts)
                        except Exception:
                            base_outline_points_multi.append([])
                        effective_offset = offset
                        adjusted = False
                        self_intersects = False
                        offset_uv_local = offset_polygon_2d(uv_loop, effective_offset, area_hint=area_val)
                        attempts = 0
                        while _polygon_self_intersects(offset_uv_local) and attempts < 5:
                            self_intersects = True
                            adjusted = True
                            effective_offset *= 0.5
                            if effective_offset <= 1e-6:
                                break
                            offset_uv_local = offset_polygon_2d(uv_loop, effective_offset, area_hint=area_val)
                            attempts += 1
                        any_self = any_self or self_intersects
                        any_adjusted = any_adjusted or adjusted
                        base_tol_local = choose_simplification_tolerance(offset_uv_local)
                        if detail_norm_local == "low":
                            tol_local = base_tol_local * 2.0
                        elif detail_norm_local == "high":
                            tol_local = base_tol_local * 0.5
                        else:
                            tol_local = base_tol_local
                        simple_uv_local = simplify_polyline_rdp(offset_uv_local, tol_local)
                        last_simple_uv = simple_uv_local
                        if detail_norm_local == "low":
                            num_samples_local = max(4 * len(simple_uv_local), samples_per_side * 2)
                        elif detail_norm_local == "high":
                            num_samples_local = max(4 * len(simple_uv_local), samples_per_side * 8)
                        else:
                            num_samples_local = max(4 * len(simple_uv_local), samples_per_side * 4)
                        if use_spline_local:
                            spline_local = fit_uniform_closed_bspline(simple_uv_local)
                            sampled_uv_local = sample_bspline(spline_local, num_samples_local)
                        else:
                            if len(simple_uv_local) >= 2:
                                sampled_uv_local: list[tuple[float, float]] = []
                                for i_ in range(len(simple_uv_local)):
                                    p0_ = simple_uv_local[i_]
                                    p1_ = simple_uv_local[(i_ + 1) % len(simple_uv_local)]
                                    sampled_uv_local.append(p0_)
                                    seg_len_local = math.hypot(p1_[0] - p0_[0], p1_[1] - p0_[1])
                                    steps_local = max(1, int(seg_len_local / (effective_offset or 1.0) * 2))
                                    for s_ in range(1, steps_local):
                                        t_ = s_ / steps_local
                                        sampled_uv_local.append((p0_[0] * (1 - t_) + p1_[0] * t_, p0_[1] * (1 - t_) + p1_[1] * t_))
                            else:
                                sampled_uv_local = simple_uv_local
                        pts3d_local = lift_uv_to_3d(sampled_uv_local, slice_plane)
                        path_points_local = [PathPoint(x=p[0], y=p[1], z=p[2]) for p in pts3d_local]
                        all_paths.append(path_points_local)
                    # Build metadata
                    meta: dict[str, object] = {
                        "source": "outline" if use_model_outline else "slice",
                        "detail": detail_norm_local,
                        "num_islands": len(all_paths),
                    }
                    if use_model_outline:
                        meta["perimeterSource"] = "outline"
                        if outline_detail is not None:
                            meta["outlineDetail"] = outline_detail
                        if outline_stitch_percent is not None:
                            meta["stitch_percent"] = outline_stitch_percent
                    else:
                        meta["perimeterSource"] = "slice"
                    meta["selfIntersection"] = any_self
                    meta["offsetAdjusted"] = any_adjusted
                    if island_mode != "multi" and all_paths:
                        meta["effectiveOffset"] = offset if not any_adjusted else None
                    if base_outline_points_multi:
                        if island_mode == "multi":
                            meta["baseOutlineMulti"] = [
                                [ {"x": p.x, "y": p.y, "z": p.z} for p in outline_pts ]
                                for outline_pts in base_outline_points_multi
                            ]
                        else:
                            outline0 = base_outline_points_multi[0]
                            meta["baseOutline"] = [ {"x": p.x, "y": p.y, "z": p.z} for p in outline0 ]
                            meta["baseOutlinePointCount"] = len(outline0)
                    generate_perimeter_path._last_meta = meta  # type: ignore[attr-defined]
                    if island_mode != "multi" and last_simple_uv is not None:
                        try:
                            ctrl_pts3d_local = lift_uv_to_3d(last_simple_uv, slice_plane)
                            generate_perimeter_path._last_control_points_3d = [ PathPoint(x=p[0], y=p[1], z=p[2]) for p in ctrl_pts3d_local ]
                        except Exception:
                            generate_perimeter_path._last_control_points_3d = None  # type: ignore[attr-defined]
                    else:
                        generate_perimeter_path._last_control_points_3d = None  # type: ignore[attr-defined]
                    if island_mode == "multi":
                        return all_paths
                    else:
                        return all_paths[0] if all_paths else []
            except Exception:
                pass  # if new logic fails, fall through to legacy behaviour
            # --- End new multi-loop perimeter logic ---

            base_points_uv: list[tuple[float, float]] | None = None
            base_area: float | None = None

            if use_model_outline:
                # Use the raster‑based perimeter loops as the starting curve.  The
                # perimeter helper collapses the mesh into a raster grid, extracts
                # contour loops via marching squares and returns all outer islands.
                try:
                    loops, _areas_unused, _meta_unused = compute_raster_perimeter_polygons(
                        model_id=model_id,
                        plane=plane,
                        detail=detail,
                        island_mode="multi",
                        outline_stitch_percent=outline_stitch_percent,
                    )
                except Exception:
                    loops = []
                if loops:
                    # Choose the largest loop by projected area
                    best_idx = None
                    best_area = 0.0
                    for i, pts3d in enumerate(loops):
                        try:
                            uv_local = [project_point_to_plane_uv(p, slice_plane) for p in pts3d]
                            a_val = _polygon_area_2d(uv_local)
                            if abs(a_val) > abs(best_area):
                                best_area = a_val
                                best_idx = i
                        except Exception:
                            continue
                    if best_idx is None:
                        best_idx = 0
                    outline_pts_3d = loops[best_idx]
                    base_points_uv = [project_point_to_plane_uv(p, slice_plane) for p in outline_pts_3d]
                    base_area = _polygon_area_2d(base_points_uv)
            else:
                # Original behaviour: slice the mesh and use the outer loop
                loop = compute_slice_perimeter_loop(model_id, slice_plane)
                if loop is not None and loop.points_2d:
                    base_points_uv = loop.points_2d
                    base_area = loop.area

            if base_points_uv:
                # Create and store base outline in 3D for metadata and preview.
                try:
                    base_outline_3d = lift_uv_to_3d(base_points_uv, slice_plane)
                    base_outline_pts = [PathPoint(x=p[0], y=p[1], z=p[2]) for p in base_outline_3d]
                    # Store on function for retrieval by API layer
                    generate_perimeter_path._last_base_outline_3d = base_outline_pts  # type: ignore[attr-defined]
                except Exception:
                    # In case lifting fails, clear previous outline
                    generate_perimeter_path._last_base_outline_3d = None  # type: ignore[attr-defined]
                    base_outline_pts = None

                # Offset the base loop outward by the requested clearance.  Provide the
                # signed area as a hint to retain the correct winding.  If the resulting
                # polygon self‑intersects, iteratively reduce the offset until
                # intersections are resolved.  Track the effective offset and whether
                # adjustments occurred.
                effective_offset = offset
                adjusted = False
                self_intersects = False
                offset_uv = offset_polygon_2d(
                    base_points_uv,
                    effective_offset,
                    area_hint=base_area,
                )
                # Detect self‑intersection and shrink offset if necessary
                # Limit to a fixed number of attempts to avoid infinite loops.
                attempts = 0
                while _polygon_self_intersects(offset_uv) and attempts < 5:
                    self_intersects = True
                    adjusted = True
                    effective_offset *= 0.5
                    # Break if offset becomes too small relative to model size
                    if effective_offset <= 1e-6:
                        break
                    offset_uv = offset_polygon_2d(
                        base_points_uv,
                        effective_offset,
                        area_hint=base_area,
                    )
                    attempts += 1

                # Determine detail level for simplification. Use "auto" when
                # detail is None or invalid.
                detail_norm = (detail or "auto").strip().lower()
                if detail_norm not in {"low", "medium", "high", "auto"}:
                    detail_norm = "auto"

                # Compute base tolerance from polygon size
                base_tol = choose_simplification_tolerance(offset_uv)
                if detail_norm == "low":
                    tol = base_tol * 2.0
                elif detail_norm == "high":
                    tol = base_tol * 0.5
                else:
                    # medium or auto
                    tol = base_tol

                # Simplify polygon
                simple_uv = simplify_polyline_rdp(offset_uv, tol)

                # Determine number of samples based on detail level
                if detail_norm == "low":
                    num_samples = max(4 * len(simple_uv), samples_per_side * 2)
                elif detail_norm == "high":
                    num_samples = max(4 * len(simple_uv), samples_per_side * 8)
                else:
                    num_samples = max(4 * len(simple_uv), samples_per_side * 4)

                # Decide whether to fit a spline
                if use_spline_override is not None:
                    use_spline = use_spline_override
                else:
                    use_spline = smoothness > 0.0

                if use_spline:
                    spline = fit_uniform_closed_bspline(simple_uv)
                    sampled_uv = sample_bspline(spline, num_samples)
                else:
                    # Directly resample the simplified polygon to ensure a
                    # roughly uniform spacing of points.
                    if len(simple_uv) >= 2:
                        sampled_uv = []
                        for i in range(len(simple_uv)):
                            p0 = simple_uv[i]
                            p1 = simple_uv[(i + 1) % len(simple_uv)]
                            sampled_uv.append(p0)
                            # Insert intermediate points based on length. Use effective_offset
                            seg_len = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                            steps = max(1, int(seg_len / (effective_offset or 1.0) * 2))
                            for s in range(1, steps):
                                t = s / steps
                                sampled_uv.append(
                                    (p0[0] * (1 - t) + p1[0] * t, p0[1] * (1 - t) + p1[1] * t)
                                )
                    else:
                        sampled_uv = simple_uv

                # Lift back to 3D and construct PathPoint list
                points_3d = lift_uv_to_3d(sampled_uv, slice_plane)
                result = [PathPoint(x=p[0], y=p[1], z=p[2]) for p in points_3d]

                # Record metadata for inspection by the API layer.
                meta: dict[str, object] = {
                    "source": "outline" if use_model_outline else "slice",
                    "detail": detail_norm,
                    "samples": len(result),
                }
                # Record base outline in metadata if available
                if base_outline_pts:
                    meta["baseOutline"] = [
                        {"x": p.x, "y": p.y, "z": p.z} for p in base_outline_pts
                    ]
                    meta["baseOutlinePointCount"] = len(base_outline_pts)
                # Normalise perimeterSource so the frontend can always
                # report how the perimeter was obtained.
                if use_model_outline:
                    meta["perimeterSource"] = "outline"
                    if outline_detail is not None:
                        meta["outlineDetail"] = outline_detail
                    if outline_stitch_tolerance is not None:
                        meta["outlineStitchTolerance"] = outline_stitch_tolerance
                else:
                    meta["perimeterSource"] = "slice"
                # Self‑intersection and offset adjustment flags
                meta["selfIntersection"] = self_intersects
                meta["offsetAdjusted"] = adjusted
                meta["effectiveOffset"] = effective_offset

                generate_perimeter_path._last_meta = meta  # type: ignore[attr-defined]

                try:
                    ctrl_pts3d = lift_uv_to_3d(simple_uv, slice_plane)
                    generate_perimeter_path._last_control_points_3d = [
                        PathPoint(x=p[0], y=p[1], z=p[2]) for p in ctrl_pts3d
                    ]
                except Exception:
                    generate_perimeter_path._last_control_points_3d = None  # type: ignore[attr-defined]

                return result
        except Exception as exc:
            # Log error and fall back
            logging.getLogger(__name__).exception(
                "Error computing perimeter for model_id=%s (use_model_outline=%s): %s",
                model_id,
                use_model_outline,
                exc,
            )

# -------------------------------------------------------------------------
    # Fallback: legacy bounding‑box rounded rectangle path.  This code path is
    # executed when no model_id is provided or when the slice pipeline fails.
    # After computing the path, metadata is recorded to indicate a bbox fallback.


    # Create a SlicePlane instance for unified plane handling.  This
    # normalises the plane string, applies a default when needed and
    # encapsulates the offset, origin and normal.  Debug logging is
    # performed within ``make_slice_plane`` when SLICE_DEBUG is set.
    slice_plane = make_slice_plane(plane, height_offset)
    # Determine axis indices based on the slice plane type.  Each tuple
    # represents (a1, a2, ac) where a1 and a2 are the axes used to
    # construct the rectangle and ac is the axis that remains constant.
    if slice_plane.plane_type == "xy":
        i1, i2, ic = 0, 1, 2
    elif slice_plane.plane_type == "xz":
        i1, i2, ic = 0, 2, 1
    else:  # "yz"
        i1, i2, ic = 1, 2, 0
    # Extract bounding extents along selected axes
    min_a1 = bbox_min[i1]
    max_a1 = bbox_max[i1]
    min_a2 = bbox_min[i2]
    max_a2 = bbox_max[i2]
    # Constant coordinate for the plane uses the offset from the SlicePlane
    const_val = slice_plane.offset
    # Compute outer rectangle extents by offsetting
    bottom_a2 = min_a2 - offset
    top_a2 = max_a2 + offset
    left_a1 = min_a1 - offset
    right_a1 = max_a1 + offset
    # Determine arc sampling granularity; ensure a minimum number of points
    arc_steps = max(8, samples_per_side // 4)
    # Determine which edges should receive bulge (apply to longer dimension).
    d_a1 = max_a1 - min_a1
    d_a2 = max_a2 - min_a2
    bulge_bottom = bulge_top = False
    bulge_left = bulge_right = False
    if bulge_factor > 0.0:
        if d_a1 >= d_a2:
            # Bulge along edges parallel to the a1 axis (i.e. bottom and top)
            bulge_bottom = bulge_top = True
        else:
            # Bulge along edges parallel to the a2 axis (i.e. left and right)
            bulge_left = bulge_right = True
    # Compute centre in the plane for bulge direction
    c_a1 = 0.5 * (min_a1 + max_a1)
    c_a2 = 0.5 * (min_a2 + max_a2)
    # Helper to apply bulge to a point in the 2D plane
    def apply_bulge(a1: float, a2: float, t: float, apply: bool) -> tuple[float, float]:
        if not apply or bulge_factor <= 0.0:
            return a1, a2
        bulge = bulge_factor * math.sin(math.pi * t)
        # Direction from centre to current point
        v1 = a1 - c_a1
        v2 = a2 - c_a2
        length = math.hypot(v1, v2)
        if length > 0.0:
            a1 += bulge * (v1 / length)
            a2 += bulge * (v2 / length)
        return a1, a2
    # Internal function to build a PathPoint from a1,a2 and const
    def make_point(a1: float, a2: float) -> PathPoint:
        coords = [0.0, 0.0, 0.0]
        coords[i1] = a1
        coords[i2] = a2
        coords[ic] = const_val
        return PathPoint(x=coords[0], y=coords[1], z=coords[2])
    # Build base polygon with optional bulge on selected edges
    base_points: list[PathPoint] = []
    # Bottom side (from a1=min_a1 to a1=max_a1 at a2=bottom_a2)
    for i in range(samples_per_side):
        t = i / samples_per_side
        a1 = min_a1 + (max_a1 - min_a1) * t
        a2 = bottom_a2
        a1, a2 = apply_bulge(a1, a2, t, bulge_bottom)
        base_points.append(make_point(a1, a2))
    # Bottom‑right corner arc (centre at (max_a1, min_a2))
    corner_a1 = max_a1
    corner_a2 = min_a2
    for i in range(arc_steps):
        theta = math.radians(270.0 + 90.0 * i / arc_steps)
        a1 = corner_a1 + offset * math.cos(theta)
        a2 = corner_a2 + offset * math.sin(theta)
        base_points.append(make_point(a1, a2))
    # Right side (from a2=min_a2 to a2=max_a2 at a1=right_a1)
    for i in range(samples_per_side):
        t = i / samples_per_side
        a2 = min_a2 + (max_a2 - min_a2) * t
        a1 = right_a1
        a1, a2 = apply_bulge(a1, a2, t, bulge_right)
        base_points.append(make_point(a1, a2))
    # Top‑right corner arc (centre at (max_a1, max_a2))
    corner_a1 = max_a1
    corner_a2 = max_a2
    for i in range(arc_steps):
        theta = math.radians(0.0 + 90.0 * i / arc_steps)
        a1 = corner_a1 + offset * math.cos(theta)
        a2 = corner_a2 + offset * math.sin(theta)
        base_points.append(make_point(a1, a2))
    # Top side (from a1=max_a1 to a1=min_a1 at a2=top_a2)
    for i in range(samples_per_side):
        t = i / samples_per_side
        a1 = max_a1 - (max_a1 - min_a1) * t
        a2 = top_a2
        a1, a2 = apply_bulge(a1, a2, t, bulge_top)
        base_points.append(make_point(a1, a2))
    # Top‑left corner arc (centre at (min_a1, max_a2))
    corner_a1 = min_a1
    corner_a2 = max_a2
    for i in range(arc_steps):
        theta = math.radians(90.0 + 90.0 * i / arc_steps)
        a1 = corner_a1 + offset * math.cos(theta)
        a2 = corner_a2 + offset * math.sin(theta)
        base_points.append(make_point(a1, a2))
    # Left side (from a2=max_a2 to a2=min_a2 at a1=left_a1)
    for i in range(samples_per_side):
        t = i / samples_per_side
        a2 = max_a2 - (max_a2 - min_a2) * t
        a1 = left_a1
        a1, a2 = apply_bulge(a1, a2, t, bulge_left)
        base_points.append(make_point(a1, a2))
    # Bottom‑left corner arc (centre at (min_a1, min_a2))
    corner_a1 = min_a1
    corner_a2 = min_a2
    for i in range(arc_steps):
        theta = math.radians(180.0 + 90.0 * i / arc_steps)
        a1 = corner_a1 + offset * math.cos(theta)
        a2 = corner_a2 + offset * math.sin(theta)
        base_points.append(make_point(a1, a2))
    # Ensure closed loop by appending starting point
    if base_points:
        base_points.append(base_points[0])
    # If no smoothing requested or less than 4 points, record metadata and return base path
    if smoothness <= 0.0 or len(base_points) < 4:
        generate_perimeter_path._last_meta = {
            "perimeterSource": "bboxFallback",
            "simplificationTolerance": 0.0,
            "splined": False,
            "controlPointCount": 0,
            "sampledPointCount": len(base_points),
        }
        return base_points
    # Remove duplicate last element for control polygon
    ctrl = base_points[:-1]
    n = len(ctrl)
    # Determine samples per segment based on smoothness (1–4 samples per original segment)
    max_extra = 4
    samples_per_segment = max(1, int(1 + smoothness * (max_extra - 1)))
    smooth_points: list[PathPoint] = []
    # Precompute 2D plane coordinates for Catmull–Rom interpolation
    ctrl_plane: list[tuple[float, float]] = []
    for p in ctrl:
        if i1 == 0:
            a1_coord = p.x
        elif i1 == 1:
            a1_coord = p.y
        else:
            a1_coord = p.z
        if i2 == 0:
            a2_coord = p.x
        elif i2 == 1:
            a2_coord = p.y
        else:
            a2_coord = p.z
        ctrl_plane.append((a1_coord, a2_coord))
    for i in range(n):
        p0_a1, p0_a2 = ctrl_plane[(i - 1) % n]
        p1_a1, p1_a2 = ctrl_plane[i]
        p2_a1, p2_a2 = ctrl_plane[(i + 1) % n]
        p3_a1, p3_a2 = ctrl_plane[(i + 2) % n]
        for j in range(samples_per_segment):
            t = j / samples_per_segment
            # Catmull–Rom interpolation in the 2D plane
            a1 = 0.5 * (
                (2 * p1_a1)
                + (-p0_a1 + p2_a1) * t
                + (2 * p0_a1 - 5 * p1_a1 + 4 * p2_a1 - p3_a1) * t * t
                + (-p0_a1 + 3 * p1_a1 - 3 * p2_a1 + p3_a1) * t * t * t
            )
            a2 = 0.5 * (
                (2 * p1_a2)
                + (-p0_a2 + p2_a2) * t
                + (2 * p0_a2 - 5 * p1_a2 + 4 * p2_a2 - p3_a2) * t * t
                + (-p0_a2 + 3 * p1_a2 - 3 * p2_a2 + p3_a2) * t * t * t
            )
            smooth_points.append(make_point(a1, a2))
    # Close loop by repeating first smooth point
    if smooth_points:
        smooth_points.append(smooth_points[0])
    # Record fallback metadata for smoothed path
    generate_perimeter_path._last_meta = {
        "perimeterSource": "bboxFallback",
        "simplificationTolerance": 0.0,
        "splined": False,
        "controlPointCount": 0,
        "sampledPointCount": len(smooth_points),
    }
    return smooth_points


def validate_path(
    points: Iterable[PathPoint],
    bbox_min: Iterable[float],
    bbox_max: Iterable[float],
    camera_radius: float,
) -> Tuple[bool, float, List[dict]]:
    """Validate that each point maintains clearance from the model.

    The model is approximated by its axis-aligned bounding box.  The
    clearance for a point is the minimum distance from the point to
    the bounding box surfaces.  If a point lies inside the box the
    clearance is zero.  A path is valid if all points have clearance
    strictly greater than the camera radius.

    Args:
        points: Iterable of PathPoint objects defining the path.
        bbox_min: Minimum x, y, z of the bounding box.
        bbox_max: Maximum x, y, z of the bounding box.
        camera_radius: The required clearance from the model.

    Returns:
        A tuple `(valid, min_clearance, violations)` where:
          - valid is True if all points meet the clearance requirement.
          - min_clearance is the minimum clearance across all points.
          - violations is a list of dictionaries with details of
            points that violate the clearance requirement.
    """
    min_x, min_y, min_z = bbox_min
    max_x, max_y, max_z = bbox_max
    min_clearance: float = float('inf')
    violations: List[dict] = []
    for idx, p in enumerate(points):
        # Distance to each pair of faces along axes
        # If inside along an axis, distance to that axis face is negative
        dx = 0.0
        if p.x < min_x:
            dx = min_x - p.x
        elif p.x > max_x:
            dx = p.x - max_x
        else:
            dx = -min(p.x - min_x, max_x - p.x)
        dy = 0.0
        if p.y < min_y:
            dy = min_y - p.y
        elif p.y > max_y:
            dy = p.y - max_y
        else:
            dy = -min(p.y - min_y, max_y - p.y)
        dz = 0.0
        if p.z < min_z:
            dz = min_z - p.z
        elif p.z > max_z:
            dz = p.z - max_z
        else:
            dz = -min(p.z - min_z, max_z - p.z)
        # Clearance is the smallest positive distance to the bounding box
        # If point lies inside the box along all axes, dx, dy, dz will be negative
        axis_distances = [abs(val) for val in (dx, dy, dz) if val > 0]
        if axis_distances:
            clearance = min(axis_distances)
        else:
            clearance = 0.0
        # Track minimum clearance
        if clearance < min_clearance:
            min_clearance = clearance
        # Check if this point violates the clearance requirement
        if clearance <= camera_radius:
            violations.append({
                "index": idx,
                "point": {"x": p.x, "y": p.y, "z": p.z},
                "clearance": clearance,
            })
    valid = len(violations) == 0
    if min_clearance == float('inf'):
        min_clearance = 0.0
    return valid, min_clearance, violations