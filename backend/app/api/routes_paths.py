"""
Routes for path creation, validation and export.

The endpoints in this router provide stub implementations for
generating a flythrough path, validating a path, and exporting it as
CSV.  In Phase 1 the generated path is a simple circle around the
origin and validation always succeeds.  Later phases will use
geometry to compute and validate paths.
"""

from __future__ import annotations

import io
import uuid
import csv
from typing import Dict
import math

from fastapi import APIRouter, HTTPException, Response

from .models import (
    PathCreateRequest,
    PathResponse,
    PathValidateRequest,
    PathValidateResponse,
    PathPoint,
)
from ..services.path_planner import (
    generate_circular_path,
    generate_orbit_path,
    generate_perimeter_path,
    ORBIT_STRATEGY,
    PERIMETER_STRATEGY,
    validate_path as validate_path_points,
)
from ..services.geometry import (
    get_mesh_for_model,
    load_shape_for_model,
    compute_bounding_box,
    compute_mesh_silhouette_loops,
)

# Import additional services for outline extraction
from ..services.slicing import make_slice_plane, compute_slice_perimeter_loop, lift_uv_to_3d, project_point_to_plane_uv
from .models import OutlineResponse, PathPoint


router = APIRouter()

# In‑memory registry of paths keyed by pathId.  Each entry maps to a
# dictionary containing the associated modelId, list of PathPoint
# objects and metadata.  This simplistic approach is sufficient for
# Phase 1 but may be replaced with persistent storage in later
# versions.
path_registry: Dict[str, Dict] = {}

# Maximum number of points allowed in a generated path.
# Paths exceeding this cap will be downsampled to improve performance.
MAX_PATH_POINTS: int = 5000


@router.post(
    "/models/{model_id}/paths",
    response_model=PathResponse,
    status_code=201,
)
async def create_path(model_id: str, body: PathCreateRequest) -> PathResponse:
    """Generate a flythrough path for the given model.

    This endpoint uses the model's bounding box to compute a safe orbit
    path around it.  The radius of the orbit scales with the largest
    dimension of the model and the requested camera radius.  The
    generated path is stored in memory for subsequent validation and
    export.

    Args:
        model_id: Identifier of the model for which to generate the path.
        body: Parameters for path creation including cameraRadius,
            samplingResolution and strategy (currently only 'orbit' is
            supported).

    Returns:
        PathResponse: A new path orbiting around the model's bounding box.
    """
    # Determine number of samples for path generation.  For the orbit
    # strategy we use the first element of the sampling resolution
    # multiplied by four to produce a smooth circle.  For the perimeter
    # strategy we'll derive the per‑side samples from the sampling
    # resolution below.  Ensure a reasonable minimum of 64 overall.
    num_samples = max(64, body.samplingResolution[0] * 4)
    # Attempt to load the shape and compute its bounding box.  If OCC is
    # unavailable or an error occurs, fall back to using the mesh's
    # bounding box (which itself falls back to a unit cube if needed).
    bbox_min: tuple[float, float, float]
    bbox_max: tuple[float, float, float]
    try:
        shape = load_shape_for_model(model_id)
        if shape is not None:
            min_xyz, max_xyz = compute_bounding_box(shape)
            bbox_min = tuple(min_xyz)
            bbox_max = tuple(max_xyz)
        else:
            raise Exception("Shape loading returned None")
    except Exception:
        # Fallback: compute bounding box from mesh (which may be a stub)
        mesh = get_mesh_for_model(model_id)
        bbox_min = tuple(mesh.bbox.min)
        bbox_max = tuple(mesh.bbox.max)
    # Decide which path generation strategy to use based on the request
    strategy = body.strategy
    points: list[PathPoint]
    length: float
    minkowski_boundary_loops = None
    if strategy == PERIMETER_STRATEGY:
        # Derive per‑side sampling: use one quarter of the requested
        # resolution for each straight segment, but enforce a minimum.
        samples_per_side = max(8, body.samplingResolution[0] // 4)
        # Read optional plane and heightOffset parameters.  Defaults are
        # provided by the PathCreateRequest model so getattr will
        # succeed even if the frontend omits the fields.
        plane = getattr(body, "plane", "xy")
        height_offset = getattr(body, "heightOffset", 0.0)
        # Read detail level and spline override parameters.  Use getattr
        # to provide None when not supplied.
        detail = getattr(body, "perimeterDetail", None)
        use_spline_override = getattr(body, "useSpline", None)
        use_model_outline = getattr(body, "useModelOutline", False)
        outline_stitch_tol = getattr(body, "outlineStitchTolerance", None)
        outline_detail = getattr(body, "outlineDetail", None)
        outline_stitch_percent = getattr(body, "outlineStitchPercent", None)
        island_mode = getattr(body, "perimeterIslandMode", "single")
        # Generate a perimeter path; may return a single list of PathPoints or a list
        # of lists when island_mode="multi".  Flatten to a single path for now to
        # preserve the original API contract.  The metadata exposed via
        # generate_perimeter_path._last_meta will still indicate the number of islands.
        # Determine whether to use the new rolling‑circle perimeter mode and the circle diameter.
        perimeter_use_rolling_circle = getattr(body, "perimeterUseRollingCircle", False)
        perimeter_circle_diameter = getattr(body, "perimeterCircleDiameter", None)
        path_result = generate_perimeter_path(
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            offset=body.cameraRadius,
            samples_per_side=samples_per_side,
            bulge_factor=getattr(body, "bulgeFactor", 0.0),
            smoothness=getattr(body, "smoothness", 0.5),
            plane=plane,
            height_offset=height_offset,
            model_id=model_id,
            detail=detail,
            use_spline_override=use_spline_override,
            use_model_outline=use_model_outline,
            outline_stitch_percent=outline_stitch_percent,
            island_mode=island_mode,
            # Pass through rolling‑circle parameters to the planner.
            perimeter_use_rolling_circle=perimeter_use_rolling_circle,
            perimeter_circle_diameter=perimeter_circle_diameter,
        )
        # Flatten multi‑path result
        if path_result and isinstance(path_result[0], list):
            # path_result is a list of lists; choose the first path for compatibility
            points = path_result[0]
        else:
            points = path_result  # type: ignore[assignment]
        # Compute length by summing Euclidean distances between consecutive points.
        length = 0.0
        for p0, p1 in zip(points, points[1:]):
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            dz = p1.z - p0.z
            length += math.sqrt(dx * dx + dy * dy + dz * dz)
        strategy_used = PERIMETER_STRATEGY
        try:
            from ..services import path_planner as _pp

            minkowski_loops_attr = getattr(
                _pp.generate_perimeter_path, "_last_minkowski_boundary_loops", None
            )
            if minkowski_loops_attr:
                minkowski_boundary_loops = [
                    [PathPoint(x=p.x, y=p.y, z=p.z) for p in loop]
                    for loop in minkowski_loops_attr
                    if loop
                ]
            else:
                minkowski_boundary_loops = None
        except Exception:
            minkowski_boundary_loops = None
    elif strategy == ORBIT_STRATEGY:
        # Generate a circular orbit path around the bounding box
        points = generate_orbit_path(
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            camera_radius=body.cameraRadius,
            num_samples=num_samples,
            factor=1.5,
        )
        # Approximate length using the radius of the orbit
        sx = bbox_max[0] - bbox_min[0]
        sy = bbox_max[1] - bbox_min[1]
        sz = bbox_max[2] - bbox_min[2]
        max_dim = max(sx, sy, sz)
        radius = max_dim * 1.5 + body.cameraRadius
        length = 2 * math.pi * radius
        strategy_used = ORBIT_STRATEGY
    else:
        # Unrecognised strategy; inform the caller
        raise HTTPException(status_code=400, detail=f"Unknown strategy '{strategy}'")
    path_id = uuid.uuid4().hex
    # Apply performance cap: downsample the path if it contains too many points.
    downsampled = False
    if len(points) > MAX_PATH_POINTS:
        step = math.ceil(len(points) / MAX_PATH_POINTS)
        original_points = points
        points = original_points[::step]
        # Ensure the path is closed by appending the starting point if necessary
        if points and (
            points[0].x != points[-1].x
            or points[0].y != points[-1].y
            or points[0].z != points[-1].z
        ):
            points.append(points[0])
        # Recompute length for the downsampled path
        new_length = 0.0
        for p0, p1 in zip(points, points[1:]):
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            dz = p1.z - p0.z
            new_length += math.sqrt(dx * dx + dy * dy + dz * dz)
        length = new_length
        downsampled = True
    # Build metadata including point count and whether the path was downsampled
    metadata = {
        "length": length,
        "strategy": strategy_used,
        "points": len(points),
        "downsampled": downsampled,
    }
    # Record additional perimeter metadata when using the perimeter strategy.  The
    # path planner attaches extra information (perimeterSource,
    # simplificationTolerance, splined, controlPointCount,
    # sampledPointCount) to its ``_last_meta`` attribute when
    # generate_perimeter_path is called.  This information is merged
    # into the response metadata for perimeter paths.
    if strategy_used == PERIMETER_STRATEGY:
        try:
            # Always record plane and height offset for perimeter paths
            metadata["plane"] = getattr(body, "plane", "xy")
            metadata["heightOffset"] = getattr(body, "heightOffset", 0.0)
            # Merge planner metadata if available
            from ..services import path_planner as _pp
            extra = getattr(_pp.generate_perimeter_path, "_last_meta", None)
            if isinstance(extra, dict):
                metadata.update(extra)
        except Exception:
            pass
    # Optionally log creation details for debugging
    try:
        # Include plane and height offset in log when available.  Fallback
        # gracefully if attributes are missing (e.g. orbit strategy).
        extra = ""
        if strategy_used == PERIMETER_STRATEGY:
            extra = f" plane={getattr(body, 'plane', 'xy')} heightOffset={getattr(body, 'heightOffset', 0.0)}"
        print(
            f"[create_path] model={model_id} strategy={strategy_used} radius={body.cameraRadius} "
            f"points={len(points)} bbox_min={bbox_min} bbox_max={bbox_max} downsampled={downsampled}{extra}"
        )
    except Exception:
        pass
    # Store path in registry for later retrieval (validate/export)
    # Store control points for SolidWorks export when available.  The
    # planner attaches a ``_last_control_points_3d`` attribute to
    # ``generate_perimeter_path`` if a slice‑based perimeter was computed.
    control_points = None
    try:
        from ..services import path_planner as _pp
        control_points = getattr(_pp.generate_perimeter_path, "_last_control_points_3d", None)
    except Exception:
        control_points = None
    path_registry[path_id] = {
        "modelId": model_id,
        "points": points,
        "metadata": metadata,
        "bbox_min": bbox_min,
        "bbox_max": bbox_max,
        "controlPoints": control_points,
        "minkowskiBoundaryLoops": minkowski_boundary_loops if strategy_used == PERIMETER_STRATEGY else None,
    }
    return PathResponse(
        pathId=path_id,
        modelId=model_id,
        points=points,
        metadata=metadata,
        minkowskiBoundaryLoops=minkowski_boundary_loops if strategy_used == PERIMETER_STRATEGY else None,
    )


@router.post(
    "/models/{model_id}/paths/{path_id}/validate",
    response_model=PathValidateResponse,
)
async def validate_path(
    model_id: str, path_id: str, body: PathValidateRequest
) -> PathValidateResponse:
    """Validate the given path against the model's bounding box.

    The edited path provided in the request body is validated by
    computing the clearance of each point from the model's bounding
    box.  Points closer than the specified camera radius are reported
    as violations.
    """
    # Ensure path exists and corresponds to the given model
    entry = path_registry.get(path_id)
    if not entry:
        raise HTTPException(status_code=404, detail="Path not found")
    if entry["modelId"] != model_id:
        raise HTTPException(status_code=400, detail="Model mismatch for path")
    # Use bounding box associated with the stored path; if missing,
    # recompute from shape or mesh.  We prefer the shape's bounding
    # box because it reflects the actual model geometry.  Fallback to
    # the mesh's bounding box (which may be a stub) if OCC is not
    # available or an error occurs.
    bbox_min = entry.get("bbox_min")
    bbox_max = entry.get("bbox_max")
    if not bbox_min or not bbox_max:
        # Try to compute from shape
        try:
            shape = load_shape_for_model(model_id)
            if shape is not None:
                min_xyz, max_xyz = compute_bounding_box(shape)
                bbox_min = tuple(min_xyz)
                bbox_max = tuple(max_xyz)
            else:
                raise Exception("Shape loading returned None")
        except Exception:
            # Fallback to mesh bbox
            mesh = get_mesh_for_model(model_id)
            bbox_min = tuple(mesh.bbox.min)
            bbox_max = tuple(mesh.bbox.max)
    # Validate path using the provided points (may differ from stored)
    valid, min_clearance, violations = validate_path_points(
        points=body.points,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        camera_radius=body.cameraRadius,
    )
    return PathValidateResponse(
        valid=valid,
        minClearance=min_clearance,
        violations=violations,
    )


@router.get("/models/{model_id}/paths/{path_id}/export")
async def export_path(model_id: str, path_id: str) -> Response:
    """Export the stored path as CSV.

    Args:
        model_id: Identifier of the model.
        path_id: Identifier of the path to export.

    Returns:
        A Response containing CSV data for the points.
    """
    entry = path_registry.get(path_id)
    if not entry:
        raise HTTPException(status_code=404, detail="Path not found")
    # Build CSV content in memory
    output = io.StringIO()
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(["x", "y", "z"])
    for p in entry["points"]:
        # p is a PathPoint instance
        writer.writerow([p.x, p.y, p.z])
    csv_data = output.getvalue()
    # Return the CSV as text/csv
    return Response(content=csv_data, media_type="text/csv")


# New endpoint: SolidWorks-compatible export
@router.get("/models/{model_id}/paths/{path_id}/export/solidworks")
async def export_path_solidworks(model_id: str, path_id: str, mode: str = "dense") -> Response:
    """Export the stored path as a CSV formatted for SolidWorks.

    The returned CSV includes an index column and uses capitalised column
    names (index,X,Y,Z).  Coordinates are output with fixed precision.

    A query parameter ``mode`` controls whether the dense sampled path
    points or only the simplified/spline control points are exported.
    Accepted values:

    * ``dense`` (default) – export all sampled path points as before.
    * ``controlPoints`` – export only the simplified control points from
      the slice‑based perimeter pipeline.  When unavailable the dense
      points are used as a fallback.

    Args:
        model_id: Identifier of the model.
        path_id: Identifier of the path to export.
        mode: Export mode ('dense' or 'controlPoints').  Case insensitive.

    Returns:
        A Response containing CSV data for the points in SolidWorks format.
    """
    entry = path_registry.get(path_id)
    if not entry:
        raise HTTPException(status_code=404, detail="Path not found")
    # Determine which point set to export based on mode
    selected_mode = (mode or "dense").strip().lower()
    points_to_export = entry["points"]
    if selected_mode == "controlpoints":
        ctrl = entry.get("controlPoints")
        if ctrl:
            points_to_export = ctrl
    # Build CSV content in memory with index and capitalised headers
    output = io.StringIO()
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(["index", "X", "Y", "Z"])
    for idx, p in enumerate(points_to_export):
        # p may be a PathPoint or a simple object with x,y,z attributes
        writer.writerow([
            idx,
            f"{p.x:.6f}",
            f"{p.y:.6f}",
            f"{p.z:.6f}",
        ])
    csv_data = output.getvalue()
    return Response(content=csv_data, media_type="text/csv")


# ---------------------------------------------------------------------------
# Outline endpoint (Phase 0.9.6.4)
#
# Returns the base perimeter outline of a model on a selected plane.  This
# endpoint is used by the frontend to preview the outline independently of
# path computation.  It supports both slice‑based (mesh) outlines and
# HLR‑based (projection) outlines.  When no outline can be extracted the
# bounding box rectangle is returned as a fallback.  The response
# conforms to the OutlineResponse schema defined in api/models.py.

@router.get(
    "/models/{model_id}/outline",
    response_model=OutlineResponse,
)
async def get_model_outline(
    model_id: str,
    plane: str = "xy",
    heightOffset: float = 0.0,
    useModelOutline: bool = False,
    outlineStitchTolerance: float | None = None,
    outlineDetail: str | None = None,
    outlineStitchPercent: float | None = None,
    perimeterIslandMode: str = "single",
) -> OutlineResponse:
    """Compute the base perimeter outline of a model on a given plane.

    The outline is the closed loop representing the model's cross‑section
    (slice) or projected silhouette (HLR) on the selected plane at the
    specified height offset.  When HLR projection is requested but fails,
    the function falls back to slice‑based extraction and finally to a
    simple bounding box rectangle.

    Query Parameters:
        plane: One of 'xy', 'xz' or 'yz'.  Defaults to 'xy'.
        heightOffset: Offset along the orthogonal axis (default 0).
        useModelOutline: If True, use the model's projected outline (HLR);
            otherwise use the mesh slice.
        outlineStitchTolerance: Optional override for HLR stitching tolerance.
        outlineDetail: Optional detail level for HLR sampling ('low', 'medium', 'high', 'auto').

    Returns:
        OutlineResponse containing the outline points and metadata.
    """
    # Normalise plane
    plane = (plane or "xy").strip().lower()
    # Attempt to build a slice plane descriptor for consistent projection
    try:
        slice_plane = make_slice_plane(plane, heightOffset)
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"Invalid plane '{plane}': {exc}")
    # Prepare metadata container
    metadata: dict[str, object] = {
        "plane": plane,
        "heightOffset": heightOffset,
    }
    points_3d: list[tuple[float, float, float]] | None = None
    source = None
    # Try mesh‑based outline when requested.  The new silhouette logic collapses
    # the cached mesh onto the requested plane and extracts boundary loops
    # without relying on HLR/BRep.  We ignore outlineStitchTolerance and
    # outlineDetail here, using outlineStitchPercent as the primary tuning
    # parameter.  Height offset is applied after collapsing.
    if useModelOutline:
        try:
            loops = compute_mesh_silhouette_loops(
                model_id=model_id,
                plane=plane,
                outline_stitch_percent=outlineStitchPercent,
                height_offset=heightOffset,
            )
            if loops:
                # Choose the appropriate loop for preview.  When multi island
                # mode is disabled pick the largest loop by projected area; otherwise
                # simply take the first loop.  Use the slicing helpers to compute
                # area in the 2D plane.
                if perimeterIslandMode != "multi" and len(loops) > 1:
                    from ..services.slicing import _polygon_area_2d
                    slice_plane = make_slice_plane(plane, heightOffset)
                    best_idx = None
                    best_area = 0.0
                    for i, pts3d in enumerate(loops):
                        try:
                            uv_pts = [project_point_to_plane_uv(p, slice_plane) for p in pts3d]
                            a = _polygon_area_2d(uv_pts)
                            if abs(a) > abs(best_area):
                                best_area = a
                                best_idx = i
                        except Exception:
                            continue
                    if best_idx is not None:
                        points_3d = loops[best_idx]
                    else:
                        points_3d = loops[0]
                else:
                    # Single or multi mode – take the first loop
                    points_3d = loops[0]
                source = "outline"
        except Exception:
            # Ignore errors here; fallback to slice
            points_3d = None
            source = None
    # Slice‑based outline as fallback or when not using model outline
    if points_3d is None:
        try:
            loop = compute_slice_perimeter_loop(model_id, slice_plane)
            if loop is not None and loop.points_2d:
                # Convert UV points to 3D coordinates
                points_3d_temp = lift_uv_to_3d(loop.points_2d, slice_plane)
                if points_3d_temp:
                    points_3d = points_3d_temp
                    source = "slice"
        except Exception:
            points_3d = None
            source = None
    # Bounding box rectangle fallback
    if points_3d is None or not points_3d:
        # Attempt to compute bounding box from shape; fallback to mesh
        try:
            shape = load_shape_for_model(model_id)
            if shape is not None:
                min_xyz, max_xyz = compute_bounding_box(shape)
                bbox_min = tuple(min_xyz)
                bbox_max = tuple(max_xyz)
            else:
                raise Exception("Shape loading returned None")
        except Exception:
            mesh = get_mesh_for_model(model_id)
            bbox_min = tuple(mesh.bbox.min)
            bbox_max = tuple(mesh.bbox.max)
        # Build rectangle in the requested plane at the specified offset
        min_x, min_y, min_z = bbox_min
        max_x, max_y, max_z = bbox_max
        if plane == "xy":
            zc = heightOffset
            points_3d = [
                (min_x, min_y, zc),
                (max_x, min_y, zc),
                (max_x, max_y, zc),
                (min_x, max_y, zc),
                (min_x, min_y, zc),
            ]
        elif plane == "xz":
            yc = heightOffset
            points_3d = [
                (min_x, yc, min_z),
                (max_x, yc, min_z),
                (max_x, yc, max_z),
                (min_x, yc, max_z),
                (min_x, yc, min_z),
            ]
        else:  # yz
            xc = heightOffset
            points_3d = [
                (xc, min_y, min_z),
                (xc, max_y, min_z),
                (xc, max_y, max_z),
                (xc, min_y, max_z),
                (xc, min_y, min_z),
            ]
        source = "bboxFallback"
    # Ensure closed loop (repeat first point if necessary)
    if points_3d:
        first = points_3d[0]
        last = points_3d[-1]
        if first != last:
            points_3d = list(points_3d) + [first]
    # Build metadata
    metadata["perimeterSource"] = source or "unknown"
    if useModelOutline:
        metadata["useModelOutline"] = True
    if outlineDetail is not None:
        metadata["outlineDetail"] = outlineDetail
    if outlineStitchTolerance is not None:
        metadata["outlineStitchTolerance"] = outlineStitchTolerance
    if outlineStitchPercent is not None:
        metadata["outlineStitchPercent"] = outlineStitchPercent
    # Convert to PathPoint list
    outline_points = [PathPoint(x=p[0], y=p[1], z=p[2]) for p in points_3d]
    return OutlineResponse(modelId=model_id, points=outline_points, metadata=metadata)