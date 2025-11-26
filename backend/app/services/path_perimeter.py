"""
Perimeter-based path generation for Fly.

This module contains the heavier slice/outline-based perimeter logic,
including:
  * Minkowski-offset / rolling-circle perimeter paths
  * Multi-island outline handling
  * B-spline fitting and sampling
  * Bounding-box fallback perimeter generation

The API is intentionally kept identical to the original
:mod:`path_planner` functions so that callers can import
:func:`generate_perimeter_path` and
:func:`generate_rolling_circle_perimeter_path` without change.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Iterable, List, Tuple

from ..api.models import PathPoint

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
    compute_raster_perimeter_polygons,
    get_mesh_for_model,
)

# -------------------------------------------------------------------------
# Geometry helpers for smoothing and self-intersection detection
# -------------------------------------------------------------------------


def _smooth_closed_polyline_laplacian(
    points: List[Tuple[float, float]],
    iterations: int = 1,
    alpha: float = 0.5,
) -> List[Tuple[float, float]]:
    """
    Simple Laplacian smoothing for a closed 2D polyline.

    Each iteration moves a vertex towards the average of its two
    neighbours by a factor ``alpha``.  This preserves the overall
    shape and offset radius, but irons out tiny kinks and zig-zags.
    """
    n = len(points)
    if n < 3 or iterations <= 0:
        return points

    smoothed = points
    for _ in range(iterations):
        new_points: List[Tuple[float, float]] = []
        for i in range(n):
            prev_pt = smoothed[(i - 1) % n]
            curr_pt = smoothed[i]
            next_pt = smoothed[(i + 1) % n]

            avg_x = 0.5 * (prev_pt[0] + next_pt[0])
            avg_y = 0.5 * (prev_pt[1] + next_pt[1])

            new_x = curr_pt[0] + alpha * (avg_x - curr_pt[0])
            new_y = curr_pt[1] + alpha * (avg_y - curr_pt[1])

            new_points.append((new_x, new_y))
        smoothed = new_points

    return smoothed


def _choose_smoothing_iterations(detail: str) -> int:
    """
    Map UI 'detail' → how aggressively we smooth direction changes.

    - low          → no extra smoothing
    - auto/medium  → 1 iteration
    - high         → 2 iterations (most smoothing)
    """
    d = (detail or "").strip().lower()
    if d == "low":
        return 0
    if d in {"auto", "medium"}:
        return 1
    if d == "high":
        return 2
    return 1


def _orientation(p: tuple[float, float], q: tuple[float, float], r: tuple[float, float]) -> int:
    """Return the orientation of the ordered triplet (p, q, r).

    The function returns:
        * 0 if the points are colinear
        * 1 if they are oriented clockwise
        * 2 if they are oriented counter-clockwise

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
    """Return True if the closed polygon defined by *poly* self-intersects.

    The input should be an ordered list of vertices (x, y).  The first and
    last vertices may or may not be the same; this function will treat the
    polygon as closed either way.  Adjacent edges and the first/last edge
    pair are ignored when testing for intersections.
    """
    n = len(poly)
    if n < 4:
        # A triangle cannot self-intersect
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


# -------------------------------------------------------------------------
# Rolling-circle perimeter path helper
# -------------------------------------------------------------------------


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
    """Generate one or more closed paths for a rolling-circle perimeter.

    The locus of the circle centre is equivalent to offsetting the outer
    perimeter by ``circle_radius + base_clearance``.  This helper
    leverages the raster perimeter extraction pipeline to obtain the
    base loops, projects them into the chosen 2D plane, offsets them
    outward by the effective clearance and fits a closed B-spline
    through the simplified offset polygon.

    Returns a list of lists of :class:`PathPoint` objects, one sublist
    per island.
    """
    logger = logging.getLogger(__name__)
    generate_rolling_circle_perimeter_path._last_minkowski_boundary_loops = []  # type: ignore[attr-defined]
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
            clearance_radius=max(0.0, circle_radius + base_clearance),
        )
        # Unpack loops; older versions may return just loops
        if isinstance(result, tuple) and len(result) >= 1:
            loops_3d = result[0]  # type: ignore[assignment]
        else:
            loops_3d = result  # type: ignore[assignment]
    except Exception as exc:  # pragma: no cover - defensive logging
        logger.error(f"{prefix} Error obtaining raster perimeter polygons: {exc}")
        return []

    t_perim = time.perf_counter()
    logger.info(
        f"{prefix} raster perimeter polygons: {len(loops_3d)} loops in {t_perim - t_start:.4f}s"
    )

    # Project loops into UV coordinates according to the chosen plane,
    # and compute signed area to determine orientation.
    loops_uv: list[list[tuple[float, float]]] = []
    loops_area: list[float] = []
    for loop in loops_3d:
        if len(loop) < 3:
            continue
        try:
            uv = [project_point_to_plane_uv(p, slice_plane) for p in loop]
        except Exception:
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

    # Normalise detail for simplification, sampling and smoothing
    detail_norm = (perimeter_detail or "auto").strip().lower()
    if detail_norm not in {"low", "medium", "high", "auto"}:
        detail_norm = "auto"

    all_paths: list[list[PathPoint]] = []
    minkowski_boundary_loops: list[list[PathPoint]] = []

    # Process each loop independently
    for uv_loop, area_val in zip(loops_uv, loops_area):
        # Offset the polygon outward and guard against self-intersections
        t_off0 = time.perf_counter()
        effective_offset = offset_distance
        try:
            offset_uv = offset_polygon_2d(uv_loop, effective_offset, area_hint=area_val)
        except Exception as exc:
            logger.error(f"{prefix} error offsetting polygon: {exc}")
            continue

        attempts = 0
        while _polygon_self_intersects(offset_uv) and attempts < 5:
            attempts += 1
            effective_offset *= 0.5
            if effective_offset <= 1e-6:
                logger.warning(
                    f"{prefix} offset polygon self-intersects; giving up after shrinking to {effective_offset:.6f}"
                )
                break
            try:
                offset_uv = offset_polygon_2d(uv_loop, effective_offset, area_hint=area_val)
            except Exception as exc:
                logger.error(
                    f"{prefix} error re-offsetting polygon on attempt {attempts}: {exc}"
                )
                break

        t_off1 = time.perf_counter()
        logger.info(
            f"{prefix} offsetting: {len(uv_loop)}→{len(offset_uv)} pts in "
            f"{t_off1 - t_off0:.4f}s (distance={effective_offset:.4f}, attempts={attempts})"
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
            f"{prefix} simplify: {len(offset_uv)}→{len(simple_uv)} pts in "
            f"{t_simp1 - t_simp0:.4f}s (tol={tol:.6f})"
        )

        # Determine sample count based on detail and original resolution
        if detail_norm == "low":
            num_samples = max(4 * len(simple_uv), samples_per_side * 2)
        elif detail_norm == "high":
            num_samples = max(4 * len(simple_uv), samples_per_side * 8)
        else:
            num_samples = max(4 * len(simple_uv), samples_per_side * 4)

        # Fit a closed B-spline and sample uniformly
        t_spline0 = time.perf_counter()
        try:
            spline = fit_uniform_closed_bspline(simple_uv)
            sampled_uv = sample_bspline(spline, num_samples)
        except Exception as exc:
            logger.error(f"{prefix} error fitting/sampling spline: {exc}")
            continue
        t_spline1 = time.perf_counter()
        logger.info(
            f"{prefix} spline fit+sample: {num_samples} samples in "
            f"{t_spline1 - t_spline0:.4f}s"
        )

        # Extra pass: Laplacian smoothing of the sampled loop to remove
        # disjoint direction flips at inflection points without changing
        # the overall offset radius too much.
        smooth_iterations = _choose_smoothing_iterations(detail_norm)
        if smooth_iterations > 0:
            t_smooth0 = time.perf_counter()
            sampled_uv = _smooth_closed_polyline_laplacian(
                sampled_uv,
                iterations=smooth_iterations,
                alpha=0.5,
            )
            t_smooth1 = time.perf_counter()
            logger.info(
                "%s Laplacian smoothing: iterations=%d time=%.4fs",
                prefix,
                smooth_iterations,
                t_smooth1 - t_smooth0,
            )

        # Lift sampled UV coordinates back into 3D
        t_lift0 = time.perf_counter()
        try:
            # Use the smoothed, sampled loop as the Minkowski boundary we expose to the UI.
            minkowski_uv = sampled_uv
            boundary_pts3d = lift_uv_to_3d(minkowski_uv, slice_plane)
            minkowski_boundary_loops.append(
                [PathPoint(x=p[0], y=p[1], z=p[2]) for p in boundary_pts3d]
            )
        except Exception as exc:
            logger.error(f"{prefix} error lifting Minkowski UV to 3D: {exc}")

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

    generate_rolling_circle_perimeter_path._last_minkowski_boundary_loops = (
        minkowski_boundary_loops
    )  # type: ignore[attr-defined]

    return all_paths


# -------------------------------------------------------------------------
# General perimeter path (outline/slice + Minkowski + fallback)
# -------------------------------------------------------------------------


def generate_perimeter_path(
    bbox_min,
    bbox_max,
    offset,
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
    perimeter_use_rolling_circle: bool = False,
    perimeter_circle_diameter: float | None = None,
):
    """Generate a perimeter-following camera path.

    This implementation performs the following steps:

    * Optionally dispatches to the rolling-circle helper when requested.
    * Extracts rasterised perimeter loops (or a bbox fallback), respecting
      ``island_mode`` and outline stitching options.
    * Offsets, simplifies, fits a closed B-spline and samples the loop.
    * Applies a light Laplacian smoothing pass to iron out heading flips
      without collapsing clearance.
    * Lifts the sampled UV points back into 3D ``PathPoint`` objects.
    """

    logger = logging.getLogger(__name__)
    prefix = "[PerimeterPath]"

    # Rolling-circle mode delegates to the specialised helper but still
    # honours island_mode and detail for smoothing.
    if perimeter_use_rolling_circle:
        circle_radius = (perimeter_circle_diameter or 0.0) * 0.5
        return generate_rolling_circle_perimeter_path(
            model_id=model_id or "",
            plane=plane,
            height_offset=height_offset,
            circle_radius=max(0.0, circle_radius),
            island_mode=island_mode,
            perimeter_detail=detail,
            base_clearance=max(0.0, offset),
            samples_per_side=samples_per_side,
        )

    t_start = time.perf_counter()
    detail_norm = (detail or "auto").strip().lower()
    if detail_norm not in {"low", "medium", "high", "auto"}:
        detail_norm = "auto"

    # Build the slice plane for projection and lifting
    slice_plane = make_slice_plane(plane, height_offset)

    # Extract perimeter loops using the raster pipeline when possible.
    loops_3d: list[list[tuple[float, float, float]]] = []
    loop_areas: list[float] = []
    meta: dict | None = None
    if model_id and use_model_outline:
        try:
            loops_3d, loop_areas, meta = compute_raster_perimeter_polygons(
                model_id=model_id,
                plane=plane,
                detail=outline_detail or detail_norm,
                island_mode=island_mode,
                outline_stitch_percent=outline_stitch_percent,
            )
        except Exception as exc:  # pragma: no cover - defensive logging
            logger.error("%s raster perimeter extraction failed: %s", prefix, exc)

    # Fallback: use a simple bounding-box rectangle when no outline is available
    if not loops_3d:
        logger.info("%s falling back to bbox perimeter", prefix)
        (xmin, ymin, zmin) = bbox_min
        (xmax, ymax, zmax) = bbox_max
        z_center = 0.5 * (zmin + zmax)
        y_center = 0.5 * (ymin + ymax)
        x_center = 0.5 * (xmin + xmax)
        if plane.lower() == "xz":
            plane_coord = y_center + height_offset
            slice_plane = make_slice_plane(plane, plane_coord)
            base_loop = [
                (xmin, zmin, plane_coord),
                (xmax, zmin, plane_coord),
                (xmax, zmax, plane_coord),
                (xmin, zmax, plane_coord),
            ]
        elif plane.lower() == "yz":
            plane_coord = x_center + height_offset
            slice_plane = make_slice_plane(plane, plane_coord)
            base_loop = [
                (plane_coord, ymin, zmin),
                (plane_coord, ymax, zmin),
                (plane_coord, ymax, zmax),
                (plane_coord, ymin, zmax),
            ]
        else:  # default xy
            plane_coord = z_center + height_offset
            slice_plane = make_slice_plane(plane, plane_coord)
            base_loop = [
                (xmin, ymin, plane_coord),
                (xmax, ymin, plane_coord),
                (xmax, ymax, plane_coord),
                (xmin, ymax, plane_coord),
            ]
        loops_3d = [base_loop]
        loop_areas = [abs(_polygon_area_2d([project_point_to_plane_uv(p, slice_plane) for p in base_loop]))]
        meta = meta or {}
        meta["isBBoxFallback"] = True

    # Respect island selection
    if island_mode != "multi" and loop_areas:
        idx = max(range(len(loop_areas)), key=lambda i: abs(loop_areas[i]))
        loops_3d = [loops_3d[idx]]
        loop_areas = [loop_areas[idx]]

    # For the synthetic bounding-box fallback, generate a simple expanded
    # rectangle directly to guarantee clearance without spline distortion.
    if meta and meta.get("isBBoxFallback"):
        plane_norm = slice_plane.plane_type
        if plane_norm == "xz":
            plane_coord = slice_plane.offset
            corner_centres = [
                ((xmax, zmin), -math.pi / 2, 0.0),
                ((xmax, zmax), 0.0, math.pi / 2),
                ((xmin, zmax), math.pi / 2, math.pi),
                ((xmin, zmin), math.pi, 3 * math.pi / 2),
            ]
            to_point = lambda x, y: (x, plane_coord, y)
        elif plane_norm == "yz":
            plane_coord = slice_plane.offset
            corner_centres = [
                ((ymax, zmin), -math.pi / 2, 0.0),
                ((ymax, zmax), 0.0, math.pi / 2),
                ((ymin, zmax), math.pi / 2, math.pi),
                ((ymin, zmin), math.pi, 3 * math.pi / 2),
            ]
            to_point = lambda y, z: (plane_coord, y, z)
        else:  # xy
            plane_coord = slice_plane.offset
            corner_centres = [
                ((xmax, ymin), -math.pi / 2, 0.0),
                ((xmax, ymax), 0.0, math.pi / 2),
                ((xmin, ymax), math.pi / 2, math.pi),
                ((xmin, ymin), math.pi, 3 * math.pi / 2),
            ]
            to_point = lambda x, y: (x, y, plane_coord)

        arc_samples = max(4, samples_per_side // 2)
        sampled_pts: list[tuple[float, float, float]] = []
        for (centre, start_ang, end_ang) in corner_centres:
            cx, cy = centre
            for i in range(arc_samples + 1):
                t = i / arc_samples
                ang = start_ang + t * (end_ang - start_ang)
                x = cx + offset * math.cos(ang)
                y = cy + offset * math.sin(ang)
                sampled_pts.append(to_point(x, y))
        sampled_pts.append(sampled_pts[0])

        path_points = [PathPoint(x=p[0], y=p[1], z=p[2]) for p in sampled_pts]
        generate_perimeter_path._last_meta = meta  # type: ignore[attr-defined]
        generate_perimeter_path._last_minkowski_boundary_loops = []  # type: ignore[attr-defined]
        return path_points if island_mode != "multi" else [path_points]

    all_paths: list[list[PathPoint]] = []
    minkowski_boundary_loops: list[list[PathPoint]] = []

    for loop_3d, area_hint in zip(loops_3d, loop_areas or [0.0]):
        if len(loop_3d) < 3:
            continue
        try:
            uv_loop = [project_point_to_plane_uv(p, slice_plane) for p in loop_3d]
        except Exception:
            continue

        # For the simple bounding-box fallback, construct an expanded rectangle
        # directly rather than relying on the offsetter.  This guarantees the
        # requested clearance in synthetic scenarios used by the tests.
        if meta and meta.get("isBBoxFallback"):
            plane_norm = slice_plane.plane_type
            if plane_norm == "xz":
                uv_loop = [
                    (xmin - offset, zmin - offset),
                    (xmax + offset, zmin - offset),
                    (xmax + offset, zmax + offset),
                    (xmin - offset, zmax + offset),
                ]
            elif plane_norm == "yz":
                uv_loop = [
                    (ymin - offset, zmin - offset),
                    (ymax + offset, zmin - offset),
                    (ymax + offset, zmax + offset),
                    (ymin - offset, zmax + offset),
                ]
            else:  # xy
                uv_loop = [
                    (xmin - offset, ymin - offset),
                    (xmax + offset, ymin - offset),
                    (xmax + offset, ymax + offset),
                    (xmin - offset, ymax + offset),
                ]
            signed_area = _polygon_area_2d(uv_loop)
            offset_uv = uv_loop
        else:
            try:
                signed_area = _polygon_area_2d(uv_loop)
            except Exception:
                continue
            if signed_area < 0:
                uv_loop = list(reversed(uv_loop))
                signed_area = -signed_area

            effective_offset = max(0.0, offset)
            try:
                offset_uv = offset_polygon_2d(uv_loop, effective_offset, area_hint=signed_area)
            except Exception as exc:
                logger.error("%s error offsetting polygon: %s", prefix, exc)
                continue

        attempts = 0
        while _polygon_self_intersects(offset_uv) and attempts < 5:
            attempts += 1
            effective_offset *= 0.5
            if effective_offset <= 1e-6:
                logger.warning("%s offset polygon self-intersects; aborting shrink", prefix)
                break
            try:
                offset_uv = offset_polygon_2d(uv_loop, effective_offset, area_hint=signed_area)
            except Exception:
                break

        base_tol = choose_simplification_tolerance(offset_uv)
        if detail_norm == "low":
            tol = base_tol * 2.0
        elif detail_norm == "high":
            tol = base_tol * 0.5
        else:
            tol = base_tol

        try:
            simple_uv = simplify_polyline_rdp(offset_uv, tol)
        except Exception as exc:
            logger.error("%s simplification failed: %s", prefix, exc)
            continue

        if detail_norm == "low":
            num_samples = max(4 * len(simple_uv), samples_per_side * 2)
        elif detail_norm == "high":
            num_samples = max(4 * len(simple_uv), samples_per_side * 8)
        else:
            num_samples = max(4 * len(simple_uv), samples_per_side * 4)

        try:
            spline = fit_uniform_closed_bspline(simple_uv)
            sampled_uv = sample_bspline(spline, num_samples)
        except Exception as exc:
            logger.error("%s spline fit/sample failed: %s", prefix, exc)
            continue

        smooth_iterations = _choose_smoothing_iterations(detail_norm)
        if smooth_iterations > 0:
            sampled_uv = _smooth_closed_polyline_laplacian(
                sampled_uv,
                iterations=smooth_iterations,
                alpha=0.5,
            )

        try:
            minkowski_boundary_loops.append(
                [PathPoint(*p) for p in lift_uv_to_3d(sampled_uv, slice_plane)]
            )
        except Exception:
            pass

        try:
            pts3d = lift_uv_to_3d(sampled_uv, slice_plane)
        except Exception as exc:
            logger.error("%s lifting UV to 3D failed: %s", prefix, exc)
            continue

        if pts3d and pts3d[0] != pts3d[-1]:
            pts3d.append(pts3d[0])

        all_paths.append([PathPoint(x=p[0], y=p[1], z=p[2]) for p in pts3d])

    t_end = time.perf_counter()
    logger.info(
        "%s detail=%s paths=%d elapsed=%.4fs",
        prefix,
        detail_norm,
        len(all_paths),
        t_end - t_start,
    )

    # Expose diagnostics for the API layer
    generate_perimeter_path._last_meta = meta  # type: ignore[attr-defined]
    generate_perimeter_path._last_minkowski_boundary_loops = minkowski_boundary_loops  # type: ignore[attr-defined]

    if island_mode != "multi":
        return all_paths[0] if all_paths else []
    return all_paths
