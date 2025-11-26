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

from .path_perimeter import generate_perimeter_path as _generate_perimeter_path_impl
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
    """
    Wrapper around services.path_perimeter.generate_perimeter_path.

    This keeps the original path_planner API stable while delegating the
    actual implementation to path_perimeter.py, which now supports:

      - island_mode (single / multi)
      - outline_stitch_percent
      - outline_detail
      - rolling-circle perimeter options

    All existing callers that import generate_perimeter_path from
    path_planner continue to work, including routes_paths.create_path.
    """

    return _generate_perimeter_path_impl(
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        offset=offset,
        samples_per_side=samples_per_side,
        bulge_factor=bulge_factor,
        smoothness=smoothness,
        plane=plane,
        height_offset=height_offset,
        model_id=model_id,
        detail=detail,
        use_spline_override=use_spline_override,
        use_model_outline=use_model_outline,
        outline_stitch_percent=outline_stitch_percent,
        island_mode=island_mode,
        outline_stitch_tolerance=outline_stitch_tolerance,
        outline_detail=outline_detail,
        perimeter_use_rolling_circle=perimeter_use_rolling_circle,
        perimeter_circle_diameter=perimeter_circle_diameter,
    )
