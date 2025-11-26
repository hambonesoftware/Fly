"""Tests for union-first Minkowski boundary extraction on raster grids."""

import math

import numpy as np

from backend.app.services.raster_utils import _compute_distance_field_from_occupancy, _marching_squares_distance_field


def _loop_area(loop: list[tuple[float, float]]) -> float:
    area = 0.0
    for i in range(len(loop)):
        x0, y0 = loop[i]
        x1, y1 = loop[(i + 1) % len(loop)]
        area += x0 * y1 - x1 * y0
    return 0.5 * area


def _extract_loops_for_rectangles(gap: int, clearance: float) -> list[list[tuple[float, float]]]:
    """Construct two rectangles separated by ``gap`` and extract Minkowski loops."""

    cell_size = 1.0
    min_u = 0.0
    min_v = 0.0
    grid = np.zeros((40, 60), dtype=np.uint8)

    # First rectangle
    grid[10:28, 6:16] = 1
    # Second rectangle separated by ``gap`` columns
    start_col = 16 + gap
    grid[10:28, start_col:start_col + 10] = 1

    distance_field = _compute_distance_field_from_occupancy(grid, cell_size)
    loops = _marching_squares_distance_field(
        distance_field,
        threshold=clearance,
        cell_size=cell_size,
        min_u=min_u,
        min_v=min_v,
    )
    # Sort loops by descending absolute area for stable assertions
    loops.sort(key=lambda lp: abs(_loop_area(lp)), reverse=True)
    return loops


def test_union_first_merges_when_gap_is_smaller_than_diameter():
    """Gaps narrower than twice the radius should disappear in the Minkowski sum."""

    clearance = 3.0  # diameter = 6
    loops = _extract_loops_for_rectangles(gap=2, clearance=clearance)
    # Expect a single merged island
    assert len(loops) == 1
    # The merged loop should span the full extent of both rectangles plus clearance
    xs = [p[0] for p in loops[0]]
    assert math.isclose(min(xs), 6.0 - clearance, abs_tol=0.75)
    assert math.isclose(max(xs), 30.0, abs_tol=1.0)


def test_union_first_retains_separation_for_wide_gaps():
    """Gaps wider than the diameter should produce distinct islands."""

    clearance = 3.0  # diameter = 6
    loops = _extract_loops_for_rectangles(gap=8, clearance=clearance)
    assert len(loops) == 2
    # Ensure the islands stay disjoint by checking span ordering
    loops.sort(key=lambda lp: min(p[0] for p in lp))
    first_max_x = max(p[0] for p in loops[0])
    second_min_x = min(p[0] for p in loops[1])
    assert second_min_x - first_max_x > 0.0
