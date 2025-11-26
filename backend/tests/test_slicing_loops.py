"""
Tests for loop construction and perimeter selection in slicing.py.

These tests validate that the helper functions for snapping endpoints,
building loops from segments and selecting the outermost perimeter behave
correctly on simple synthetic geometries.  They do not depend on
OpenCascade and run solely on the pure‑Python slicing utilities.
"""

from __future__ import annotations

import sys
from pathlib import Path
import math
import pytest

# Make the backend package importable when running tests directly via pytest
sys.path.append(str(Path(__file__).resolve().parents[1]))

from app.services.slicing import (
    SliceSegment,
    make_slice_plane,
    build_loops_from_segments,
    find_outer_perimeter_loop,
)


def create_rectangle_segments(width: float, height: float) -> list[SliceSegment]:
    """Create segments representing a rectangle in the XY plane at z=0.

    The rectangle corners are (0,0,0), (width,0,0), (width,height,0) and (0,height,0).
    """
    p0 = (0.0, 0.0, 0.0)
    p1 = (width, 0.0, 0.0)
    p2 = (width, height, 0.0)
    p3 = (0.0, height, 0.0)
    return [
        SliceSegment(p1=p0, p2=p1),
        SliceSegment(p1=p1, p2=p2),
        SliceSegment(p1=p2, p2=p3),
        SliceSegment(p1=p3, p2=p0),
    ]


def test_rectangle_loops_and_area() -> None:
    """A single rectangle should produce one loop with area equal to width*height."""
    segments = create_rectangle_segments(4.0, 3.0)
    plane = make_slice_plane("xy", 0.0)
    loops = build_loops_from_segments(segments, plane, snap_eps=1e-4)
    assert len(loops) == 1
    loop = loops[0]
    # Expect 4 unique points
    assert len(loop.points_3d) == 4
    # Area should be ±12 (4*3)
    assert math.isclose(abs(loop.area), 12.0, rel_tol=1e-6)
    # Points lie in the XY plane (z≈0)
    for p in loop.points_3d:
        assert math.isclose(p[2], 0.0, abs_tol=1e-6)


def test_outer_loop_selection_with_hole() -> None:
    """When an inner rectangle is present, the outer loop should be selected."""
    # Outer 4x4 square and inner 1x1 square
    outer_segments = create_rectangle_segments(4.0, 4.0)
    # Translate inner rectangle to (1,1) – (2,2)
    inner_raw = create_rectangle_segments(1.0, 1.0)
    inner_segments: list[SliceSegment] = []
    for seg in inner_raw:
        p1 = (seg.p1[0] + 1.0, seg.p1[1] + 1.0, seg.p1[2])
        p2 = (seg.p2[0] + 1.0, seg.p2[1] + 1.0, seg.p2[2])
        inner_segments.append(SliceSegment(p1=p1, p2=p2))
    all_segments = outer_segments + inner_segments
    plane = make_slice_plane("xy", 0.0)
    loops = build_loops_from_segments(all_segments, plane, snap_eps=1e-4)
    # Expect two loops: outer and inner
    assert len(loops) == 2
    outer = find_outer_perimeter_loop(loops)
    assert outer is not None
    # The absolute area of the outer loop should be greater than that of the inner loop
    areas = sorted([abs(lp.area) for lp in loops], reverse=True)
    assert math.isclose(areas[0], 16.0, rel_tol=1e-6)
    assert math.isclose(areas[1], 1.0, rel_tol=1e-6)
    # The chosen outer loop should correspond to the 4x4 rectangle
    xs = [p[0] for p in outer.points_3d]
    ys = [p[1] for p in outer.points_3d]
    assert math.isclose(min(xs), 0.0, abs_tol=1e-6)
    assert math.isclose(max(xs), 4.0, abs_tol=1e-6)
    assert math.isclose(min(ys), 0.0, abs_tol=1e-6)
    assert math.isclose(max(ys), 4.0, abs_tol=1e-6)