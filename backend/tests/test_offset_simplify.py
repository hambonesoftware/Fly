"""
Tests for Phase 4 offset, simplification and lifting helpers in slicing.py.

These tests validate that the 2D polygon offset function expands or
contracts simple polygons correctly, that the Ramer–Douglas–Peucker
simplifier reduces vertex count without breaking closure, and that the
UV→XYZ lifting helper produces coordinates on the appropriate slice
plane.  These helpers operate on pure Python data structures and do
not depend on any external libraries.
"""

from __future__ import annotations

import sys
from pathlib import Path
import math

import pytest

# Add backend to sys.path for importing modules when running tests directly
sys.path.append(str(Path(__file__).resolve().parents[1]))

from app.services.slicing import (
    make_slice_plane,
    offset_polygon_2d,
    simplify_polyline_rdp,
    choose_simplification_tolerance,
    lift_uv_to_3d,
)


def test_offset_polygon_rectangle() -> None:
    """Offsetting a rectangle by a positive amount should expand it uniformly."""
    # Define a simple 10×5 rectangle in the UV plane
    poly = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)]
    # Offset by 2.0 outward; area is positive so outward normal is expected
    offset_poly = offset_polygon_2d(poly, 2.0)
    # Expect four vertices with expanded extents
    assert len(offset_poly) == 4
    us = [p[0] for p in offset_poly]
    vs = [p[1] for p in offset_poly]
    # New min/max should be expanded by 2.0 on all sides
    assert math.isclose(min(us), -2.0, abs_tol=1e-6)
    assert math.isclose(max(us), 12.0, abs_tol=1e-6)
    assert math.isclose(min(vs), -2.0, abs_tol=1e-6)
    assert math.isclose(max(vs), 7.0, abs_tol=1e-6)


def test_rdp_simplification_closure() -> None:
    """RDP simplification should preserve closure and reduce points on a near‑linear polyline."""
    # Create a polyline approximating a straight line with slight deviations
    points = [
        (0.0, 0.0),
        (1.0, 0.05),
        (2.0, -0.04),
        (3.0, 0.02),
        (4.0, 0.0),
    ]
    # Close the polyline explicitly for the algorithm
    closed = points + [points[0]]
    tol = 0.1
    simplified = simplify_polyline_rdp(closed, tol)
    # Should retain only the start and end (which are the same) → two points
    # plus closure yields length 2
    assert len(simplified) == 2
    # The first and last points should match (closed loop)
    assert simplified[0] == simplified[-1]


def test_choose_simplification_tolerance_scale() -> None:
    """The default simplification tolerance scales with the largest dimension."""
    poly = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)]
    tol = choose_simplification_tolerance(poly)
    # Expect 1% of the maximum extent (10 → 0.1)
    assert math.isclose(tol, 0.1, rel_tol=1e-6)


def test_lift_uv_to_3d_mapping() -> None:
    """Lifting UV points should place them on the slice plane with constant coordinate."""
    uv_points = [(1.0, 2.0), (3.0, 4.0)]
    # XY plane at z=0.5 → (u, v, 0.5)
    plane_xy = make_slice_plane("xy", 0.5)
    pts3d_xy = lift_uv_to_3d(uv_points, plane_xy)
    assert pts3d_xy == [(1.0, 2.0, 0.5), (3.0, 4.0, 0.5)]
    # XZ plane at y=-1.0 → (u, -1.0, v)
    plane_xz = make_slice_plane("xz", -1.0)
    pts3d_xz = lift_uv_to_3d(uv_points, plane_xz)
    assert pts3d_xz == [(1.0, -1.0, 2.0), (3.0, -1.0, 4.0)]
    # YZ plane at x=2.0 → (2.0, u, v)
    plane_yz = make_slice_plane("yz", 2.0)
    pts3d_yz = lift_uv_to_3d(uv_points, plane_yz)
    assert pts3d_yz == [(2.0, 1.0, 2.0), (2.0, 3.0, 4.0)]