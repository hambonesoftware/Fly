"""
Tests for mesh–plane intersection routines defined in slicing.py.

These tests construct a simple unit cube mesh in pure Python and
exercise the intersection functions with slicing planes aligned with
the principal axes.  The expected number of intersection segments and
the planarity of the resulting points are asserted.  The tests do
not depend on OpenCascade being installed and therefore run in any
environment.
"""

from __future__ import annotations

import sys
from pathlib import Path
import math
import pytest

# Make the backend package importable when running tests directly via pytest
sys.path.append(str(Path(__file__).resolve().parents[1]))

from app.services.slicing import make_slice_plane, intersect_mesh_with_plane


def create_unit_cube_mesh() -> tuple[list[float], list[int]]:
    """Construct vertices and indices for a unit cube with corners at (0,0,0) and (1,1,1).

    The cube is composed of 12 triangles (two per face).  Vertices are
    returned as a flat list of floats and indices as a flat list of ints.
    """
    # Define vertices
    verts = [
        0.0, 0.0, 0.0,  # 0
        1.0, 0.0, 0.0,  # 1
        1.0, 1.0, 0.0,  # 2
        0.0, 1.0, 0.0,  # 3
        0.0, 0.0, 1.0,  # 4
        1.0, 0.0, 1.0,  # 5
        1.0, 1.0, 1.0,  # 6
        0.0, 1.0, 1.0,  # 7
    ]
    # Define indices (two triangles per face)
    idx = [
        0, 1, 2, 0, 2, 3,  # bottom face (z=0)
        4, 5, 6, 4, 6, 7,  # top face (z=1)
        0, 1, 5, 0, 5, 4,  # front face (y=0)
        3, 2, 6, 3, 6, 7,  # back face (y=1)
        0, 4, 7, 0, 7, 3,  # left face (x=0)
        1, 2, 6, 1, 6, 5,  # right face (x=1)
    ]
    return verts, idx


@pytest.mark.parametrize(
    "plane_type, offset, coord_index",
    [
        ("xy", 0.5, 2),  # slice midway along z
        ("xz", 0.5, 1),  # slice midway along y
        ("yz", 0.5, 0),  # slice midway along x
    ],
)
def test_cube_slice_produces_expected_segments(plane_type: str, offset: float, coord_index: int) -> None:
    """Slicing a unit cube mid‑way along any axis should produce four segments.

    The returned segment endpoints should lie exactly on the slicing plane
    coordinate (within a small tolerance).
    """
    verts, idx = create_unit_cube_mesh()
    plane = make_slice_plane(plane_type, offset)
    segs = intersect_mesh_with_plane(verts, idx, plane)
    # Expect four segments for a mid‑plane slice through the cube
    assert len(segs) == 4
    # Check each segment endpoint lies on the plane (within tolerance)
    for seg in segs:
        p1 = seg.p1
        p2 = seg.p2
        assert math.isclose(p1[coord_index], offset, abs_tol=1e-6)
        assert math.isclose(p2[coord_index], offset, abs_tol=1e-6)
        # For orthogonal coordinates, ensure values are within cube bounds
        for i in range(3):
            assert 0.0 - 1e-6 <= p1[i] <= 1.0 + 1e-6
            assert 0.0 - 1e-6 <= p2[i] <= 1.0 + 1e-6