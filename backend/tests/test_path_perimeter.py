"""
Tests for the perimeter path generator added in Phase 1.

These tests verify that the new ``generate_perimeter_path`` function
produces a closed, horizontal, rounded‑rectangle path that maintains
approximately the requested clearance from a bounding box.  They also
exercise the helper used by the API layer to ensure that the returned
points lie on a constant z plane and that the loop closure property
holds.
"""

import math
import sys
from pathlib import Path

import pytest

# Make backend modules importable when running tests standalone
sys.path.append(str(Path(__file__).resolve().parents[1]))

from app.services.path_planner import generate_perimeter_path


def _distance_to_bbox_xy(x: float, y: float, bbox_min: tuple[float, float], bbox_max: tuple[float, float]) -> float:
    """Compute horizontal distance from a point to an axis‑aligned bounding box.

    This helper returns the Euclidean distance in the XY plane from
    ``(x, y)`` to the rectangle defined by ``bbox_min`` and ``bbox_max``.
    If the point lies inside the rectangle along an axis, the distance
    contribution from that axis is zero.
    """
    min_x, min_y = bbox_min
    max_x, max_y = bbox_max
    # Distance along x axis: zero if inside the interval
    if x < min_x:
        dx = min_x - x
    elif x > max_x:
        dx = x - max_x
    else:
        dx = 0.0
    # Distance along y axis
    if y < min_y:
        dy = min_y - y
    elif y > max_y:
        dy = y - max_y
    else:
        dy = 0.0
    return math.hypot(dx, dy)


def test_generate_perimeter_path_basic_properties() -> None:
    """The perimeter generator should return a closed, level path with consistent clearance."""
    # Define an arbitrary bounding box and clearance offset
    bbox_min = (0.0, 0.0, 0.0)
    bbox_max = (10.0, 5.0, 4.0)
    offset = 20.0
    # Generate the path using the default sampling resolution
    points = generate_perimeter_path(
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        offset=offset,
        samples_per_side=32,
    )
    # Should not be empty
    assert len(points) > 0
    # Extract z coordinates and ensure all are equal to the bbox centre z
    cz_expected = 0.5 * (bbox_min[2] + bbox_max[2])
    for p in points:
        assert pytest.approx(p.z, rel=1e-6) == cz_expected
    # The first and last points should coincide (closed loop)
    first = points[0]
    last = points[-1]
    assert pytest.approx(first.x, rel=1e-6) == last.x
    assert pytest.approx(first.y, rel=1e-6) == last.y
    assert pytest.approx(first.z, rel=1e-6) == last.z
    # Compute the minimum horizontal distance from each point to the original bbox
    # All distances should be approximately equal to the offset within a reasonable tolerance.
    # We use a 5% relative tolerance to accommodate sampling discretisation.
    bbox_min_xy = (bbox_min[0], bbox_min[1])
    bbox_max_xy = (bbox_max[0], bbox_max[1])
    distances = [
        _distance_to_bbox_xy(p.x, p.y, bbox_min_xy, bbox_max_xy)
        for p in points
    ]
    min_dist = min(distances)
    max_dist = max(distances)
    assert pytest.approx(min_dist, rel=0.05) == offset
    assert pytest.approx(max_dist, rel=0.05) == offset