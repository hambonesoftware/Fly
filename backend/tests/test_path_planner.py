"""
Unit tests for path planning and validation logic.

These tests target the internal path planner functions defined in
``backend/app/services/path_planner.py``.  They verify that the
orbit path generator produces a closed circular orbit around a
bounding box with the expected radius, and that the clearance
validation correctly identifies violations.  The tests do not rely
on ``pythonocc-core`` and therefore run regardless of whether OCC is
installed.
"""

import math
import sys
from pathlib import Path

import pytest

# Add backend to sys.path for importing modules
sys.path.append(str(Path(__file__).resolve().parents[1]))

from app.services.path_planner import generate_orbit_path, validate_path
from app.api.models import PathPoint


def test_orbit_path_radius_and_closure() -> None:
    """generate_orbit_path should produce a closed orbit of expected radius."""
    # Define a bounding box centred at origin with size 2x2x2
    bbox_min = (-1.0, -1.0, -1.0)
    bbox_max = (1.0, 1.0, 1.0)
    camera_radius = 0.5
    num_samples = 64
    factor = 1.5
    points = generate_orbit_path(
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        camera_radius=camera_radius,
        num_samples=num_samples,
        factor=factor,
    )
    # Check that number of points matches the requested samples
    assert len(points) == num_samples
    # Compute centre and expected radius
    cx = (bbox_min[0] + bbox_max[0]) / 2.0
    cy = (bbox_min[1] + bbox_max[1]) / 2.0
    # Largest dimension of the bounding box
    sx = bbox_max[0] - bbox_min[0]
    sy = bbox_max[1] - bbox_min[1]
    sz = bbox_max[2] - bbox_min[2]
    max_dim = max(sx, sy, sz)
    expected_radius = max_dim * factor + camera_radius
    # Verify each point lies approximately at the expected radius from centre
    # within a small tolerance
    for p in points:
        r = math.hypot(p.x - cx, p.y - cy)
        assert pytest.approx(r, rel=1e-2) == expected_radius
        # Z coordinate should be centred
        assert pytest.approx(p.z, rel=1e-6) == (bbox_min[2] + bbox_max[2]) / 2.0
    # Check closure: first and last points should coincide (approx)
    first = points[0]
    last = points[-1]
    assert pytest.approx(first.x, rel=1e-6) == last.x
    assert pytest.approx(first.y, rel=1e-6) == last.y
    assert pytest.approx(first.z, rel=1e-6) == last.z


def test_validate_path_detects_violations() -> None:
    """validate_path should flag points too close to the bounding box."""
    bbox_min = (-1.0, -1.0, -1.0)
    bbox_max = (1.0, 1.0, 1.0)
    camera_radius = 0.5
    # Construct a path with one point inside the bounding box and one outside
    points = [
        PathPoint(x=0.0, y=0.0, z=0.0),  # inside -> violation
        PathPoint(x=2.5, y=0.0, z=0.0),  # outside -> safe
    ]
    valid, min_clearance, violations = validate_path(
        points=points,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        camera_radius=camera_radius,
    )
    # Should not be valid because first point is inside
    assert valid is False
    # There should be exactly one violation (the inside point)
    assert len(violations) == 1
    assert violations[0]["index"] == 0
    # Minimum clearance should be zero for a point inside
    assert pytest.approx(min_clearance, rel=1e-6) == 0.0
    # Now test a path that is entirely outside the bounding box
    safe_points = [PathPoint(x=3.0, y=0.0, z=0.0), PathPoint(x=0.0, y=3.0, z=0.0)]
    valid2, min_clearance2, violations2 = validate_path(
        points=safe_points,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        camera_radius=camera_radius,
    )
    assert valid2 is True
    # Minimum clearance should be positive and larger than camera radius
    assert min_clearance2 > camera_radius
    assert len(violations2) == 0