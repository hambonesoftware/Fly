
"""
Path validation utilities.

Provides a simple clearance check that validates a path relative to
the model's axis-aligned bounding box.
"""

from __future__ import annotations

import math
from typing import Iterable, List, Tuple

from ..api.models import PathPoint


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

    Returns:
        A tuple ``(valid, min_clearance, violations)`` where:

        * ``valid`` is True if all points meet the clearance requirement.
        * ``min_clearance`` is the minimum clearance across all points.
        * ``violations`` is a list of dictionaries with details of
          points that violate the clearance requirement.
    """
    min_x, min_y, min_z = bbox_min
    max_x, max_y, max_z = bbox_max

    min_clearance: float = float("inf")
    violations: List[dict] = []

    for idx, p in enumerate(points):
        # Distance to each pair of faces along axes
        # Compute signed distances from the box; negative values indicate the
        # point lies inside along that axis.  For clearance we use the
        # Euclidean distance to the box when outside, and zero when inside.
        dx = 0.0
        if p.x < min_x:
            dx = min_x - p.x
        elif p.x > max_x:
            dx = p.x - max_x

        dy = 0.0
        if p.y < min_y:
            dy = min_y - p.y
        elif p.y > max_y:
            dy = p.y - max_y

        dz = 0.0
        if p.z < min_z:
            dz = min_z - p.z
        elif p.z > max_z:
            dz = p.z - max_z

        clearance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if clearance < min_clearance:
            min_clearance = clearance

        if clearance <= camera_radius:
            violations.append(
                {
                    "index": idx,
                    "point": {"x": p.x, "y": p.y, "z": p.z},
                    "clearance": clearance,
                }
            )

    valid = len(violations) == 0
    if min_clearance == float("inf"):
        min_clearance = 0.0

    return valid, min_clearance, violations
