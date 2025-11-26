
"""
Simple circular path generator used in early Fly phases.

This module is intentionally small and self-contained so it can be
tested in isolation.  It remains available as a utility for quick
diagnostics and for any future features that need a trivial path.
"""

from __future__ import annotations

import math
from typing import List, Tuple

from ..api.models import PathPoint


def generate_circular_path(radius: float = 2.0, num_samples: int = 64) -> Tuple[List[PathPoint], float]:
    """Generate a circular path in the XY plane.

    Args:
        radius: Radius of the circle.
        num_samples: Number of points to sample along the circle.

    Returns:
        A tuple of (points, length) where `points` is a list of
        :class:`PathPoint` objects and `length` is the approximate
        perimeter of the circle.
    """
    points: List[PathPoint] = []
    for i in range(num_samples):
        angle = 2 * math.pi * i / num_samples
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = 0.0
        points.append(PathPoint(x=x, y=y, z=z))
    length = 2 * math.pi * radius
    return points, length
