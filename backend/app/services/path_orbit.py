
"""
Orbit-style flythrough path generator.

This module generates a simple circular orbit around a model's
axis-aligned bounding box.  It is used by the "orbit" strategy
on the API layer.
"""

from __future__ import annotations

import math
from typing import Iterable, List

from ..api.models import PathPoint


def generate_orbit_path(
    bbox_min: Iterable[float],
    bbox_max: Iterable[float],
    camera_radius: float,
    num_samples: int = 128,
    factor: float = 1.5,
) -> List[PathPoint]:
    """Generate a circular orbit path around the model bounding box.

    The path lies in the horizontal plane (constant z) and encircles
    the model at a radius proportional to its largest dimension.

    Args:
        bbox_min: The minimum x, y, z of the model's bounding box.
        bbox_max: The maximum x, y, z of the model's bounding box.
        camera_radius: Minimum clearance distance from the model
            (currently unused in this simple implementation).
        num_samples: Number of points to sample along the path.
        factor: Multiplier applied to the largest dimension to
            determine orbit radius.

    Returns:
        A list of :class:`PathPoint` objects defining the orbit.
    """
    min_x, min_y, min_z = bbox_min
    max_x, max_y, max_z = bbox_max

    # Compute centre and size of the bounding box
    cx = (min_x + max_x) / 2.0
    cy = (min_y + max_y) / 2.0
    cz = (min_z + max_z) / 2.0

    sx = max_x - min_x
    sy = max_y - min_y
    sz = max_z - min_z

    # Largest dimension
    max_dim = max(sx, sy, sz)

    # Determine orbit radius: scale by factor and add camera radius
    R = max_dim * factor + camera_radius

    points: List[PathPoint] = []
    for i in range(num_samples):
        angle = 2 * math.pi * i / num_samples
        x = cx + R * math.cos(angle)
        y = cy + R * math.sin(angle)
        z = cz  # keep z constant at model centre
        points.append(PathPoint(x=x, y=y, z=z))

    return points
