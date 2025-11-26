
"""
High-level path planning orchestrator for Fly.

This module is a thin facade that re-exports the main path-planning
entry points from more focused submodules.  The public API remains
compatible with earlier versions so that existing imports like::

    from backend.app.services.path_planner import (
        generate_circular_path,
        generate_orbit_path,
        generate_perimeter_path,
        ORBIT_STRATEGY,
        PERIMETER_STRATEGY,
        validate_path as validate_path_points,
    )

continue to work unchanged.
"""

from __future__ import annotations

from typing import Iterable, List, Tuple

from ..api.models import PathPoint

# Re-exported helpers from specialised submodules
from .path_circular import generate_circular_path
from .path_orbit import generate_orbit_path
from .path_perimeter import generate_rolling_circle_perimeter_path
from .path_perimeter import generate_perimeter_path as _generate_perimeter_path_impl
from .path_validation import validate_path

# -------------------------------------------------------------------------
# Path strategy identifiers
#
# These constants are imported by the API layer (routes_paths.py) and used
# by the frontend when selecting which strategy to invoke.
# -------------------------------------------------------------------------

ORBIT_STRATEGY: str = "orbit"
PERIMETER_STRATEGY: str = "perimeter"

__all__ = [
    "ORBIT_STRATEGY",
    "PERIMETER_STRATEGY",
    "generate_circular_path",
    "generate_orbit_path",
    "generate_perimeter_path",
    "generate_rolling_circle_perimeter_path",
    "validate_path",
    "PathPoint",
]


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
    """Compatibility wrapper that forwards to the perimeter implementation.

    The API mirrors the parameters used by ``routes_paths.create_path`` and simply
    delegates to :mod:`path_perimeter`.  Keeping this function here preserves
    existing import paths while allowing the implementation to evolve.
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
