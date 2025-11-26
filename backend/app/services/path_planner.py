
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
from .path_perimeter import (
    generate_perimeter_path,
    generate_rolling_circle_perimeter_path,
)
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
