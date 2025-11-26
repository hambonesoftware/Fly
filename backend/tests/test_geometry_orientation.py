"""Tests for polygon orientation normalisation in the raster pipeline."""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

from app.services.geometry import _orient_loops_ccw  # type: ignore


def test_orient_loops_ccw_reverses_clockwise_loops() -> None:
    """Clockwise loops should be reversed to counter-clockwise order."""

    # Square loop ordered clockwise in the xy plane (ax0=0, ax1=1)
    cw_loop = [
        (0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
    ]

    oriented = _orient_loops_ccw([cw_loop], ax0=0, ax1=1)

    assert len(oriented) == 1
    # Counter-clockwise square should start at the same vertex but progress
    # in the opposite order to the clockwise input.
    assert oriented[0] == list(reversed(cw_loop))


def test_orient_loops_ccw_leaves_degenerate_loops_unchanged() -> None:
    """Loops with fewer than three points should be returned as-is."""

    degenerate = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]

    oriented = _orient_loops_ccw([degenerate], ax0=0, ax1=1)

    assert oriented[0] is degenerate
