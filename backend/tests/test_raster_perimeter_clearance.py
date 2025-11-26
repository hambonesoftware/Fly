"""Regression tests for raster perimeter extraction with large clearances."""

from __future__ import annotations

import sys
from pathlib import Path

# Align import style with existing geometry tests to avoid duplicate SQLModel metadata
sys.path.append(str(Path(__file__).resolve().parents[1]))

from app.services import geometry  # type: ignore
from app.services.geometry import compute_raster_perimeter_polygons  # type: ignore
from app.api.models import MeshResponse, MeshBBox  # type: ignore


def _make_flat_square_mesh(size: float) -> MeshResponse:
    """Create a simple flat square mesh in the XZ plane."""

    half = size / 2.0
    vertices = [
        -half,
        0.0,
        -half,
        half,
        0.0,
        -half,
        half,
        0.0,
        half,
        -half,
        0.0,
        half,
    ]
    indices = [0, 1, 2, 0, 2, 3]
    # Upward normals for each vertex
    normals = [0.0, 1.0, 0.0] * 4
    bbox = MeshBBox(min=[-half, 0.0, -half], max=[half, 0.0, half])
    return MeshResponse(
        modelId="square",
        vertices=vertices,
        indices=indices,
        normals=normals,
        bbox=bbox,
    )


def test_large_clearance_expands_grid(monkeypatch) -> None:
    """Ensure Minkowski extraction returns loops when clearance exceeds the mesh size."""

    mesh = _make_flat_square_mesh(100.0)

    def _fake_get_mesh(model_id: str) -> MeshResponse:  # pragma: no cover - deterministic
        return mesh

    monkeypatch.setattr(geometry, "get_mesh_for_model", _fake_get_mesh)

    loops, areas, meta = compute_raster_perimeter_polygons(
        model_id="square",
        plane="xz",
        clearance_radius=50.0,
        detail="auto",
    )

    assert len(loops) == 1
    xs = [p[0] for p in loops[0]]
    zs = [p[2] for p in loops[0]]

    # The clearance should expand the 100x100 square by roughly 50 units each side
    assert min(xs) < -90.0 and max(xs) > 90.0
    assert min(zs) < -90.0 and max(zs) > 90.0
    # Resulting area should be on the order of (100 + 2*50)^2 = 40000
    assert areas and areas[0] > 30000.0
