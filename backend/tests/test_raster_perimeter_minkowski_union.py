from __future__ import annotations

import sys
from pathlib import Path

# Align import style with existing geometry tests to avoid duplicate SQLModel metadata
sys.path.append(str(Path(__file__).resolve().parents[1]))

from app.services import geometry  # type: ignore
from app.services.geometry import compute_raster_perimeter_polygons  # type: ignore
from app.api.models import MeshBBox, MeshResponse  # type: ignore


def _make_two_square_mesh(size: float, gap: float) -> MeshResponse:
    """Return a flat mesh with two separated squares in the XZ plane."""

    half = size / 2.0
    centers = [-(size + gap) / 2.0, (size + gap) / 2.0]
    vertices: list[float] = []
    indices: list[int] = []
    normals: list[float] = []

    for center_x in centers:
        base = len(vertices) // 3
        square_verts = [
            (center_x - half, 0.0, -half),
            (center_x + half, 0.0, -half),
            (center_x + half, 0.0, half),
            (center_x - half, 0.0, half),
        ]
        for vx, vy, vz in square_verts:
            vertices.extend([vx, vy, vz])
            normals.extend([0.0, 1.0, 0.0])
        indices.extend([base, base + 1, base + 2, base, base + 2, base + 3])

    min_x = centers[0] - half
    max_x = centers[1] + half
    bbox = MeshBBox(min=[min_x, 0.0, -half], max=[max_x, 0.0, half])
    return MeshResponse(
        modelId="two-squares",
        vertices=vertices,
        indices=indices,
        normals=normals,
        bbox=bbox,
    )


def test_union_minkowski_merges_close_islands(monkeypatch) -> None:
    """Gaps narrower than 2r should disappear after union-first Minkowski."""

    mesh = _make_two_square_mesh(size=10.0, gap=6.0)

    def _fake_get_mesh(model_id: str) -> MeshResponse:  # pragma: no cover - deterministic
        return mesh

    monkeypatch.setattr(geometry, "get_mesh_for_model", _fake_get_mesh)

    loops, _, meta = compute_raster_perimeter_polygons(
        model_id="two-squares",
        plane="xz",
        detail="high",
        island_mode="multi",
        clearance_radius=4.0,
    )

    # With gap (6) < 2r (8) the inflated obstacles should merge into one outline.
    assert len(loops) == 1
    assert meta["clearanceRadius"] == 4.0


def test_union_minkowski_preserves_wide_gap(monkeypatch) -> None:
    """Gaps wider than 2r should remain as separate islands."""

    mesh = _make_two_square_mesh(size=10.0, gap=12.0)

    def _fake_get_mesh(model_id: str) -> MeshResponse:  # pragma: no cover - deterministic
        return mesh

    monkeypatch.setattr(geometry, "get_mesh_for_model", _fake_get_mesh)

    loops, _, _ = compute_raster_perimeter_polygons(
        model_id="two-squares",
        plane="xz",
        detail="high",
        island_mode="multi",
        clearance_radius=4.0,
    )

    # With gap (12) > 2r (8) the outlines should stay as two separate islands.
    assert len(loops) == 2
