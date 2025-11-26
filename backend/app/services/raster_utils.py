"""Lightweight raster utilities for Minkowski-style perimeter extraction."""

from __future__ import annotations

import math
import heapq
from typing import TYPE_CHECKING

try:
    import numpy as np  # type: ignore  # noqa: N816
except Exception as exc:  # pragma: no cover - dependency guard
    raise RuntimeError("numpy is required for raster utilities") from exc

if TYPE_CHECKING:  # pragma: no cover - for type checkers only
    from numpy.typing import NDArray


def _compute_distance_field_from_occupancy(grid: 'NDArray[np.uint8]', cell_size: float) -> 'NDArray[np.float64]':
    """Compute an 8-connected distance field from a binary occupancy grid."""

    height, width = grid.shape
    dist = np.full((height, width), np.inf, dtype=float)
    frontier: list[tuple[float, int, int]] = []
    for r in range(height):
        row = grid[r]
        for c in range(width):
            if row[c]:
                dist[r, c] = 0.0
                frontier.append((0.0, r, c))

    neighbours = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),
        (1, 1, math.sqrt(2)),
    ]

    heapq.heapify(frontier)
    while frontier:
        cur_dist, r, c = heapq.heappop(frontier)
        if cur_dist > dist[r, c]:
            continue
        for dr, dc, weight in neighbours:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= height or nc < 0 or nc >= width:
                continue
            cand = cur_dist + weight
            if cand < dist[nr, nc]:
                dist[nr, nc] = cand
                heapq.heappush(frontier, (cand, nr, nc))

    return dist * cell_size


def _marching_squares_distance_field(
    distance_field: 'NDArray[np.float64]',
    threshold: float,
    cell_size: float,
    min_u: float,
    min_v: float,
) -> list[list[tuple[float, float]]]:
    """Extract iso-contours at ``threshold`` from a distance field."""

    height, width = distance_field.shape
    adjacency: dict[tuple[float, float], list[tuple[float, float]]] = {}

    def _add_edge(p0: tuple[float, float], p1: tuple[float, float]) -> None:
        key0 = (round(p0[0], 10), round(p0[1], 10))
        key1 = (round(p1[0], 10), round(p1[1], 10))
        adjacency.setdefault(key0, []).append(key1)
        adjacency.setdefault(key1, []).append(key0)

    def interp(v0: float, v1: float, axis: str, base_x: float, base_y: float) -> tuple[float, float]:
        if v0 == v1:
            t = 0.5
        else:
            t = max(0.0, min(1.0, (threshold - v0) / (v1 - v0)))
        if axis == 'x':
            return base_x + t * cell_size, base_y
        return base_x, base_y + t * cell_size

    for row in range(height - 1):
        for col in range(width - 1):
            v_tl = distance_field[row, col]
            v_tr = distance_field[row, col + 1]
            v_br = distance_field[row + 1, col + 1]
            v_bl = distance_field[row + 1, col]
            tl = 1 if v_tl >= threshold else 0
            tr = 1 if v_tr >= threshold else 0
            br = 1 if v_br >= threshold else 0
            bl = 1 if v_bl >= threshold else 0
            pattern = (tl << 0) | (tr << 1) | (br << 2) | (bl << 3)
            if pattern == 0 or pattern == 15:
                continue
            base_x = min_u + col * cell_size
            base_y = min_v + row * cell_size

            top = interp(v_tl, v_tr, 'x', base_x, base_y)
            right = interp(v_tr, v_br, 'y', base_x + cell_size, base_y)
            bottom = interp(v_bl, v_br, 'x', base_x, base_y + cell_size)
            left = interp(v_tl, v_bl, 'y', base_x, base_y)

            if pattern in {1, 14}:
                _add_edge(left, top)
            if pattern in {2, 13}:
                _add_edge(top, right)
            if pattern in {3, 12}:
                _add_edge(left, right)
            if pattern in {4, 11}:
                _add_edge(right, bottom)
            if pattern in {5}:
                _add_edge(left, bottom)
                _add_edge(top, right)
            if pattern in {6, 9}:
                _add_edge(top, bottom)
            if pattern in {7, 8}:
                _add_edge(left, bottom)
            if pattern in {10}:
                _add_edge(top, right)
                _add_edge(left, bottom)

    loops_uv: list[list[tuple[float, float]]] = []
    while adjacency:
        start = next(iter(adjacency))
        path = [start]
        current = start
        prev: tuple[float, float] | None = None
        while True:
            neighbours = adjacency.get(current)
            if not neighbours:
                break
            next_pt = neighbours[0]
            if prev and len(neighbours) > 1 and neighbours[0] == prev:
                next_pt = neighbours[1]
            adjacency[current].remove(next_pt)
            if not adjacency[current]:
                adjacency.pop(current, None)
            adjacency[next_pt].remove(current)
            if not adjacency.get(next_pt):
                adjacency.pop(next_pt, None)
            prev, current = current, next_pt
            if current == start:
                break
            path.append(current)
        if len(path) >= 3:
            loops_uv.append(path)

    return loops_uv

