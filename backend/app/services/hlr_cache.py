"""
In‑memory caching layer for HLR‑derived model outlines.

Computing a hidden‑line removal (HLR) projection of a CAD model can be
expensive for large assemblies.  This module provides a tiny LRU cache
for the 3D perimeter polylines returned by
``compute_hlr_perimeter_for_model`` so that repeated perimeter
requests with identical parameters reuse previously computed results.

The cache key uniquely identifies an outline by:

- ``model_id`` – ID of the model being processed.
- ``plane`` – projection plane (``"xy"``, ``"xz"``, ``"yz"``).
- ``stitch_tolerance`` – tolerance used to connect edges into wires.
- ``world_step`` – effective sampling step length in model units.
- ``detail`` – normalised outline detail level (``"low"``, ``"medium"``, ``"high"``, ``"auto"``).

The implementation mirrors the structure of :mod:`slice_cache` using an
``OrderedDict`` with least‑recently‑used (LRU) eviction.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from threading import RLock
from typing import List, Tuple, Optional


@dataclass(frozen=True)
class HlrCacheKey:
    """Unique identifier for a cached HLR perimeter.

    Attributes:
        model_id: Identifier of the model being projected.
        plane: Projection plane ("xy", "xz" or "yz").
        stitch_tolerance: Stitching tolerance used when connecting edges.
        world_step: Effective sampling step length in model units.
        detail: Normalised outline detail level.
    """

    model_id: str
    plane: str
    stitch_tolerance: float
    world_step: float
    detail: str


# Underlying storage for the HLR cache.  The key is a :class:`HlrCacheKey`
# and the value is the list of 3D points returned by the geometry layer.
_cache: "OrderedDict[HlrCacheKey, List[Tuple[float, float, float]]]" = OrderedDict()

# Maximum number of perimeter entries to retain before evicting the
# least recently used one.
MAX_CACHE_ENTRIES = 64

# Lock guarding cache access so multiple threads can safely use the
# cache in parallel.
_lock = RLock()


def get_hlr_perimeter_from_cache(key: HlrCacheKey) -> Optional[List[Tuple[float, float, float]]]:
    """Retrieve a perimeter polyline from the cache.

    Args:
        key: Cache key identifying the perimeter.

    Returns:
        The cached list of 3D points if present, otherwise ``None``.
    """
    with _lock:
        pts = _cache.get(key)
        if pts is not None:
            # Mark as recently used
            _cache.move_to_end(key)
        return pts


def put_hlr_perimeter_in_cache(key: HlrCacheKey, points: List[Tuple[float, float, float]]) -> None:
    """Store a perimeter polyline in the cache.

    If the cache exceeds its capacity after insertion the least
    recently used entry is removed.

    Args:
        key: Cache key.
        points: List of 3D points to cache.
    """
    with _lock:
        _cache[key] = points
        _cache.move_to_end(key)
        if len(_cache) > MAX_CACHE_ENTRIES:
            _cache.popitem(last=False)
