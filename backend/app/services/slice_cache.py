"""
Simple in‑memory caching layer for slice segments.

This module introduces a small helper to cache the results of
``slice_model_mesh`` calls.  Intersecting a tessellated model with a
plane can be expensive for large meshes, so repeated requests for
identical slicing parameters should reuse previously computed
segments.  A ``SliceCacheKey`` uniquely identifies a slice by
``model_id``, ``plane_type``, ``offset`` and level of detail (lod).

The cache is implemented as an ``OrderedDict`` to provide
least‑recently‑used (LRU) eviction.  When the number of cached
entries exceeds ``MAX_CACHE_ENTRIES`` the oldest entry is dropped.

Usage::

    from .slice_cache import SliceCacheKey, get_slice_segments_from_cache, put_slice_segments_in_cache
    key = SliceCacheKey(model_id="abc", plane_type="xy", offset=0.0, lod=0)
    segs = get_slice_segments_from_cache(key)
    if segs is None:
        segs = slice_model_mesh(model_id, plane)
        put_slice_segments_in_cache(key, segs)

"""

from __future__ import annotations

from dataclasses import dataclass
from collections import OrderedDict
from threading import RLock
from typing import List, Optional

from .slicing import SliceSegment

@dataclass(frozen=True)
class SliceCacheKey:
    """Unique identifier for a cached slice result.

    Attributes:
        model_id: Identifier of the model being sliced.
        plane_type: One of ``"xy"``, ``"xz"`` or ``"yz"``.
        offset: Position of the slicing plane along its orthogonal axis.
        lod: Level of detail (reserved for future use; currently always 0).
    """

    model_id: str
    plane_type: str
    offset: float
    lod: int = 0


# Underlying storage for the slice cache.  The key is a
# ``SliceCacheKey`` and the value is the list of ``SliceSegment``
# objects returned by ``intersect_mesh_with_plane``.  A reentrant lock
# protects the dictionary to allow safe concurrent access.
_cache: "OrderedDict[SliceCacheKey, List[SliceSegment]]" = OrderedDict()
_lock = RLock()
# Maximum number of entries retained in the cache.  Once this limit
# is reached the least recently used entry is evicted on insertion of
# a new entry.  This prevents unbounded memory growth under heavy use.
MAX_CACHE_ENTRIES: int = 32


def get_slice_segments_from_cache(key: SliceCacheKey) -> Optional[List[SliceSegment]]:
    """Retrieve cached slice segments if available.

    Args:
        key: Cache key identifying the slice.

    Returns:
        A list of ``SliceSegment`` objects if present in the cache,
        otherwise ``None``.
    """
    with _lock:
        segs = _cache.get(key)
        if segs is not None:
            # Move the key to the end to mark it as recently used
            _cache.move_to_end(key)
        return segs


def put_slice_segments_in_cache(key: SliceCacheKey, segments: List[SliceSegment]) -> None:
    """Store slice segments in the cache.

    If the cache exceeds its configured capacity after insertion the
    least recently used entry is removed.

    Args:
        key: Cache key identifying the slice.
        segments: List of ``SliceSegment`` objects to cache.
    """
    with _lock:
        # Insert/update entry
        _cache[key] = segments
        _cache.move_to_end(key)
        # Evict least recently used if capacity exceeded
        if len(_cache) > MAX_CACHE_ENTRIES:
            _cache.popitem(last=False)