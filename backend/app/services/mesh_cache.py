"""
Mesh cache serialization utilities.

This module provides helper functions to save and load triangulated
mesh data to/from disk using compressed NumPy archives (``.npz``).
Meshes are stored as a combination of vertices, triangle indices,
vertex normals, and bounding box limits.  The archive format uses
NumPy arrays with explicit data types to minimise storage space and
loading time.  Loading functions return plain Python lists for
compatibility with the existing API schemas.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Tuple

import numpy as np


def save_mesh_cache(
    path: Path,
    vertices: List[float],
    indices: List[int],
    normals: List[float],
    bbox_min: List[float],
    bbox_max: List[float],
) -> None:
    """Write mesh data to a compressed ``.npz`` file.

    Args:
        path: Destination file path.  Parent directories will not be
            created; callers should ensure the directory exists.
        vertices: Flat list of vertex coordinates (x, y, z) in
            model coordinate space.
        indices: Flat list of indices forming triangles (triplets
            referencing positions in ``vertices``).
        normals: Flat list of per‑vertex normals (x, y, z) or
            placeholder zeros.  Must correspond 1:1 with vertices.
        bbox_min: List of three floats representing the minimum x, y,
            z coordinates of the bounding box.
        bbox_max: List of three floats representing the maximum x, y,
            z coordinates of the bounding box.
    """
    # Convert lists into NumPy arrays with explicit dtypes.  Vertices
    # and normals are reshaped into (N, 3) to preserve semantics.  We
    # use float32 for compactness; indices use uint32 as meshes rarely
    # exceed 4 billion vertices.
    vertices_arr = np.array(vertices, dtype=np.float32).reshape(-1, 3)
    indices_arr = np.array(indices, dtype=np.uint32)
    normals_arr = np.array(normals, dtype=np.float32).reshape(-1, 3)
    bbox_min_arr = np.array(bbox_min, dtype=np.float32)
    bbox_max_arr = np.array(bbox_max, dtype=np.float32)
    # Save using NumPy's compressed format.  Field names are chosen to
    # reflect the stored content for ease of debugging.
    np.savez_compressed(
        path,
        vertices=vertices_arr,
        indices=indices_arr,
        normals=normals_arr,
        bbox_min=bbox_min_arr,
        bbox_max=bbox_max_arr,
    )


def load_mesh_cache(
    path: Path,
) -> Tuple[List[float], List[int], List[float], List[float], List[float]]:
    """Load mesh data from a compressed ``.npz`` file.

    Args:
        path: File path to the ``.npz`` archive.

    Returns:
        A tuple containing:
            - vertices: flat list of floats
            - indices: flat list of ints
            - normals: flat list of floats
            - bbox_min: list of three floats
            - bbox_max: list of three floats

    Raises:
        FileNotFoundError: If the specified path does not exist.
        ValueError: If the loaded archive does not contain the
            expected fields.
    """
    if not path.exists():
        raise FileNotFoundError(f"Mesh cache file not found: {path}")
    data = np.load(path, allow_pickle=False)
    required_keys = {"vertices", "indices", "normals", "bbox_min", "bbox_max"}
    if not required_keys.issubset(data.files):
        missing = required_keys - set(data.files)
        raise ValueError(f"Mesh cache file is missing fields: {missing}")
    vertices_arr = data["vertices"].astype(np.float32)
    indices_arr = data["indices"].astype(np.uint32)
    normals_arr = data["normals"].astype(np.float32)
    bbox_min_arr = data["bbox_min"].astype(np.float32)
    bbox_max_arr = data["bbox_max"].astype(np.float32)
    # Flatten arrays to Python lists for API compatibility
    vertices_list = vertices_arr.reshape(-1).tolist()
    indices_list = indices_arr.reshape(-1).tolist()
    normals_list = normals_arr.reshape(-1).tolist()
    bbox_min_list = bbox_min_arr.tolist()
    bbox_max_list = bbox_max_arr.tolist()
    return vertices_list, indices_list, normals_list, bbox_min_list, bbox_max_list