"""
Utilities and abstractions for slicing operations.

This module introduces a simple, pure‑Python representation of a
slice plane as well as helpers to construct such objects from the
user‑provided plane selection and height offset parameters.  A
``SlicePlane`` encapsulates the orientation of a slicing plane in
standard Cartesian coordinates and provides both an origin point on
the plane and a normal vector perpendicular to it.  These values
allow later phases of the project to perform geometric operations
(such as triangle–plane intersection) without depending on any
OpenCascade types or other external libraries.

The ``make_slice_plane`` function normalises input strings and fills
out the appropriate origin and normal based on the three supported
planes (``"xy"``, ``"xz"`` and ``"yz"``).  It also records the
offset provided by the caller for convenience.  Debug logging can be
enabled via the ``SLICE_DEBUG`` environment variable; when set, the
constructor helper will emit a concise message describing the plane
being constructed.
"""

from __future__ import annotations

import os
import logging
from dataclasses import dataclass
from typing import Literal, Tuple

logger = logging.getLogger(__name__)


@dataclass
class SlicePlane:
    """Immutable representation of a slice plane in 3D space.

    Attributes:
        plane_type: One of ``"xy"``, ``"xz"`` or ``"yz"`` indicating which
            principal plane the slice lies in.  The order corresponds
            to the two coordinate axes that remain free in the plane.
        offset: The constant value along the axis orthogonal to the
            plane.  For example, an ``"xy"`` plane with ``offset=0.5``
            has all points satisfying ``z = 0.5``.
        origin: A representative point on the plane.  This is a
            3‑tuple of floats and by convention has the fixed
            coordinate equal to ``offset`` and the remaining
            coordinates set to zero.  Other origins on the plane may
            be used interchangeably in later computations.
        normal: A unit vector perpendicular to the plane.  It is one
            of ``(0,0,1)``, ``(0,1,0)`` or ``(1,0,0)`` depending on
            ``plane_type``.
    """

    plane_type: Literal["xy", "xz", "yz"]
    offset: float
    origin: Tuple[float, float, float]
    normal: Tuple[float, float, float]


def make_slice_plane(plane: str, offset: float) -> SlicePlane:
    """Construct a :class:`SlicePlane` from a plane identifier and offset.

    The plane identifier is normalised to lowercase and validated
    against the supported values (``"xy"``, ``"xz"`` and ``"yz"``).
    Depending on the selection, the origin and normal are computed
    according to the following mapping:

    - ``"xy"``: origin = (0, 0, offset), normal = (0, 0, 1)
    - ``"xz"``: origin = (0, offset, 0), normal = (0, 1, 0)
    - ``"yz"``: origin = (offset, 0, 0), normal = (1, 0, 0)

    Debug messages are emitted when the environment variable
    ``SLICE_DEBUG`` evaluates to truthy.  The log includes the plane
    type, offset, origin and normal for easy inspection.

    Args:
        plane: String identifying the plane.  Case insensitive; any
            unrecognised value defaults to ``"xy"``.
        offset: Constant coordinate along the axis orthogonal to the
            plane.

    Returns:
        SlicePlane: A new dataclass instance encapsulating the plane
            selection.
    """
    plane_norm = (plane or "xy").strip().lower()
    if plane_norm not in {"xy", "xz", "yz"}:
        # Default to xy if an unknown plane is provided
        plane_norm = "xy"
    if plane_norm == "xy":
        origin = (0.0, 0.0, float(offset))
        normal = (0.0, 0.0, 1.0)
    elif plane_norm == "xz":
        origin = (0.0, float(offset), 0.0)
        normal = (0.0, 1.0, 0.0)
    else:  # "yz"
        origin = (float(offset), 0.0, 0.0)
        normal = (1.0, 0.0, 0.0)
    slice_plane = SlicePlane(
        plane_type=plane_norm,
        offset=float(offset),
        origin=origin,
        normal=normal,
    )
    # Emit debug logs when SLICE_DEBUG is enabled
    if os.getenv("SLICE_DEBUG"):
        logger.debug(
            "SlicePlane created: plane_type=%s offset=%s origin=%s normal=%s",
            slice_plane.plane_type,
            slice_plane.offset,
            slice_plane.origin,
            slice_plane.normal,
        )
    return slice_plane


# Public symbols exported by this module.  New names added in Phase 2 allow
# higher‑level code to import intersection utilities directly.
__all__ = [
    # Core slice plane types and helpers
    "SlicePlane",
    "make_slice_plane",
    # Geometry and intersection primitives
    "SliceSegment",
    "dot",
    "sub",
    "add",
    "scale",
    "signed_distance_to_plane",
    "intersect_triangle_with_plane",
    "intersect_mesh_with_plane",
    # Phase 3 – loop construction and perimeter selection
    "SliceLoop",
    "project_point_to_plane_uv",
    "build_snapped_points",
    "build_loops_from_segments",
    "find_outer_perimeter_loop",
    "compute_slice_perimeter_loop",
    # Phase 4 – offset, simplify, spline and lifting helpers
    "offset_polygon_2d",
    "simplify_polyline_rdp",
    "choose_simplification_tolerance",
    "BSplinePath2D",
    "fit_uniform_closed_bspline",
    "sample_bspline",
    "lift_uv_to_3d",
]

# --- Phase 2 additions: mesh–plane intersection utilities ---

from dataclasses import dataclass
from typing import List, Tuple, Iterable, Optional
import math

# --- Phase 3 additions: segment stitching, loop construction and perimeter selection ---
from typing import Dict


@dataclass
class SliceLoop:
    """A closed loop of slice segments projected onto a plane.

    A loop consists of an ordered list of 3D points, the corresponding 2D
    projection in the plane's coordinate space and the signed area of the
    polygon in 2D.  The area is positive for one winding direction (e.g.
    counter‑clockwise) and negative for the other.  Absolute area can
    therefore be used to determine the outermost perimeter.
    """

    points_3d: List[Tuple[float, float, float]]
    points_2d: List[Tuple[float, float]]
    area: float


def project_point_to_plane_uv(p: Tuple[float, float, float], plane: SlicePlane) -> Tuple[float, float]:
    """Project a 3D point onto the 2D coordinate system of a slice plane.

    The slice plane defines which axes remain free (e.g. ``"xy"`` keeps the
    x and y coordinates).  This helper returns the (u, v) pair used for
    planar computations such as area.  The constant axis defined by
    ``plane.offset`` is ignored.

    Args:
        p: The 3D point to project.
        plane: The slice plane specifying the orientation.

    Returns:
        A tuple ``(u, v)`` representing the projected coordinates.
    """
    if plane.plane_type == "xy":
        return (p[0], p[1])
    elif plane.plane_type == "xz":
        return (p[0], p[2])
    else:  # "yz"
        return (p[1], p[2])


def _polygon_area_2d(points_uv: List[Tuple[float, float]]) -> float:
    """Compute the signed area of a 2D polygon using the shoelace formula.

    The polygon is assumed to be closed (i.e. the first point is implicitly
    connected to the last).  The area returned is positive for one
    orientation (counter‑clockwise) and negative for the other.

    Args:
        points_uv: Sequence of (u, v) points defining the polygon.

    Returns:
        The signed area of the polygon.
    """
    n = len(points_uv)
    if n < 3:
        return 0.0
    area = 0.0
    for i in range(n):
        u0, v0 = points_uv[i]
        u1, v1 = points_uv[(i + 1) % n]
        area += u0 * v1 - u1 * v0
    return 0.5 * area


def build_snapped_points(
    segments: List[SliceSegment],
    eps: float,
) -> Tuple[List[Tuple[float, float, float]], List[Tuple[int, int]]]:
    """Deduplicate segment endpoints using a snapping tolerance.

    This helper bins 3D points by rounding each coordinate to the nearest
    multiple of ``eps``.  Points whose snapped keys coincide are treated
    as identical.  Unique canonical points are returned along with a list
    of undirected edges connecting their indices.

    Args:
        segments: The list of raw slice segments.
        eps: Snapping tolerance controlling bin size.

    Returns:
        A tuple ``(points, edges)`` where ``points`` is a list of unique
        snapped 3D coordinates and ``edges`` is a list of ``(i, j)`` pairs
        referencing ``points``.  Edges are undirected; duplicate edges are
        removed.
    """
    key_to_index: Dict[Tuple[int, int, int], int] = {}
    points: List[Tuple[float, float, float]] = []
    edges: List[Tuple[int, int]] = []
    if eps <= 0.0:
        raise ValueError("eps must be positive for snapping")

    def snap_key(pt: Tuple[float, float, float]) -> Tuple[int, int, int]:
        return (
            int(round(pt[0] / eps)),
            int(round(pt[1] / eps)),
            int(round(pt[2] / eps)),
        )

    for seg in segments:
        # Snap both endpoints
        pts = [seg.p1, seg.p2]
        indices: List[int] = []
        for pt in pts:
            key = snap_key(pt)
            idx = key_to_index.get(key)
            if idx is None:
                idx = len(points)
                key_to_index[key] = idx
                points.append(pt)
            indices.append(idx)
        # Add undirected edge
        i, j = indices
        if i == j:
            # Degenerate segment; skip
            continue
        edges.append((i, j))
    # Deduplicate edges by treating them as undirected
    unique_edges_set = set()
    unique_edges: List[Tuple[int, int]] = []
    for i, j in edges:
        # Sort tuple to ensure consistent ordering
        edge_key = (i, j) if i < j else (j, i)
        if edge_key not in unique_edges_set:
            unique_edges_set.add(edge_key)
            unique_edges.append(edge_key)
    return points, unique_edges


def build_loops_from_segments(
    segments: List[SliceSegment],
    plane: SlicePlane,
    snap_eps: float = 1e-4,
) -> List[SliceLoop]:
    """Construct closed loops from raw slice segments.

    This function snaps endpoints to merge nearly coincident vertices,
    builds an undirected adjacency graph and walks that graph to form
    closed loops.  Each loop is then projected onto the plane and its
    signed area computed.

    Args:
        segments: Intersection segments from :func:`intersect_mesh_with_plane`.
        plane: The slice plane used to generate the segments.
        snap_eps: Tolerance for endpoint snapping.

    Returns:
        A list of :class:`SliceLoop` objects representing closed loops.  The
        order of points in each loop corresponds to the traversal order.
    """
    if not segments:
        return []
    # Snap endpoints and build unique point list and undirected edges
    points, edges = build_snapped_points(segments, snap_eps)
    # Build adjacency list
    adj: Dict[int, List[int]] = {i: [] for i in range(len(points))}
    for i, j in edges:
        adj[i].append(j)
        adj[j].append(i)
    # Track visited edges using undirected representation
    visited_edges: set[frozenset[int]] = set()
    loops: List[SliceLoop] = []
    # Helper to mark edge visited
    def mark_edge(a: int, b: int) -> None:
        visited_edges.add(frozenset({a, b}))

    # Iterate over all edges to start traversals
    for (i_start, j_start) in edges:
        edge_key = frozenset({i_start, j_start})
        if edge_key in visited_edges:
            continue
        # Begin a new loop
        loop_indices: List[int] = []
        prev = i_start
        curr = j_start
        mark_edge(prev, curr)
        # Add starting vertex
        loop_indices.append(prev)
        # Walk until returning to start or getting stuck
        while True:
            loop_indices.append(curr)
            # Determine next neighbor: choose any neighbor not equal to prev and whose edge is unvisited
            next_idx: Optional[int] = None
            for nb in adj[curr]:
                if nb == prev:
                    continue
                if frozenset({curr, nb}) not in visited_edges:
                    next_idx = nb
                    break
            if next_idx is None:
                # Dead end; break
                break
            # Advance
            mark_edge(curr, next_idx)
            prev, curr = curr, next_idx
            # If we returned to starting vertex, we close the loop
            if curr == loop_indices[0]:
                break
        # Verify loop contains at least 3 unique vertices and is closed
        if len(loop_indices) >= 4 and loop_indices[0] == loop_indices[-1]:
            # Remove duplicate last element for processing
            loop_indices = loop_indices[:-1]
        # Ensure at least 3 points
        unique_vertex_count = len(set(loop_indices))
        if unique_vertex_count < 3:
            continue
        # Build 3D and 2D point lists in order
        pts_3d = [points[idx] for idx in loop_indices]
        pts_2d = [project_point_to_plane_uv(p, plane) for p in pts_3d]
        area = _polygon_area_2d(pts_2d)
        loops.append(SliceLoop(points_3d=pts_3d, points_2d=pts_2d, area=area))
    return loops


def find_outer_perimeter_loop(loops: List[SliceLoop]) -> Optional[SliceLoop]:
    """Select the loop with the largest absolute area.

    Args:
        loops: List of loops to consider.

    Returns:
        The loop whose absolute area is maximal, or ``None`` if there are no loops.
    """
    if not loops:
        return None
    return max(loops, key=lambda lp: abs(lp.area))


def compute_slice_perimeter_loop(model_id: str, plane: SlicePlane) -> Optional[SliceLoop]:
    """Compute the outermost perimeter loop for a given model and slice plane.

    This convenience wrapper slices the model mesh, constructs loops and
    returns the loop with the largest absolute area.  It returns
    ``None`` if no segments or loops are found.

    Args:
        model_id: Identifier of the model to slice.
        plane: The slice plane definition.

    Returns:
        The outermost :class:`SliceLoop` or ``None`` if unavailable.
    """
    # Import lazily to avoid circular dependency
    from .geometry import slice_model_mesh

    segments = slice_model_mesh(model_id, plane)
    if not segments:
        return None
    loops = build_loops_from_segments(segments, plane)
    return find_outer_perimeter_loop(loops)


@dataclass
class SliceSegment:
    """A line segment resulting from intersecting a triangle with a plane.

    Each segment is defined by two distinct points ``p1`` and ``p2``.
    These points lie on the slice plane and are expressed as 3D tuples.
    """

    p1: Tuple[float, float, float]
    p2: Tuple[float, float, float]


def dot(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    """Compute the dot product of two 3D vectors.

    Args:
        a: First vector.
        b: Second vector.

    Returns:
        The scalar dot product ``a·b``.
    """
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def sub(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Subtract two 3D vectors (a - b)."""
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Add two 3D vectors."""
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(a: Tuple[float, float, float], s: float) -> Tuple[float, float, float]:
    """Scale a 3D vector by ``s``."""
    return (a[0] * s, a[1] * s, a[2] * s)


def signed_distance_to_plane(p: Tuple[float, float, float], plane: SlicePlane) -> float:
    """Compute the signed distance from a point to a plane.

    The distance is positive on the side of the plane pointed to by
    the normal and negative on the opposite side.  A value near zero
    (within a small epsilon) indicates that the point lies very close
    to the plane.

    Args:
        p: The point whose distance to the plane is computed.
        plane: The target slice plane.

    Returns:
        The signed distance from ``p`` to ``plane``.
    """
    # Vector from origin on the plane to p
    v = sub(p, plane.origin)
    return dot(v, plane.normal)


def intersect_triangle_with_plane(
    A: Tuple[float, float, float],
    B: Tuple[float, float, float],
    C: Tuple[float, float, float],
    plane: SlicePlane,
    eps: float = 1e-6,
) -> List[Tuple[float, float, float]]:
    """Intersect a single triangle with a plane.

    Returns zero, one or two intersection points depending on how the
    triangle straddles the plane.  Coplanar or degenerate triangles
    produce an empty list.  Degenerate edges due to numerical issues
    yield at most two distinct points.

    Args:
        A: First vertex of the triangle.
        B: Second vertex of the triangle.
        C: Third vertex of the triangle.
        plane: The slice plane.
        eps: Tolerance for treating distances as zero.

    Returns:
        A list of intersection points as 3D tuples.  The length is
        either 0, 1 or 2.  In degenerate cases where more points are
        found, the two furthest apart are retained.
    """
    # Compute signed distances to plane
    dA = signed_distance_to_plane(A, plane)
    dB = signed_distance_to_plane(B, plane)
    dC = signed_distance_to_plane(C, plane)
    # Check for coplanarity: all vertices lie very close to plane
    if abs(dA) < eps and abs(dB) < eps and abs(dC) < eps:
        # Coplanar triangles are skipped; optionally log in debug mode
        if os.getenv("SLICE_DEBUG"):
            logger.debug(
                "intersect_triangle_with_plane: coplanar triangle encountered at plane_type=%s offset=%s",
                plane.plane_type,
                plane.offset,
            )
        return []
    # If all distances have same sign and magnitude above epsilon → no intersection
    if (dA > eps and dB > eps and dC > eps) or (dA < -eps and dB < -eps and dC < -eps):
        return []
    # Otherwise compute intersection on edges where distances have opposite sign
    points: List[Tuple[float, float, float]] = []
    verts = [A, B, C]
    ds = [dA, dB, dC]
    # Edges: AB, BC, CA (as indices (0,1),(1,2),(2,0))
    edges = [(0, 1), (1, 2), (2, 0)]
    for i, j in edges:
        d_i = ds[i]
        d_j = ds[j]
        # Check if edge crosses plane: signs differ and not both near zero
        if d_i * d_j < -eps * eps:
            P = verts[i]
            Q = verts[j]
            # parameter t for interpolation along PQ from P
            t = d_i / (d_i - d_j)
            # Intersection point
            PQ = sub(Q, P)
            R = add(P, scale(PQ, t))
            points.append(R)
        elif abs(d_i) < eps and abs(d_j) >= eps:
            # Vertex i lies on plane, vertex j off plane → include vertex i
            points.append(verts[i])
        elif abs(d_j) < eps and abs(d_i) >= eps:
            # Vertex j lies on plane
            points.append(verts[j])
    # Remove duplicate points within tolerance
    unique: List[Tuple[float, float, float]] = []
    for p in points:
        found = False
        for q in unique:
            if math.isclose(p[0], q[0], abs_tol=eps) and math.isclose(p[1], q[1], abs_tol=eps) and math.isclose(p[2], q[2], abs_tol=eps):
                found = True
                break
        if not found:
            unique.append(p)
    if len(unique) <= 2:
        return unique
    # More than two points due to numerical degeneracy: pick two furthest apart
    # Compute pairwise distances squared
    max_pair = (unique[0], unique[1])
    max_dist_sq = -1.0
    for i in range(len(unique)):
        for j in range(i + 1, len(unique)):
            p = unique[i]
            q = unique[j]
            dist_sq = (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2
            if dist_sq > max_dist_sq:
                max_dist_sq = dist_sq
                max_pair = (p, q)
    # Log a warning for numerical oddities when debug is enabled
    if os.getenv("SLICE_DEBUG"):
        logger.debug(
            "intersect_triangle_with_plane: degenerate intersection with %d points; using furthest two",
            len(unique),
        )
    return [max_pair[0], max_pair[1]]


def intersect_mesh_with_plane(
    vertices: List[float],
    indices: List[int],
    plane: SlicePlane,
    eps: float = 1e-6,
) -> List[SliceSegment]:
    """Intersect an entire mesh with a plane and collect line segments.

    Iterates over the triangle index buffer, computes intersection
    points for each triangle and emits a list of :class:`SliceSegment`
    objects whenever exactly two intersection points are found.  Basic
    statistics (triangle count, segment count, coplanar count) are
    gathered and logged when ``SLICE_DEBUG`` is enabled.

    Args:
        vertices: Flat list of vertex coordinates (x0, y0, z0, x1, y1, z1, ...).
        indices: Flat list of integer indices; every three entries form
            a triangle.
        plane: The slice plane.
        eps: Tolerance used in distance comparisons.

    Returns:
        A list of :class:`SliceSegment` objects corresponding to the
        intersection segments between the mesh and the plane.
    """
    segs: List[SliceSegment] = []
    total_tris = 0
    coplanar = 0
    # iterate over triangles
    for k in range(0, len(indices), 3):
        if k + 2 >= len(indices):
            break
        i0, i1, i2 = indices[k], indices[k + 1], indices[k + 2]
        total_tris += 1
        # decode vertices
        try:
            A = (vertices[3 * i0], vertices[3 * i0 + 1], vertices[3 * i0 + 2])
            B = (vertices[3 * i1], vertices[3 * i1 + 1], vertices[3 * i1 + 2])
            C = (vertices[3 * i2], vertices[3 * i2 + 1], vertices[3 * i2 + 2])
        except IndexError:
            # Malformed index buffer; skip
            continue
        pts = intersect_triangle_with_plane(A, B, C, plane, eps=eps)
        if not pts:
            # Count coplanar triangles for diagnostics (if distances are near zero)
            # We recheck distances here because intersect_triangle_with_plane uses eps
            dA = signed_distance_to_plane(A, plane)
            dB = signed_distance_to_plane(B, plane)
            dC = signed_distance_to_plane(C, plane)
            if abs(dA) < eps and abs(dB) < eps and abs(dC) < eps:
                coplanar += 1
            continue
        if len(pts) == 1:
            # Single point indicates triangle touches plane at a vertex; skip for segment generation
            continue
        elif len(pts) == 2:
            segs.append(SliceSegment(p1=pts[0], p2=pts[1]))
        else:
            # Should not occur because intersect_triangle_with_plane collapses to 2 when >2
            segs.append(SliceSegment(p1=pts[0], p2=pts[1]))
    # Emit diagnostic logs when debugging
    if os.getenv("SLICE_DEBUG"):
        logger.debug(
            "intersect_mesh_with_plane: inspected=%d triangles, segments=%d, coplanar=%d",
            total_tris,
            len(segs),
            coplanar,
        )
    # Deduplicate segments that share the same endpoints (order-insensitive).
    # This prevents double-counting when both triangles of a quad emit the
    # same intersection edge, as occurs with the unit cube stub mesh.
    unique: List[SliceSegment] = []
    seen = set()
    for seg in segs:
        key = tuple(sorted((seg.p1, seg.p2)))
        if key in seen:
            continue
        seen.add(key)
        unique.append(seg)

    # Merge collinear segments that share an endpoint so the result reflects
    # the geometric intersection edges rather than per-triangle segments.
    def _collinear(a: Tuple[float, float, float], b: Tuple[float, float, float], c: Tuple[float, float, float], eps: float = 1e-9) -> bool:
        ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
        cross = (
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        )
        return abs(cross[0]) <= eps and abs(cross[1]) <= eps and abs(cross[2]) <= eps

    merged = True
    while merged:
        merged = False
        for i in range(len(unique)):
            for j in range(i + 1, len(unique)):
                s1, s2 = unique[i], unique[j]
                shared = None
                if s1.p1 == s2.p1:
                    shared = s1.p1
                    other1, other2 = s1.p2, s2.p2
                elif s1.p1 == s2.p2:
                    shared = s1.p1
                    other1, other2 = s1.p2, s2.p1
                elif s1.p2 == s2.p1:
                    shared = s1.p2
                    other1, other2 = s1.p1, s2.p2
                elif s1.p2 == s2.p2:
                    shared = s1.p2
                    other1, other2 = s1.p1, s2.p1
                if shared is None:
                    continue
                if not _collinear(other1, shared, other2):
                    continue
                # Merge into a single segment spanning the outer endpoints
                new_seg = SliceSegment(p1=other1, p2=other2)
                unique.pop(j)
                unique.pop(i)
                unique.append(new_seg)
                merged = True
                break
            if merged:
                break

    return unique

# -----------------------------------------------------------------------------
# Phase 4: Offset, simplify, spline and lifting helpers
#
# These functions operate purely in 2D (u, v) coordinates associated with a
# particular slice plane.  They provide the ability to expand a polygon
# outward/inward, reduce its vertex count via the Ramer–Douglas–Peucker
# algorithm, optionally fit a simple closed B‑spline and sample it, and
# finally map the resulting 2D points back into 3D coordinates on the
# original slice plane.

def _line_intersection(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    p3: Tuple[float, float],
    p4: Tuple[float, float],
    eps: float = 1e-12,
) -> Optional[Tuple[float, float]]:
    """Compute the intersection point of two lines defined by p1–p2 and p3–p4.

    If the lines are parallel (within ``eps`` tolerance) or coincident, None
    is returned.  Otherwise the precise intersection point is computed via
    determinants.

    Args:
        p1, p2: Endpoints of the first line.
        p3, p4: Endpoints of the second line.
        eps: Tolerance for detecting parallelism.

    Returns:
        The (x, y) intersection point or None if lines do not intersect.
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < eps:
        return None
    det1 = x1 * y2 - y1 * x2
    det2 = x3 * y4 - y3 * x4
    px = (det1 * (x3 - x4) - (x1 - x2) * det2) / denom
    py = (det1 * (y3 - y4) - (y1 - y2) * det2) / denom
    return (px, py)


def offset_polygon_2d(
    points_uv: List[Tuple[float, float]],
    offset: float,
    area_hint: Optional[float] = None,
) -> List[Tuple[float, float]]:
    """Offset a simple closed polygon outward or inward by a given distance.

    A minimal implementation suitable for outer perimeter loops.  It computes
    outward normals for each edge based on the polygon's winding and offsets
    the edges accordingly.  New vertices are found by intersecting
    consecutive offset edges.  If the polygon area is zero or fewer than
    three points are provided, the original points are returned.

    Args:
        points_uv: Closed polygon defined by a list of (u, v) points.  The
            points should define a simple, non‑self‑intersecting outer loop.
        offset: Distance to offset; positive values move edges outward,
            negative values move inward.  The direction is adjusted by
            the polygon's area sign (counter‑clockwise area > 0).  For
            example, specifying a positive offset always expands the
            polygon outward.
        area_hint: Optional hint for the polygon area used to determine
            winding; if ``None`` the area is computed directly.

    Returns:
        A list of (u, v) points representing the offset polygon.  The
        returned list has the same length as the input and is closed
        (first and last points implicitly connected).
    """
    n = len(points_uv)
    if n < 3:
        return points_uv
    # Compute area to determine orientation
    area = area_hint if area_hint is not None else _polygon_area_2d(points_uv)
    if abs(area) < 1e-12:
        return points_uv
    # Determine sign: positive area → CCW → outward normal along (dy,-dx)
    sign = 1.0 if area > 0 else -1.0
    # Precompute offset edges
    offset_edges: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
    for i in range(n):
        p0 = points_uv[i]
        p1 = points_uv[(i + 1) % n]
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        length = math.hypot(dx, dy)
        if length == 0.0:
            # Degenerate edge; skip
            offset_edges.append((p0, p0))
            continue
        # Outward normal direction (dy,-dx) scaled by sign
        nx = (dy / length) * sign
        ny = (-dx / length) * sign
        off_p0 = (p0[0] + offset * nx, p0[1] + offset * ny)
        off_p1 = (p1[0] + offset * nx, p1[1] + offset * ny)
        offset_edges.append((off_p0, off_p1))
    # Compute intersections for each vertex
    result: List[Tuple[float, float]] = []
    for i in range(n):
        prev_edge = offset_edges[i - 1]
        curr_edge = offset_edges[i]
        inter = _line_intersection(prev_edge[0], prev_edge[1], curr_edge[0], curr_edge[1])
        if inter is None:
            # If lines are parallel, fallback to start of current offset edge
            inter = curr_edge[0]
        result.append(inter)
    return result


def _rdp_rec(points: List[Tuple[float, float]], first: int, last: int, tol: float, keep: List[bool]) -> None:
    """Recursive helper for Ramer–Douglas–Peucker simplification.

    Marks points to keep in the ``keep`` list.  The algorithm finds the
    point with the maximum perpendicular distance to the line connecting
    ``first`` and ``last`` indices; if that distance exceeds the
    tolerance, the point is kept and recursion continues on the two
    sub‑segments.

    Args:
        points: Polyline points.
        first: Index of the start point.
        last: Index of the end point.
        tol: Tolerance for simplification.
        keep: Mutable list of booleans indicating which points to keep.
    """
    # Find point with maximum distance to segment first–last
    max_dist = 0.0
    index = -1
    x1, y1 = points[first]
    x2, y2 = points[last]
    # Precompute denominator of distance (segment length)
    dx = x2 - x1
    dy = y2 - y1
    denom = math.hypot(dx, dy)
    for i in range(first + 1, last):
        px, py = points[i]
        if denom == 0.0:
            dist = math.hypot(px - x1, py - y1)
        else:
            # Perpendicular distance to infinite line
            dist = abs(dy * px - dx * py + x2 * y1 - y2 * x1) / denom
        if dist > max_dist:
            max_dist = dist
            index = i
    if max_dist > tol and index != -1:
        keep[index] = True
        _rdp_rec(points, first, index, tol, keep)
        _rdp_rec(points, index, last, tol, keep)


def simplify_polyline_rdp(points_uv: List[Tuple[float, float]], tolerance: float) -> List[Tuple[float, float]]:
    """Simplify a closed polyline using the Ramer–Douglas–Peucker algorithm.

    The input polyline is treated as closed: the first and last points
    are considered connected.  The simplification preserves the start
    point and ensures closure of the simplified polyline by repeating the
    first point if necessary.

    Args:
        points_uv: Sequence of (u, v) points defining the polyline.  The
            list should represent a closed loop (first and last points
            implicitly connected).  A duplicate last point is allowed but
            not required.
        tolerance: Maximum allowed deviation; smaller values produce more
            detailed polylines.

    Returns:
        A simplified list of (u, v) points representing the same closed
        polyline.  The first point equals the last.
    """
    n = len(points_uv)
    if n < 3 or tolerance <= 0.0:
        # Ensure closed
        pts = points_uv[:]
        if pts and pts[0] != pts[-1]:
            pts.append(pts[0])
        return pts
    # Remove duplicate closing point if present
    points = points_uv[:]
    if points[0] == points[-1]:
        points = points[:-1]
    m = len(points)
    keep_flags = [False] * m
    keep_flags[0] = True
    keep_flags[-1] = True
    _rdp_rec(points, 0, m - 1, tolerance, keep_flags)
    simplified: List[Tuple[float, float]] = [points[i] for i in range(m) if keep_flags[i]]
    # Ensure closure while handling the degenerate two-point case gracefully.
    if simplified:
        if len(simplified) == 2 and simplified[0] != simplified[1]:
            # When only two points remain but they differ, collapse to a closed
            # loop consisting of the start point repeated.  This matches the
            # expectation of a closed polyline with minimal representation.
            simplified = [simplified[0], simplified[0]]
        elif simplified[0] != simplified[-1]:
            simplified.append(simplified[0])
    return simplified


def choose_simplification_tolerance(points_uv: List[Tuple[float, float]]) -> float:
    """Choose a default RDP tolerance based on the scale of the polyline.

    Computes the axis‑aligned bounding box of the polyline and returns
    1% of the maximum extent.  A minimum value is enforced to avoid
    extremely small tolerances.

    Args:
        points_uv: Polyline points used to compute extents.

    Returns:
        A float tolerance value.
    """
    if not points_uv:
        return 0.0
    us = [p[0] for p in points_uv]
    vs = [p[1] for p in points_uv]
    width = max(us) - min(us)
    height = max(vs) - min(vs)
    max_dim = max(width, height)
    tol = 0.01 * max_dim
    # Enforce a minimum tolerance to avoid degenerate results
    return max(tol, 1e-6)


@dataclass
class BSplinePath2D:
    """Minimal representation of a closed 2D uniform B‑spline path.

    Attributes:
        control_points: The (u, v) control points.  For closed splines the
            first ``degree`` points should be repeated at the end.  These
            points should not include a duplicate last point; closure is
            implicit.
        degree: Degree of the spline (default is 3 for cubic).
        knots: Uniform knot vector suitable for evaluation via De Boor.
    """

    control_points: List[Tuple[float, float]]
    degree: int
    knots: List[float]


def fit_uniform_closed_bspline(points_uv: List[Tuple[float, float]], degree: int = 3) -> BSplinePath2D:
    """Fit a closed uniform B‑spline through a sequence of points.

    This simplified implementation constructs a closed B‑spline by
    repeating the first ``degree`` control points at the end and
    generating a uniform knot vector.  No attempt is made to optimise
    control points; the input points themselves are used.  For Phase 4,
    this minimal approach suffices to enable optional smoothing.

    Args:
        points_uv: Control points defining a closed loop.  The first
            point should equal the last to enforce closure; if not, it
            will be appended.
        degree: Degree of the spline (default 3).

    Returns:
        A :class:`BSplinePath2D` with control points, degree and knot vector.
    """
    # Ensure closure of control points
    pts = points_uv[:]
    if not pts:
        return BSplinePath2D(control_points=[], degree=degree, knots=[])
    if pts[0] != pts[-1]:
        pts.append(pts[0])
    # Remove duplicate final control point for interior representation
    ctrl_pts = pts[:-1]
    n = len(ctrl_pts)
    # For a closed spline, duplicate the first 'degree' control points at end
    extended_ctrl = ctrl_pts + ctrl_pts[:degree]
    # Uniform knot vector: values from 0 to len(extended_ctrl)+degree inclusive
    knots = list(range(len(extended_ctrl) + degree + 1))
    return BSplinePath2D(control_points=extended_ctrl, degree=degree, knots=knots)


def _de_boor(k: int, degree: int, knots: List[float], ctrl: List[Tuple[float, float]], u: float) -> Tuple[float, float]:
    """Evaluate a point on a B‑spline curve using the De Boor algorithm.

    Args:
        k: Index such that ``knots[k] <= u < knots[k+1]``.
        degree: Degree of the spline.
        knots: Knot vector.
        ctrl: Control points (possibly extended for closed spline).
        u: Parameter value.

    Returns:
        A tuple (u, v) representing the point on the curve.
    """
    # Copy relevant control points
    d = [list(ctrl[k - degree + i]) for i in range(degree + 1)]
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            i = k - degree + j
            alpha = 0.0
            denom = knots[i + degree + 1 - r] - knots[i]
            if denom != 0:
                alpha = (u - knots[i]) / denom
            d[j][0] = (1 - alpha) * d[j - 1][0] + alpha * d[j][0]
            d[j][1] = (1 - alpha) * d[j - 1][1] + alpha * d[j][1]
    return (d[degree][0], d[degree][1])


def sample_bspline(path: BSplinePath2D, num_samples: int) -> List[Tuple[float, float]]:
    """Sample a B‑spline path at uniformly spaced parameter values.

    A basic implementation using the De Boor algorithm.  For closed
    splines, the control points and knots are assumed to have been
    extended appropriately.  The parameter domain spans from
    ``degree`` to ``len(control_points)``.

    Args:
        path: The B‑spline path to sample.
        num_samples: Number of sample points desired.

    Returns:
        A list of (u, v) tuples sampled along the curve.
    """
    ctrl = path.control_points
    deg = path.degree
    knots = path.knots
    if not ctrl or num_samples <= 0:
        return []
    # The parameter range for a closed uniform B‑spline: from degree to
    # (len(ctrl) - degree - 1).  Because we duplicated the first 'degree'
    # control points at the end, the effective number of segments is
    # len(ctrl) - degree.
    u_start = deg
    u_end = len(ctrl) - deg
    samples: List[Tuple[float, float]] = []
    for i in range(num_samples):
        u = u_start + (u_end - u_start) * i / num_samples
        # For uniform knots we can compute the span directly as floor(u)
        k = int(u)
        # Clamp k to valid range
        k = max(deg, min(k, len(ctrl) - 1))
        pt = _de_boor(k, deg, knots, ctrl, u)
        samples.append(pt)
    return samples


def lift_uv_to_3d(
    points_uv: Iterable[Tuple[float, float]],
    plane: SlicePlane,
) -> List[Tuple[float, float, float]]:
    """Lift 2D (u, v) points back into 3D coordinates according to a slice plane.

    Args:
        points_uv: Sequence of (u, v) points in the slice plane's UV space.
        plane: The slice plane used to define the mapping.  The offset
            defines the constant coordinate value along the orthogonal axis.

    Returns:
        A list of (x, y, z) points lying on the original 3D slice plane.
    """
    result: List[Tuple[float, float, float]] = []
    for u, v in points_uv:
        if plane.plane_type == "xy":
            result.append((u, v, plane.offset))
        elif plane.plane_type == "xz":
            result.append((u, plane.offset, v))
        else:  # "yz"
            result.append((plane.offset, u, v))
    return result