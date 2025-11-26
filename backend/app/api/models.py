"""
Pydantic data models for the flythrough planner API.

These models define the shapes of requests and responses used by the
backend.  Maintaining these schemas separately helps enforce the API
contract defined in Phase 0 and makes it easy to adjust fields as the
service evolves.
"""

from __future__ import annotations

from typing import List, Dict, Any
from pydantic import BaseModel, Field
from typing import Literal


class ModelInfo(BaseModel):
    """Metadata returned after a model file is uploaded."""

    modelId: str = Field(..., description="Unique identifier for the uploaded model")
    filename: str = Field(..., description="Original filename provided by the client")
    status: str = Field(..., description="Current storage status of the model")


class MeshBBox(BaseModel):
    """Axis‑aligned bounding box for a mesh."""

    min: List[float] = Field(..., description="Minimum x, y, z coordinates of the mesh")
    max: List[float] = Field(..., description="Maximum x, y, z coordinates of the mesh")


class MeshResponse(BaseModel):
    """Response returned for a mesh request."""

    modelId: str = Field(..., description="Identifier of the associated model")
    vertices: List[float] = Field(..., description="Flat list of vertex positions (x, y, z …)")
    indices: List[int] = Field(..., description="Index buffer defining the mesh triangles")
    normals: List[float] = Field(..., description="Flat list of vertex normals (x, y, z …)")
    bbox: MeshBBox = Field(..., description="Bounding box around the mesh")


# New data model for listing model status and metadata
class ModelStatusInfo(BaseModel):
    """Summary information about a stored model, including its processing status.

    This schema is used by the new CRUD endpoints added in Phase 3 to
    surface a list of models and their current processing state.  It
    includes minimal identifying and status fields.
    """

    modelId: str = Field(..., description="Unique identifier for the model")
    name: str = Field(..., description="Original filename provided by the user")
    createdAt: Any = Field(..., description="Timestamp of when the model was uploaded")
    status: str = Field(..., description="Processing status of the model (uploaded, preprocessing, ready, failed)")
    errorMessage: str | None = Field(
        default=None, description="Optional error message if preprocessing failed"
    )


class PathCreateRequest(BaseModel):
    """Request body for creating a new path."""

    cameraRadius: float = Field(..., description="Radius of the virtual camera or clearance")
    samplingResolution: List[int] = Field(
        ..., description="Resolution of the sampling grid used by the planner"
    )
    strategy: str = Field(..., description="Planning strategy to use (e.g. 'orbit')")
    # Optional tuning parameters for the perimeter path generator (Phase 4).
    # bulgeFactor controls how far selected edges of the perimeter may bulge
    # outward beyond the base offset.  A value of 0.0 (default) disables bulging.
    bulgeFactor: float = Field(
        default=0.0,
        description="Additional outward bulge applied to selected perimeter edges (0.0–1.0)",
    )
    # smoothness controls how aggressively the path is smoothed using a spline.
    # 0.0 yields the original straight‑edge path; 1.0 produces a fully
    # Catmull–Rom resampled path at a higher resolution.
    smoothness: float = Field(
        default=0.5,
        description="Smoothness of the perimeter path interpolation (0.0–1.0)",
    )

    # New in Phase 5: perimeter detail level.  Controls how finely the
    # slice‑based perimeter is simplified and sampled.  Accepted values
    # are "low", "medium", "high" or "auto".  When unspecified
    # (None) the planner defaults to "auto" which selects a medium
    # detail appropriate for the model size.  Invalid values are
    # treated as "auto".
    perimeterDetail: str | None = Field(
        default=None,
        description="Requested level of detail for the perimeter path ('low', 'medium', 'high', 'auto')",
    )

    # New in Phase 5: override to toggle spline fitting for the
    # perimeter path.  When True the simplified perimeter is fitted
    # with a closed B‑spline; when False the raw simplified polygon
    # points are used directly.  If unspecified (None) the existing
    # smoothness parameter controls spline usage as in earlier phases.
    useSpline: bool | None = Field(
        default=None,
        description="Whether to fit a spline through the simplified perimeter points (default determined by smoothness)",
    )

    # New parameters for Phase 6: plane selection and height offset.  When
    # generating a perimeter path the planner slices the model at a
    # particular plane.  The `plane` field selects one of the three
    # orthogonal planes defined by the model’s axes: 'xy' produces a
    # horizontal slice (constant z), 'xz' a vertical slice in the
    # y‑direction (constant y) and 'yz' a vertical slice in the x‑direction
    # (constant x).  The `heightOffset` sets the constant coordinate of
    # that plane relative to the global origin.  Positive or negative
    # values can be used to sample cross‑sections above or below the
    # model.
    plane: str = Field(
        default="xy",
        description="Plane on which to compute the perimeter ('xy', 'xz' or 'yz')",
    )
    heightOffset: float = Field(
        default=0.0,
        description="Offset along the selected axis where the perimeter plane is located",
    )
    useModelOutline: bool = Field(
        default=False,
        description=(
            "When true and strategy='perimeter', use the model's projected "
            "outline (HLR) as the starting curve instead of a mesh slice."
        ),
    )

    outlineStitchTolerance: float | None = Field(
        default=None,
        description="Optional override for HLR outline stitching tolerance (model units).",
    )
    outlineDetail: str | None = Field(
        default=None,
        description=(
            "Detail level for HLR outline sampling ('low', 'medium', 'high', 'auto'). "
            "When omitted, 'auto' is used."
        ),
    )

    # New in version 0.9.7: relative stitching for outlines.  When set,
    # specifies the tolerance as a percentage of the model's characteristic
    # length rather than an absolute model unit.  Values outside the range
    # [0, 100] are clamped by the backend.  When omitted, a default of 2%
    # is used.
    outlineStitchPercent: float | None = Field(
        default=None,
        ge=0.0,
        le=100.0,
        description=(
            "Stitch tolerance as a percentage of the model's characteristic length "
            "used when generating projected outlines. 0% → minimal stitching, 100% → "
            "aggressive stitching (tends toward a single silhouette)."
        ),
    )

    # New in version 0.9.7: island mode for perimeter paths.  When "multi"
    # the planner returns a separate path for each disconnected island in the
    # outline; when "single" the islands are combined into a single path by
    # choosing the largest loop.  The default is "single" for backwards
    # compatibility.
    perimeterIslandMode: Literal["single", "multi"] = Field(
        default="single",
        description=(
            "Whether to generate a single perimeter path (from the union of all outlines) "
            "or multiple paths, one per island."
        ),
    )

    # New in version 0.9.11: options for rolling‑circle perimeter paths.  When
    # computing a perimeter path using the model outline, these fields control
    # whether a rolling‑circle locus should be generated instead of the
    # standard offset path.  The rolling‑circle path traces the offset of
    # the perimeter by the camera circle radius plus the base clearance.  See
    # the backend service documentation for details.
    perimeterUseRollingCircle: bool = Field(
        default=False,
        description=(
            "When true and strategy='perimeter' and useModelOutline is true, "
            "generate a rolling‑circle perimeter path instead of the standard offset path."
        ),
    )
    perimeterCircleDiameter: float | None = Field(
        default=None,
        description=(
            "Diameter of the camera circle used for rolling‑circle perimeter paths.  "
            "If unspecified or <= 0, a default based on the model size is chosen.  "
            "The effective circle radius is half of this value."
        ),
    )




class PathPoint(BaseModel):
    """Single 3D point along a path."""

    x: float
    y: float
    z: float


class PathResponse(BaseModel):
    """Response returned after a path is created."""

    pathId: str = Field(..., description="Unique identifier for the generated path")
    modelId: str = Field(..., description="Identifier of the associated model")
    points: List[PathPoint] = Field(..., description="Ordered list of points along the path")
    metadata: Dict[str, Any] = Field(
        ..., description="Additional metadata such as length and strategy"
    )


class PathValidateRequest(BaseModel):
    """Request body for validating a user‑edited path."""

    points: List[PathPoint] = Field(..., description="Points defining the path to validate")
    cameraRadius: float = Field(..., description="Camera radius used to compute clearance")


class PathValidateResponse(BaseModel):
    """Response returned after validating a path."""

    valid: bool = Field(..., description="Whether the path is considered valid")
    minClearance: float = Field(
        ..., description="Minimum clearance between the path and the model surface"
    )
    violations: List[Dict[str, Any]] = Field(
        default_factory=list,
        description="List of violation details for points that do not meet clearance",
    )

# New in Phase 9.6.4: response model for perimeter outlines.
class OutlineResponse(BaseModel):
    """Response returned when requesting a model's perimeter outline.

    This schema describes the base outline extracted from a model on a
    particular plane.  The returned points always lie on the selected
    plane (after applying any height offset) and form a closed loop.
    Additional metadata indicates how the outline was obtained (slice,
    outline or bbox fallback) and any tuning parameters used.
    """

    modelId: str = Field(..., description="Identifier of the associated model")
    points: List[PathPoint] = Field(
        ..., description="Ordered list of points defining the base outline"
    )
    metadata: Dict[str, Any] | None = Field(
        default=None,
        description="Additional metadata about the outline such as the source and sampling parameters",
    )