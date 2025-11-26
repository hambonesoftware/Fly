"""
API routes for perimeter extraction.

This module defines an endpoint to compute the mesh‑based perimeter loops
for a given model.  The perimeter finder projects the tessellated mesh
onto one of the principal planes (XY, XZ, YZ), stitches boundary edges
into loops, discards interior polygons and returns the outermost
footprints.  Clients can use this endpoint to visualise the model
perimeter before generating a flythrough path.

The endpoint returns a JSON object containing the plane, a list of
perimeter loops with their areas and a metadata section with summary
information such as the stitch tolerance and island mode.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, HTTPException, Query

from ..services.geometry import compute_raster_perimeter_polygons
from ..services.geometry import get_mesh_for_model

# Import area helper from slicing for computing loop areas
try:
    from ..services.slicing import _polygon_area_2d  # type: ignore
except Exception:
    _polygon_area_2d = None  # type: ignore

logger = logging.getLogger(__name__)

router = APIRouter()


@router.get(
    "/models/{model_id}/perimeter",
    response_model=None,
)
async def get_perimeter(
    model_id: str,
    plane: str = Query("xy", description="Projection plane: xy, xz or yz"),
    outlineStitchPercent: Optional[float] = Query(
        None,
        description="Relative stitch percentage controlling vertex snapping tolerance (0–100).",
    ),
    perimeterIslandMode: str = Query(
        "single",
        description="Island mode: 'single' to return only the largest island, 'multi' to return all outer islands.",
        regex="^(single|multi)$",
    ),
    perimeterDetail: Optional[str] = Query(
        None,
        description="Detail level controlling raster resolution: 'low', 'medium', 'high' or 'auto'.",
    ),
) -> Dict[str, Any]:
    """
    Compute perimeter loops for a model by projecting its mesh onto a plane.

    This endpoint exposes the mesh‑based perimeter finder.  It collapses the
    tessellated mesh along the requested plane, stitches boundary edges into
    closed loops and discards any polygons contained wholly within a larger
    polygon.  Only the outer islands are returned according to the requested
    island mode.

    The query parameters allow clients to control the projection plane, the
    relative stitch tolerance and whether a single island or multiple
    islands should be returned.  An optional detail parameter is
    accepted for forward compatibility but currently has no effect.

    Returns:
        A dictionary with the projection plane, an array of loops and a
        metadata object summarising the computation.
    """
    pl = (plane or "xy").strip().lower()
    # Validate plane
    if pl not in {"xy", "xz", "yz"}:
        raise HTTPException(status_code=400, detail=f"Invalid plane '{plane}'. Must be one of xy, xz or yz.")
    # Determine island mode
    island_mode = (perimeterIslandMode or "single").strip().lower()
    if island_mode not in {"single", "multi"}:
        raise HTTPException(status_code=400, detail=f"Invalid island mode '{perimeterIslandMode}'.")
    # Compute perimeter loops via raster engine
    try:
        loops3d, areas, meta_raster = compute_raster_perimeter_polygons(
            model_id=model_id,
            plane=pl,
            detail=perimeterDetail,
            island_mode=island_mode,
            outline_stitch_percent=outlineStitchPercent,
        )
    except Exception as exc:
        logger.exception(
            "perimeter endpoint error for model_id=%s: %s", model_id, exc
        )
        raise HTTPException(status_code=500, detail=f"Failed to compute perimeter: {exc}")
    # If no loops returned, respond accordingly
    if not loops3d:
        return {
            "plane": pl,
            "loops": [],
            "meta": {
                "stitchTolerance": meta_raster.get('stitchTolerance'),
                "cellSize": meta_raster.get('cellSize'),
                "gridWidth": meta_raster.get('gridWidth'),
                "gridHeight": meta_raster.get('gridHeight'),
                "islandMode": island_mode,
                "totalLoops": 0,
                "totalPoints": 0,
            },
        }
    # Build loops section with area
    loops_output: List[Dict[str, Any]] = []
    total_points = 0
    for idx, pts3d in enumerate(loops3d):
        points_array = [[float(p[0]), float(p[1]), float(p[2])] for p in pts3d]
        total_points += len(points_array)
        loops_output.append({
            "index": idx,
            "points": points_array,
            "area": areas[idx] if idx < len(areas) else None,
        })
    meta = {
        "stitchTolerance": meta_raster.get('stitchTolerance'),
        "cellSize": meta_raster.get('cellSize'),
        "gridWidth": meta_raster.get('gridWidth'),
        "gridHeight": meta_raster.get('gridHeight'),
        "islandMode": island_mode,
        "totalLoops": len(loops_output),
        "totalPoints": total_points,
        "timings": meta_raster.get('timings'),
    }
    return {
        "plane": pl,
        "loops": loops_output,
        "meta": meta,
    }