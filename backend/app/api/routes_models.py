"""
Routes for model upload and mesh retrieval.

This router exposes endpoints to upload a CAD model file and to
retrieve a triangulated mesh representation.  In Phase 1 the mesh is
a hard‑coded cube; later phases will load and tessellate the actual
uploaded model.
"""

from __future__ import annotations

from fastapi import APIRouter, UploadFile, File, BackgroundTasks

from .models import ModelInfo, MeshResponse, ModelStatusInfo
from ..services.storage import save_model_file
from ..services.geometry import get_mesh_for_model, precompute_mesh_for_model
from ..services.models_store import (
    list_models as list_model_records,
    get_model_record,
    delete_model as delete_model_record,
)


router = APIRouter()


@router.post("/models", response_model=ModelInfo, status_code=201)
async def upload_model(
    file: UploadFile = File(...),
    background_tasks: BackgroundTasks = None,
) -> ModelInfo:
    """Upload an IGES or STEP model file and trigger mesh precomputation.

    The file is saved to disk and a unique model identifier is
    generated.  After saving, a background task is enqueued to
    precompute a coarse mesh for the uploaded model.  This ensures
    that subsequent mesh requests can be served quickly from the
    cache.  No changes are made to the response schema.
    """
    model_info = save_model_file(file)
    # Schedule asynchronous mesh precomputation using FastAPI's background tasks.
    # If background_tasks is None (unlikely), precomputation is skipped.
    if background_tasks is not None:
        background_tasks.add_task(precompute_mesh_for_model, model_info.modelId, 0)
    return model_info


@router.get("/models/{model_id}/mesh", response_model=MeshResponse)
async def get_mesh(model_id: str) -> MeshResponse:
    """Retrieve a stub mesh for the given model.

    Args:
        model_id: Identifier returned when the model was uploaded.

    Returns:
        MeshResponse: A cube mesh as a placeholder until geometry is
        implemented in Phase 3.
    """
    return get_mesh_for_model(model_id)


# New CRUD and status endpoints for Phase 3

@router.get("/models", response_model=list[ModelStatusInfo])
async def list_models() -> list[ModelStatusInfo]:
    """Return a list of all models with their status.

    Returns:
        A list of ModelStatusInfo entries for each stored model.
    """
    records = list_model_records()
    # Map database records to API response objects
    result: list[ModelStatusInfo] = []
    for r in records:
        result.append(
            ModelStatusInfo(
                modelId=r.model_id,
                name=r.original_name,
                createdAt=r.created_at,
                status=r.status,
                errorMessage=r.error_message,
            )
        )
    return result


@router.get("/models/{model_id}", response_model=ModelStatusInfo)
async def get_model(model_id: str) -> ModelStatusInfo:
    """Return detailed status information for a single model.

    Args:
        model_id: Identifier of the model to retrieve.

    Returns:
        A ModelStatusInfo describing the model.

    Raises:
        HTTPException: If the model does not exist.
    """
    record = get_model_record(model_id)
    if record is None:
        from fastapi import HTTPException

        raise HTTPException(status_code=404, detail="Model not found")
    return ModelStatusInfo(
        modelId=record.model_id,
        name=record.original_name,
        createdAt=record.created_at,
        status=record.status,
        errorMessage=record.error_message,
    )


@router.delete("/models/{model_id}", status_code=204)
async def delete_model(model_id: str) -> None:
    """Delete a model and, if appropriate, its underlying data.

    Args:
        model_id: Identifier of the model to delete.

    Side effects:
        Removes the ModelRecord from the database.  Underlying binary
        files and mesh caches are not deleted in this minimal
        implementation.  A more sophisticated cleanup could remove
        these if they are no longer referenced.
    """
    delete_model_record(model_id)
    # Per FastAPI semantics, returning None with status 204 yields an empty response
    return None