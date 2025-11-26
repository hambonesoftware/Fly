"""
Local storage service for uploaded CAD models.

In Phase 1 this module provides a simple mechanism to persist uploaded
IGES/STEP files on disk and retrieve them later by identifier.  Future
phases may expand this to use more robust storage backends.
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)

import os
import uuid
from pathlib import Path
from typing import Optional

from fastapi import UploadFile, HTTPException

from ..api.models import ModelInfo

# Import metadata storage helpers.  These imports are placed here to
# avoid circular dependencies and to defer database creation until
# actually needed.
# Import metadata storage helpers.  These imports are placed here to
# avoid circular dependencies and to defer database creation until
# actually needed.
from .models_store import (
    ModelRecord,
    BinaryFileRecord,
    insert_model_record,
    get_binary_file_by_hash,
    create_binary_file,
    get_mesh_cache_for_binary,
)
import hashlib


# Base directory for storing uploaded models.  Files are organised as
# backend/storage/models/{modelId}/original.<ext>.  The parent
# directories are created on demand.
BASE_DIR = Path(__file__).resolve().parents[3]
STORAGE_MODELS_DIR = BASE_DIR / "storage" / "models"
STORAGE_MODELS_DIR.mkdir(parents=True, exist_ok=True)

# Directory for canonical binary files keyed by hash.  Files are stored
# in ``storage/binary/{file_hash}{ext}`` and reused across multiple
# models with the same content.
STORAGE_BINARY_DIR = BASE_DIR / "storage" / "binary"
STORAGE_BINARY_DIR.mkdir(parents=True, exist_ok=True)

# Temporary directory for streaming uploads while computing hashes.
STORAGE_TEMP_DIR = BASE_DIR / "storage" / "tmp"
STORAGE_TEMP_DIR.mkdir(parents=True, exist_ok=True)


def save_model_file(upload_file: UploadFile) -> ModelInfo:
    """Persist an uploaded model file to disk and return its metadata.

    This implementation deduplicates uploads based on their SHA‑256
    hash.  Files are streamed into a temporary location while
    computing their hash.  If a binary with the same hash already
    exists, the temporary file is discarded and the existing binary
    file reused.  Otherwise the file is moved into the canonical
    ``storage/binary`` directory.  A new ``ModelRecord`` referencing
    the appropriate ``BinaryFileRecord`` is then created.

    Args:
        upload_file: Incoming file from the client.

    Returns:
        ModelInfo: Metadata describing the stored file.
    """
    # Log the upload for debugging
    logger.info("Saving uploaded model %s", getattr(upload_file, "filename", "<unknown>"))
    # Generate a unique identifier for the model
    model_id = uuid.uuid4().hex
    # Determine the file extension (including dot if present)
    _, ext = os.path.splitext(upload_file.filename)
    ext = ext or ""
    # Stream the upload into a temporary file while computing its hash
    sha256 = hashlib.sha256()
    temp_path = STORAGE_TEMP_DIR / f"tmp_{model_id}"
    with temp_path.open("wb") as tmp_file:
        while True:
            chunk = upload_file.file.read(8192)
            if not chunk:
                break
            tmp_file.write(chunk)
            sha256.update(chunk)
    file_hash = sha256.hexdigest()
    filesize_bytes = temp_path.stat().st_size
    # Determine the canonical binary path
    canonical_path = STORAGE_BINARY_DIR / f"{file_hash}{ext}"
    # Attempt to fetch existing binary record
    binary = get_binary_file_by_hash(file_hash)
    if binary is None:
        # No existing binary; move temp file into canonical location
        if not canonical_path.exists():
            canonical_path.parent.mkdir(parents=True, exist_ok=True)
            temp_path.replace(canonical_path)
        else:
            # Rare race: file already exists on disk; remove temp
            temp_path.unlink(missing_ok=True)
        # Create a new BinaryFileRecord
        binary = create_binary_file(file_hash, str(canonical_path), filesize_bytes)
    else:
        # Existing binary found; remove the temp file and update canonical_path
        temp_path.unlink(missing_ok=True)
        canonical_path = Path(binary.file_path)
    # Determine initial status: ready if a mesh cache exists, otherwise uploaded
    cache = get_mesh_cache_for_binary(binary.id, lod=0)
    # Determine initial status: if a mesh cache already exists the model is ready;
    # otherwise mark it as preprocessing to indicate that a background job will run.
    status = "ready" if cache is not None else "preprocessing"
    # Create the model record referencing the binary
    record = ModelRecord(
        model_id=model_id,
        binary_file_id=binary.id,
        file_hash=file_hash,
        original_name=upload_file.filename or "",
        file_path=str(canonical_path),
        filesize_bytes=filesize_bytes,
        status=status,
    )
    insert_model_record(record)
    # Return minimal info (legacy client expects status 'stored')
    return ModelInfo(modelId=model_id, filename=upload_file.filename, status="stored")


def get_model_file_path(model_id: str) -> Path:
    """Return the canonical file path for a stored model.

    In Phase 3 models no longer store their own copy of the uploaded
    file; instead the path is recorded in the database.  This
    function first looks up the ``ModelRecord`` and returns its
    ``file_path``.  If the record is missing, it falls back to the
    legacy per-model directory.

    Args:
        model_id: Identifier of the model whose file path is needed.

    Returns:
        pathlib.Path: Path to the stored file on disk.

    Raises:
        HTTPException: If the model or file cannot be found.
    """
    # Attempt to retrieve from the database
    from .models_store import get_model_record  # local import to avoid circular import issues
    record = get_model_record(model_id)
    if record is not None:
        return Path(record.file_path)
    # Fall back to scanning the legacy per-model directory for backward compatibility
    model_dir = STORAGE_MODELS_DIR / model_id
    logger.debug("Legacy model directory for id %s resolved to %s", model_id, model_dir)
    if model_dir.exists() and model_dir.is_dir():
        for file_path in model_dir.iterdir():
            if file_path.is_file() and file_path.name.startswith("original"):
                return file_path
    # Not found
    raise HTTPException(status_code=404, detail="Model file not found")