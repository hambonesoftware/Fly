"""
High‑level operations over model metadata.

This module defines the ``ModelRecord`` SQLModel class and helper
functions to initialise the database and to insert and retrieve
metadata about uploaded models.  A ``ModelRecord`` stores the
identifier assigned to an uploaded model along with a content hash,
original filename, storage path, file size and timestamp.  This
metadata layer lays the groundwork for future phases which will use
hashes for deduplication and caching.
"""

from __future__ import annotations

from datetime import datetime
from typing import Optional, List

from sqlmodel import SQLModel, Field, select

from .db import create_db_and_tables, get_session


class BinaryFileRecord(SQLModel, table=True):
    """Database model representing a unique CAD file on disk.

    A ``BinaryFileRecord`` stores the SHA‑256 hash of the file
    contents and points to the canonical location of the file on
    disk.  Multiple ``ModelRecord`` instances may reference the same
    ``BinaryFileRecord`` when identical files are uploaded.
    """

    id: Optional[int] = Field(default=None, primary_key=True)
    file_hash: str = Field(index=True, unique=True)
    file_path: str
    filesize_bytes: int
    created_at: datetime = Field(default_factory=datetime.utcnow)


class ModelRecord(SQLModel, table=True):
    """Database model representing a user-facing model entry.

    A model references a ``BinaryFileRecord`` via ``binary_file_id`` and
    stores metadata such as the original filename provided by the
    user.  Additional fields track the processing status of the
    associated geometry.
    """

    model_id: str = Field(primary_key=True)
    binary_file_id: int = Field(foreign_key="binaryfilerecord.id")
    file_hash: str
    original_name: str
    file_path: str
    filesize_bytes: int
    created_at: datetime = Field(default_factory=datetime.utcnow)
    # Status of the model: uploaded, preprocessing, ready, failed
    status: str = Field(default="uploaded")
    # Optional error message if processing failed
    error_message: Optional[str] = None


class MeshCacheRecord(SQLModel, table=True):
    """Database model representing a cached mesh for a binary file.

    Each cache record corresponds to a particular binary file and
    level of detail (LOD).  The mesh is stored on disk as a
    compressed ``.npz`` file; this record tracks where it is located
    and summary statistics such as vertex and triangle counts and
    bounding box extremes.
    """

    id: Optional[int] = Field(default=None, primary_key=True)
    binary_file_id: int = Field(foreign_key="binaryfilerecord.id")
    lod: int = 0
    mesh_path: str
    vertex_count: int
    triangle_count: int
    bbox_min_x: float
    bbox_min_y: float
    bbox_min_z: float
    bbox_max_x: float
    bbox_max_y: float
    bbox_max_z: float
    created_at: datetime = Field(default_factory=datetime.utcnow)


def init_db() -> None:
    """Initialise the database and create tables if they do not exist.

    This function should be called on application startup.  It
    delegates table creation to the underlying DB module.
    """
    create_db_and_tables()


def insert_model_record(record: ModelRecord) -> None:
    """Insert a new ``ModelRecord`` into the database.

    Args:
        record: The ``ModelRecord`` instance to persist.
    """
    with get_session() as session:
        session.add(record)
        session.commit()


def get_model_record(model_id: str) -> Optional[ModelRecord]:
    """Retrieve a ``ModelRecord`` by its model identifier.

    Args:
        model_id: The identifier of the model to fetch.

    Returns:
        The matching ``ModelRecord`` if found, otherwise ``None``.
    """
    with get_session() as session:
        return session.get(ModelRecord, model_id)


def list_models() -> List[ModelRecord]:
    """Return all model records in the database."""
    with get_session() as session:
        statement = select(ModelRecord)
        return list(session.exec(statement))


def get_binary_file_by_hash(file_hash: str) -> Optional[BinaryFileRecord]:
    """Retrieve a ``BinaryFileRecord`` by its file hash."""
    with get_session() as session:
        statement = select(BinaryFileRecord).where(BinaryFileRecord.file_hash == file_hash)
        return session.exec(statement).first()


def get_binary_file_by_id(binary_file_id: int) -> Optional[BinaryFileRecord]:
    """Retrieve a ``BinaryFileRecord`` by its primary key."""
    with get_session() as session:
        return session.get(BinaryFileRecord, binary_file_id)


def create_binary_file(file_hash: str, file_path: str, filesize_bytes: int) -> BinaryFileRecord:
    """Create and persist a new ``BinaryFileRecord``.

    Args:
        file_hash: SHA‑256 hash of the file contents.
        file_path: Absolute path to the file on disk.
        filesize_bytes: Size of the file in bytes.

    Returns:
        The persisted ``BinaryFileRecord`` instance.
    """
    record = BinaryFileRecord(
        file_hash=file_hash,
        file_path=file_path,
        filesize_bytes=filesize_bytes,
    )
    with get_session() as session:
        session.add(record)
        session.commit()
        session.refresh(record)
        return record


def update_models_status_for_binary(binary_file_id: int, status: str, error_message: Optional[str] = None) -> None:
    """Update the status of all models referencing a given binary.

    Args:
        binary_file_id: The ID of the ``BinaryFileRecord`` to update models for.
        status: New status value (e.g. 'ready', 'failed').
        error_message: Optional error message to set on models.
    """
    with get_session() as session:
        stmt = select(ModelRecord).where(ModelRecord.binary_file_id == binary_file_id)
        models = session.exec(stmt).all()
        for m in models:
            m.status = status
            m.error_message = error_message
            session.add(m)
        session.commit()


def delete_model(model_id: str) -> None:
    """Delete a model and optionally clean up orphaned binary and caches.

    If, after deletion, no other models reference the same binary
    file, this function leaves cleanup to the caller.  Deleting
    binary files and cache files can be implemented externally.

    Args:
        model_id: Identifier of the model to delete.
    """
    with get_session() as session:
        model = session.get(ModelRecord, model_id)
        if model is None:
            return
        binary_file_id = model.binary_file_id
        session.delete(model)
        session.commit()
        # Check if any other models reference this binary
        stmt = select(ModelRecord).where(ModelRecord.binary_file_id == binary_file_id)
        remaining = session.exec(stmt).first()
        if remaining is None:
            # Orphaned binary can be cleaned up by caller
            pass


def get_mesh_cache_for_binary(binary_file_id: int, lod: int = 0) -> Optional[MeshCacheRecord]:
    """Retrieve a mesh cache record for a given binary file and LOD."""
    with get_session() as session:
        statement = select(MeshCacheRecord).where(
            MeshCacheRecord.binary_file_id == binary_file_id,
            MeshCacheRecord.lod == lod,
        )
        return session.exec(statement).first()


def upsert_mesh_cache_for_binary(record: MeshCacheRecord) -> None:
    """Insert or replace a mesh cache record for a binary.

    If a record already exists for the given binary and LOD, it will be
    replaced.
    """
    with get_session() as session:
        stmt = select(MeshCacheRecord).where(
            MeshCacheRecord.binary_file_id == record.binary_file_id,
            MeshCacheRecord.lod == record.lod,
        )
        existing = session.exec(stmt).first()
        if existing is not None:
            session.delete(existing)
            session.commit()
        session.add(record)
        session.commit()


def get_mesh_cache(file_hash: str, lod: int = 0) -> Optional[MeshCacheRecord]:
    """Backward‑compatible helper to retrieve a cache by file hash.

    This helper remains for compatibility with older phases.  It
    searches for any ``BinaryFileRecord`` matching the given file hash
    and returns the first matching ``MeshCacheRecord``.  Prefer using
    ``get_mesh_cache_for_binary`` in new code.
    """
    # Find the binary
    binary = get_binary_file_by_hash(file_hash)
    if binary is None:
        return None
    return get_mesh_cache_for_binary(binary.id, lod)


def upsert_mesh_cache(record: MeshCacheRecord) -> None:
    """Backward‑compatible helper to insert or replace a cache by file hash.

    This function wraps ``upsert_mesh_cache_for_binary`` for older
    interfaces which set ``record.binary_file_id`` based on the file
    hash.
    """
    """Backward‑compatible helper to insert or replace a cache record.

    This function simply forwards to ``upsert_mesh_cache_for_binary``.
    It ignores any file hash information on the record and relies on
    the ``binary_file_id`` being set on the record instance.
    """
    upsert_mesh_cache_for_binary(record)