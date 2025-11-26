"""
Database configuration and session management for the flythrough backend.

This module defines a SQLModel engine targeting a SQLite database stored
in the project's ``storage`` directory.  It exposes helper functions to
initialise the schema and to obtain session objects for interacting
with the database.  Creating this layer centrally keeps database
configuration isolated from business logic in other modules.
"""

from __future__ import annotations

from pathlib import Path
from sqlmodel import SQLModel, create_engine, Session

# Determine the base directory for storage.  We walk up two parent
# directories from this file to locate the project root, then append
# ``storage``.  If the storage directory doesn't exist, create it.
STORAGE_DIR = Path(__file__).resolve().parents[2] / "storage"
STORAGE_DIR.mkdir(parents=True, exist_ok=True)

# Path to the SQLite database file.  Using as_posix() ensures the
# resulting URL is valid on all platforms.  We avoid echoing SQL
# statements for cleanliness.
engine = create_engine(
    f"sqlite:///{(STORAGE_DIR / 'fly.db').as_posix()}", echo=False
)

def create_db_and_tables() -> None:
    """Create all tables in the database.

    This should be called once on application startup.  If the
    database file does not exist it will be created automatically.
    """
    SQLModel.metadata.create_all(engine)


def get_session() -> Session:
    """Return a new SQLModel session bound to the engine.

    Use this helper whenever you need to perform database operations.
    Sessions returned by this function should be managed with a
    context manager (``with get_session() as session: ...``) to
    ensure that connections are properly closed.
    """
    return Session(engine)