"""
Main application module for the flythrough backend.

This file sets up the FastAPI application, configures CORS so the
frontend can make cross-origin requests, mounts the static frontend
files when they exist, and exposes a simple health check endpoint.

Routers for the model and path APIs are included under the `/api`
namespace.  Additional routes and services will be added in later
phases.
"""

from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .api.routes_models import router as models_router
from .api.routes_paths import router as paths_router
from .api.routes_perimeter import router as perimeter_router

# Import database initialisation here to avoid circular dependencies.  This
# import is placed at module level so it can be referenced both in
# create_app and in the startup event defined within it.  The init_db
# function creates tables in the SQLite database if they do not
# already exist.
from .services.models_store import init_db  # type: ignore


def create_app() -> FastAPI:
    """Factory to create and configure the FastAPI application.

    Returns:
        FastAPI: The configured FastAPI application instance.
    """
    app = FastAPI()

    # Register a startup event to initialise the database.  This ensures
    # that the SQLite schema is created before any requests are
    # processed.  The init_db function is idempotent and safe to call
    # multiple times.
    @app.on_event("startup")  # type: ignore[misc]
    async def startup_event() -> None:
        init_db()

    # Allow all origins by default.  In production you should restrict
    # this to the domains that are allowed to access your API.
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Health check endpoint for monitoring and deployment probes.
    @app.get("/api/health")
    async def health() -> dict[str, str]:
        return {"status": "ok"}

    # Include API routers under the /api prefix.  Tags are optional but
    # help organise endpoints in the automatically generated docs.
    app.include_router(models_router, prefix="/api", tags=["models"])
    app.include_router(paths_router, prefix="/api", tags=["paths"])
    app.include_router(perimeter_router, prefix="/api", tags=["perimeter"])

    # Mount the frontend as static files if it exists.  This allows
    # serving the compiled HTML/CSS/JS without a separate web server.
    # The frontend directory is located two levels up from this file.
    frontend_dir = Path(__file__).resolve().parents[2] / "frontend"
    if frontend_dir.exists():
        app.mount(
            "/",
            StaticFiles(directory=str(frontend_dir), html=True),
            name="frontend",
        )

    return app


# Create the application instance.  Uvicorn will import this when
# running `uvicorn app.main:app` from within the backend/app
app = create_app()
