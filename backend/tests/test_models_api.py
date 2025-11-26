"""
Tests for model upload and mesh retrieval endpoints.

These tests use FastAPI's TestClient to simulate requests against the
application without running a real server.  They verify that
uploading a file returns a model identifier and that requesting the
mesh returns the expected stub data.
"""

import io
import sys
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

# Add the backend directory to sys.path so we can import the app
sys.path.append(str(Path(__file__).resolve().parents[1]))

from app.main import app  # type: ignore


@pytest.fixture
def client() -> TestClient:
    return TestClient(app)


def test_upload_and_get_mesh(client: TestClient) -> None:
    """Uploading a file should return a modelId and mesh retrieval should succeed."""
    # Upload a dummy file.  We use BytesIO to simulate an uploaded file.
    file_content = b"solid dummy IGES content"
    response = client.post(
        "/api/models",
        files={"file": ("dummy.igs", io.BytesIO(file_content), "application/octet-stream")},
    )
    # The API may return 200 or 201 based on FastAPI defaults; accept either.
    assert response.status_code in (200, 201)
    data = response.json()
    assert "modelId" in data
    model_id = data["modelId"]
    # Retrieve the stub mesh
    mesh_resp = client.get(f"/api/models/{model_id}/mesh")
    assert mesh_resp.status_code == 200
    mesh = mesh_resp.json()
    assert mesh["modelId"] == model_id
    # Ensure the cube has vertices and indices
    assert len(mesh.get("vertices", [])) > 0
    assert len(mesh.get("indices", [])) > 0