"""
Tests for path creation, validation and export endpoints.

These tests ensure that the stub path endpoints behave as expected.
The path creation should return a path identifier and a non‑empty list
of points.  Validation should always be valid in this phase.  Export
should return CSV data.
"""

import io
import sys
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

sys.path.append(str(Path(__file__).resolve().parents[1]))
from app.main import app  # type: ignore


@pytest.fixture
def client() -> TestClient:
    return TestClient(app)


def _upload_dummy_model(client: TestClient) -> str:
    """Helper to upload a dummy model and return its ID."""
    response = client.post(
        "/api/models",
        files={"file": ("dummy.igs", io.BytesIO(b"dummy"), "application/octet-stream")},
    )
    return response.json()["modelId"]


def test_create_and_validate_path(client: TestClient) -> None:
    """Creating a path and validating it should succeed with stub data."""
    model_id = _upload_dummy_model(client)
    # Create a path with arbitrary parameters (ignored in Phase 1)
    body = {
        "cameraRadius": 0.1,
        "samplingResolution": [40, 40, 40],
        "strategy": "orbit",
    }
    resp = client.post(f"/api/models/{model_id}/paths", json=body)
    assert resp.status_code in (200, 201)
    data = resp.json()
    assert "pathId" in data
    assert len(data.get("points", [])) > 0
    path_id = data["pathId"]
    # Validate the path.  Use the same points that were returned.
    validate_body = {"points": data["points"], "cameraRadius": 0.1}
    val_resp = client.post(
        f"/api/models/{model_id}/paths/{path_id}/validate", json=validate_body
    )
    assert val_resp.status_code == 200
    val_data = val_resp.json()
    assert val_data.get("valid") is True
    # Export the path as CSV and check content
    csv_resp = client.get(f"/api/models/{model_id}/paths/{path_id}/export")
    assert csv_resp.status_code == 200
    # The CSV should start with a header and have at least one data line
    lines = csv_resp.text.strip().split("\n")
    assert lines[0] == "x,y,z"
    assert len(lines) > 1