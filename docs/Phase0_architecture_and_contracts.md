# Phase 0 – Architecture + API and Data Contracts

## Goal
Define the minimal architecture, directory layout, and JSON contracts so that backend and frontend can be developed in parallel by the agents. No code is written here; only decisions and specs.

This plan assumes that agent files already exist in `agent_fly.zip` (for example: Orchestrator, BackendDev, FrontendDev, DocAgent), and that ChatGPT /agent mode will call them as needed.

---

## 0.1 Directory Layout

Agents must standardize on the following project structure:

```text
fly_project/
  backend/
    app/
      __init__.py
      main.py
      api/
        __init__.py
        models.py       # Pydantic schemas
        routes_models.py
        routes_paths.py
      core/
        __init__.py
        config.py
        logging.py
      services/
        __init__.py
        storage.py      # local file storage of uploaded models
        geometry.py     # IGES/STEP import + meshing
        path_planner.py # path planning algorithms
    tests/
      test_models_api.py
      test_paths_api.py
    pyproject.toml or requirements.txt

  frontend/
    index.html
    css/
      styles.css
    js/
      main.js          # bootstrap, API calls, UI wiring
      apiClient.js     # fetch wrappers
      viewer.js        # Three.js setup + model rendering
      pathOverlay.js   # path drawing + interaction
      ui.js            # DOM interactions, buttons, status

  README.md
```

Agents must adhere to this layout unless explicitly instructed otherwise in a later plan or agent file.

---

## 0.2 REST API Contract (v1)

**Base URL:** `/api`

All JSON structures must be stable so that frontend and backend agents can work independently.

### 1. Upload model

- **Endpoint:** `POST /api/models`
- **Request:** `multipart/form-data`
  - `file`: IGES or STEP model file
- **Response (201 JSON):**
```json
{
  "modelId": "string",
  "filename": "original_name.igs",
  "status": "stored"
}
```

### 2. Get triangulated mesh for viewing

- **Endpoint:** `GET /api/models/{modelId}/mesh`
- **Response (200 JSON):**
```json
{
  "modelId": "string",
  "vertices": [0.0, 0.0, 0.0],
  "indices": [0, 1, 2],
  "normals": [0.0, 1.0, 0.0],
  "bbox": {
    "min": [xmin, ymin, zmin],
    "max": [xmax, ymax, zmax]
  }
}
```

The mesh format is intentionally simple and compatible with Three.js `BufferGeometry`.

### 3. Compute initial path

- **Endpoint:** `POST /api/models/{modelId}/paths`
- **Request (JSON):**
```json
{
  "cameraRadius": 0.1,
  "samplingResolution": [40, 40, 40],
  "strategy": "orbit"
}
```

- **Response (201 JSON):**
```json
{
  "pathId": "string",
  "modelId": "string",
  "points": [
    { "x": 0.0, "y": 0.0, "z": 0.0 },
    { "x": 0.1, "y": 0.0, "z": 0.0 }
  ],
  "metadata": {
    "length": 1.23,
    "strategy": "orbit"
  }
}
```

`strategy` must support `"orbit"` initially, with potential for additional strategies later.

### 4. Validate an edited path

- **Endpoint:** `POST /api/models/{modelId}/paths/{pathId}/validate`
- **Request (JSON):**
```json
{
  "points": [
    { "x": 0.0, "y": 0.0, "z": 0.0 },
    { "x": 0.1, "y": 0.1, "z": 0.1 }
  ],
  "cameraRadius": 0.1
}
```

- **Response (200 JSON):**
```json
{
  "valid": true,
  "minClearance": 0.05,
  "violations": []
}
```

When `valid` is false, `violations` must contain details per offending point (index or coordinates and clearance).

### 5. Export path for external use (for example, SolidWorks)

- **Endpoint:** `GET /api/models/{modelId}/paths/{pathId}/export`
- **Query parameters:** `format=csv` (v1)
- **Response:** `text/csv` with header row and columns:
  - `x,y,z` (and optionally `t` for parameterization in a later version).

Example CSV:

```csv
x,y,z
0.0,0.0,0.0
0.1,0.0,0.0
```

---

## 0.3 Frontend–Backend Interaction Contract

The frontend must follow this call sequence:

1. User uploads model file:
   - Frontend calls `POST /api/models`.
   - Backend returns `{ modelId, filename, status }`.
2. Frontend stores `modelId` in state and calls:
   - `GET /api/models/{modelId}/mesh` to receive mesh data.
3. When user requests an auto-generated path:
   - Frontend calls `POST /api/models/{modelId}/paths` with default or user-defined parameters.
   - Frontend then renders the returned `points` as a path line and control spheres.
4. When user edits the path and requests validation:
   - Frontend sends `points` to `POST /api/models/{modelId}/paths/{pathId}/validate`.
5. When user exports a path:
   - Frontend downloads from `GET /api/models/{modelId}/paths/{pathId}/export?format=csv`.

---

## 0.4 Acceptance Criteria for Phase 0

- The architecture and directory layout above are accepted as the baseline.
- All endpoint paths, HTTP methods, and JSON schemas are defined and stable.
- No implementation code exists yet; only documentation and design.
- The contents of this file must be kept in sync with any future changes by later phases.
