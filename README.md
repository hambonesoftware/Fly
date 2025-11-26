# Flythrough Path Planner

## Overview

This repository scaffolds a system to automatically plan and visualize camera flythrough paths for CAD models in IGES/STEP formats. The project is organized into a backend and frontend which communicate via a REST API.  The backend will handle model uploads, geometric processing and path planning. The frontend will provide a Three.js‑based viewer and editing interface.

## Current State (Phase 0)

At this stage, the repository defines the directory structure and API contracts but does not include any implementation. Phase 0 is focused purely on architecture and contracts; code will be added in later phases by specialist agents. The key elements established here are the folder layout and the REST API endpoints that the frontend and backend will rely on.

### Directory Structure

```
fly_project/
  backend/
    app/
      __init__.py
      api/
        __init__.py
        # routes and schemas will be added in Phase 1
      core/
        __init__.py
        # configuration and logging will be added in Phase 1+
      services/
        __init__.py
        # storage, geometry and path planning services will live here
    tests/
      # backend tests will be added in future phases
  frontend/
    index.html
    css/
      styles.css
    js/
      main.js
      apiClient.js
      viewer.js
      pathOverlay.js
      ui.js
  docs/
    Phase0_architecture_and_contracts.md
  README.md  ← this file
```

### API Contract

The Phase 0 documentation defines a REST API that the backend will implement and the frontend will consume.  The endpoints cover:

- Uploading a model (`POST /api/models`)
- Retrieving a triangulated mesh (`GET /api/models/{modelId}/mesh`)
- Generating an initial camera path (`POST /api/models/{modelId}/paths`)
- Validating an edited path (`POST /api/models/{modelId}/paths/{pathId}/validate`)
- Exporting a path (`GET /api/models/{modelId}/paths/{pathId}/export?format=csv`)

Detailed schemas for requests and responses are provided in `docs/Phase0_architecture_and_contracts.md`.

### Next Steps

Phase 1 will implement the FastAPI scaffold, Pydantic schemas, and stubbed storage, geometry and path services. The frontend user interface and interactive features will be developed in Phases 2 and 3.  Refer to the `plan_fly` documents for the sequence of work and responsibilities.

## Backend Setup (Phase 1)

The backend is written in Python using FastAPI.  To run it locally you will need Python 3.8 or later.  Follow these steps to set up a virtual environment and install the dependencies:

1. **Create a virtual environment** (recommended):

   ```bash
   python -m venv venv
   source venv/bin/activate  # on Windows use `venv\Scripts\activate`
   ```

2. **Install required packages**.  Phase 1 depends on FastAPI and Uvicorn for serving the application.  Install them with pip:

   ```bash
   pip install fastapi uvicorn
   ```

   Additional packages such as `python-multipart` and `pydantic` are installed automatically as dependencies of FastAPI.

3. **Run the backend**.  From the `backend/app` directory, start the development server with:

   ```bash
   cd backend/app
   uvicorn main:app --reload
   ```

   The `--reload` flag enables auto‑reloading during development.  By default the server listens on `http://127.0.0.1:8000`.

Once the server is running, the automatically generated API documentation is available at `http://127.0.0.1:8000/docs`.

## Frontend Usage (Phase 2)

Phase 2 introduces a basic web interface built with HTML, CSS and ES module JavaScript.  The frontend uses Three.js to render a triangulated mesh and overlay a simple flythrough path returned by the backend.

### How to run the frontend

### Three.js local modules

The viewer uses Three.js as an ES module. To avoid a CDN dependency, the imports in
`frontend/js/viewer.js` and `frontend/js/pathOverlay.js` expect local copies of the
Three.js module and the OrbitControls helper under `frontend/vendor/`:

- `frontend/vendor/three.module.js`
- `frontend/vendor/OrbitControls.js`

You can obtain these files from an official Three.js distribution or from a CDN
such as unpkg, for example:

- `https://unpkg.com/three@0.164.0/build/three.module.js`
- `https://unpkg.com/three@0.164.0/examples/jsm/controls/OrbitControls.js`

Download those two files and place them in `frontend/vendor/` with the names
shown above. The relative imports in the JS files will then resolve correctly
when the app is served by FastAPI.



The FastAPI application mounts the `frontend/` directory as a set of static files.  When the backend is running (see instructions above), you can access the user interface by navigating to the root of the server:

```
http://127.0.0.1:8000/
```

The UI provides controls to:

1. **Select and upload a model** – choose an IGES or STEP file and click “Upload and Load Model.”  The backend stores the file and returns a `modelId`, then the stub mesh (a cube in Phase 2) is fetched and displayed.
2. **Compute a path** – click “Compute Path” to request an initial orbit‑style path from the backend.  The path is drawn as a red line around the model with green spheres at each waypoint.
3. **Inspect the model** – use your mouse to orbit, pan and zoom the view via the built‑in OrbitControls.  The status area reports progress and any errors encountered during API calls.

This frontend is intentionally minimal.  In subsequent phases it will gain interactive path editing, validation, and animation features.  For now it demonstrates the end‑to‑end flow of uploading a model, retrieving its mesh and visualising a generated path.

## Geometry Import and Path Planning (Phase 3+)

Phase 3 introduces the first integration of real CAD geometry into the flythrough planner.  When the optional `pythonocc-core` (also known as `OCP`) package is available, the backend will load IGES/STEP files using OCC readers, tessellate them into a triangle mesh and compute an accurate axis‑aligned bounding box.  This bounding box is used to plan a safe orbit path around the model.  If `pythonocc-core` is not installed the system automatically falls back to a stub cube so you can still test the workflow.

To enable geometry import you must install `pythonocc-core` in your environment.  This package has additional system dependencies (C++ libraries) and is not installed by default.  You can install it via pip:

```bash
pip install pythonocc-core
```

If you do not have OCC available the backend will quietly fall back to the stub cube, and all other functionality (path planning, editing, validation and export) will continue to work using the approximate bounding box.

### Path Editing, Validation and Export (Phase 3)

In Phase 3 the frontend has been extended with powerful new capabilities:

1. **Edit the path** – After computing a path, drag the green spheres in the 3D view to reposition control points.  The red line updates in real time to reflect your changes.
2. **Validate clearance** – Click **Validate Path** to check whether your edited path maintains a safe distance from the model.  The status message will display the minimum clearance and report any violations.  Clearance is approximated using the model’s bounding box.
3. **Export CSV** – Click **Export Path (CSV)** to download a CSV file with the current path points.  Each row contains `x,y,z` coordinates for import into external tools.
4. **Preview the flythrough** – Click **Play Flythrough** to animate the camera along the current path.  The view smoothly moves from point to point so you can review the path before exporting.
These features build upon the orbit‑style path generation added to the backend in Phase 3.  When OCC is available, the backend computes the bounding box from the actual uploaded model; otherwise it uses a stub cube.  Validation uses this bounding box to approximate clearance.  A future enhancement could compute true distances from the model surface, but for now the bounding box provides a useful proxy.

## API Endpoints

The backend exposes the following HTTP endpoints in Phase 1.  All paths are prefixed with `/api` when registered with the application.

### `GET /api/health`

Health check endpoint.  Returns a simple JSON object to indicate that the service is running.

**Response:**

```json
{
  "status": "ok"
}
```

### `POST /api/models`

Upload a CAD model file in IGES or STEP format.  The file must be sent as `multipart/form-data` with the form field named `file`.

**Example request using `curl`:**

```bash
curl -X POST http://127.0.0.1:8000/api/models \
     -F "file=@/path/to/model.igs"
```

**Response (201):**

```json
{
  "modelId": "d63f6b23d4be4a9f8ad9ab5e5312f321",
  "filename": "model.igs",
  "status": "stored"
}
```

### `GET /api/models/{modelId}/mesh`

Retrieve a triangulated mesh for the uploaded model.  In Phase 1 this returns a hard‑coded cube mesh regardless of the model.

**Response (200):**

```json
{
  "modelId": "d63f6b23d4be4a9f8ad9ab5e5312f321",
  "vertices": [-0.5, -0.5, -0.5, 0.5, -0.5, -0.5, …],
  "indices": [0, 1, 2, 2, 3, 0, …],
  "normals": [0.0, 0.0, 0.0, …],
  "bbox": {
    "min": [-0.5, -0.5, -0.5],
    "max": [0.5, 0.5, 0.5]
  }
}
```

### `POST /api/models/{modelId}/paths`

Generate a stub flythrough path for the given model.  The request body must include a JSON object matching the `PathCreateRequest` schema:

```json
{
  "cameraRadius": 0.1,
  "samplingResolution": [40, 40, 40],
  "strategy": "orbit"
}
```

These parameters are ignored in Phase 1; the service always returns a circular path of radius 2 centred at the origin.

**Response (201):**

```json
{
  "pathId": "94c0d8bba4f84b20a27e0b4dd3c0863f",
  "modelId": "d63f6b23d4be4a9f8ad9ab5e5312f321",
  "points": [
    {"x": 2.0, "y": 0.0, "z": 0.0},
    {"x": 1.96, "y": 0.31, "z": 0.0},
    …
  ],
  "metadata": {
    "length": 12.566370614359172,
    "strategy": "orbit"
  }
}
```

### `POST /api/models/{modelId}/paths/{pathId}/validate`

Validate a user‑edited path.  The request body must include the current list of points and a camera radius:

```json
{
  "points": [
    {"x": 2.0, "y": 0.0, "z": 0.0},
    {"x": 1.96, "y": 0.31, "z": 0.0},
    …
  ],
  "cameraRadius": 0.1
}
```

In Phase 1 validation always succeeds and the response is constant:

```json
{
  "valid": true,
  "minClearance": 999.0,
  "violations": []
}
```

### `GET /api/models/{modelId}/paths/{pathId}/export`

Export the stored path as a CSV file.  The response will have content type `text/csv` and contain a header row followed by one row per point:

```csv
x,y,z
2.0,0.0,0.0
1.96,0.31,0.0
…
```

You can download the CSV using a tool such as `curl`:

```bash
curl -O -J http://127.0.0.1:8000/api/models/d63f6b23d4be4a9f8ad9ab5e5312f321/paths/94c0d8bba4f84b20a27e0b4dd3c0863f/export
```

## Current Limitations

* The geometry importer still returns a static cube mesh; IGES/STEP files are stored but not parsed.  Bounding boxes are computed from this stub mesh.
* The path planner generates an orbit around the model’s bounding box.  More sophisticated path planning strategies and true surface clearance checks are not yet implemented.
* Clearance validation uses the bounding box rather than the actual model surface.  It may permit or reject paths that would behave differently on a real model.

These constraints exist because PythonOCC/OCP and true volumetric distance checks were not available in the current environment.  Future work will integrate those libraries to provide accurate geometry processing.
