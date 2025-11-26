/*
 * API client for the flythrough planner frontend.
 *
 * This module provides asynchronous functions to talk to the backend
 * REST API defined in Phase 0. Each function returns JSON
 * responses or throws an error if the HTTP request fails.  The
 * backend is assumed to be hosted on the same origin as the
 * frontend, so relative URLs are used. Adjust `API_BASE` if you
 * deploy the frontend and backend on different hosts.
 */

const API_BASE = '/api';

/**
 * Upload a CAD model file (IGES/STEP) to the backend.
 *
 * The backend expects a multipart/form-data payload with a single
 * field named "file". On success it returns a JSON object
 * containing a generated `modelId` which can be used to query
 * meshes and generate paths.
 *
 * @param {File} file - The file selected by the user
 * @returns {Promise<object>} Resolves with ModelInfo JSON
 * @throws {Error} If the HTTP request fails
 */
export async function uploadModel(file) {
  const formData = new FormData();
  formData.append('file', file);
  const response = await fetch(`${API_BASE}/models`, {
    method: 'POST',
    body: formData,
  });
  if (!response.ok) {
    const text = await safeParseError(response);
    throw new Error(`Upload failed: ${text}`);
  }
  return await response.json();
}

/**
 * Retrieve the triangulated mesh for a given model.
 *
 * In Phase 1 the backend returns a hard‑coded cube mesh.  Later
 * phases will return the actual tessellated geometry.  The
 * response adheres to the MeshResponse schema defined in
 * `api/models.py`.
 *
 * @param {string} modelId - Identifier returned from uploadModel
 * @returns {Promise<object>} Resolves with MeshResponse JSON
 * @throws {Error} If the HTTP request fails
 */
export async function getMesh(modelId) {
  const response = await fetch(`${API_BASE}/models/${encodeURIComponent(modelId)}/mesh`, {
    method: 'GET',
  });
  if (!response.ok) {
    const text = await safeParseError(response);
    throw new Error(`Failed to get mesh: ${text}`);
  }
  return await response.json();
}

/**
 * Request a flythrough path for a given model.
 *
 * The backend uses the provided parameters to plan a path. In
 * Phase 1 and 2 the planner ignores most fields and always
 * returns a simple circular path.  The response matches the
 * PathResponse schema from `api/models.py`.
 *
 * @param {string} modelId - Identifier for the uploaded model
 * @param {object} params - Object with cameraRadius, samplingResolution and strategy
 * @returns {Promise<object>} Resolves with PathResponse JSON
 * @throws {Error} If the HTTP request fails
 */
export async function createPath(modelId, params) {
  const response = await fetch(`${API_BASE}/models/${encodeURIComponent(modelId)}/paths`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(params),
  });
  if (!response.ok) {
    const text = await safeParseError(response);
    throw new Error(`Failed to create path: ${text}`);
  }
  return await response.json();
}

/**
 * Validate a user-edited path against the model's geometry.
 *
 * Sends the current list of path points and the desired camera
 * clearance to the backend.  The backend responds with whether
 * the path is valid, the minimum clearance found, and details of
 * any violations.
 *
 * @param {string} modelId - Identifier for the uploaded model
 * @param {string} pathId - Identifier for the path previously generated
 * @param {Array<Object>} points - Array of {x, y, z} objects defining the path
 * @param {number} cameraRadius - Required clearance distance
 * @returns {Promise<object>} Resolves with PathValidateResponse JSON
 * @throws {Error} If the HTTP request fails
 */
export async function validatePath(modelId, pathId, points, cameraRadius) {
  const response = await fetch(
    `${API_BASE}/models/${encodeURIComponent(modelId)}/paths/${encodeURIComponent(pathId)}/validate`,
    {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ points, cameraRadius }),
    }
  );
  if (!response.ok) {
    const text = await safeParseError(response);
    throw new Error(`Validation failed: ${text}`);
  }
  return await response.json();
}

/**
 * Build the URL to export a path as CSV.
 *
 * The returned URL can be assigned to `window.location.href` or used
 * as the `href` of a temporary link to trigger download.  The
 * backend handles exporting the stored path identified by `pathId`.
 *
 * @param {string} modelId - Identifier for the model
 * @param {string} pathId - Identifier for the path
 * @returns {string} Absolute URL to the CSV export endpoint
 */
export function getExportUrl(modelId, pathId) {
  return `${API_BASE}/models/${encodeURIComponent(modelId)}/paths/${encodeURIComponent(pathId)}/export`;
}

/**
 * Build the URL to export a path in SolidWorks-friendly CSV format.
 *
 * This endpoint includes an index column and uses capitalised
 * headers expected by certain CAD macros.
 *
 * @param {string} modelId - Identifier for the model
 * @param {string} pathId - Identifier for the path
 * @returns {string} Absolute URL to the SolidWorks export endpoint
 */
export function getExportSolidworksUrl(modelId, pathId, mode = 'dense') {
  // Append the mode as a query parameter.  Valid values are 'dense'
  // (default) and 'controlPoints'.  The backend will fall back to
  // dense when control points are unavailable.
  const m = mode || 'dense';
  return `${API_BASE}/models/${encodeURIComponent(modelId)}/paths/${encodeURIComponent(pathId)}/export/solidworks?mode=${encodeURIComponent(m)}`;
}

/**
 * Retrieve a list of all uploaded models with their status.
 *
 * @returns {Promise<Array<object>>} Resolves with an array of ModelStatusInfo objects
 * @throws {Error} If the HTTP request fails
 */
export async function listModels() {
  const response = await fetch(`${API_BASE}/models`, { method: 'GET' });
  if (!response.ok) {
    const text = await safeParseError(response);
    throw new Error(`Failed to list models: ${text}`);
  }
  return await response.json();
}

/**
 * Retrieve status information for a single model.
 *
 * @param {string} modelId - Identifier of the model
 * @returns {Promise<object>} Resolves with a ModelStatusInfo object
 * @throws {Error} If the HTTP request fails
 */
export async function getModel(modelId) {
  const response = await fetch(`${API_BASE}/models/${encodeURIComponent(modelId)}`, {
    method: 'GET',
  });
  if (!response.ok) {
    const text = await safeParseError(response);
    throw new Error(`Failed to get model: ${text}`);
  }
  return await response.json();
}

/**
 * Request perimeter loops for a given model.
 *
 * The backend collapses the mesh onto the selected plane, stitches
 * boundary edges into loops and removes interior polygons.  The
 * returned loops correspond to the outer footprint of the model in
 * the projection plane.  See the backend docs for details on the
 * returned structure.
 *
 * @param {string} modelId - Identifier of the uploaded model
 * @param {object} params - Query parameters controlling the perimeter extraction:
 *   plane (xy, xz, yz), outlineStitchPercent (0–100), perimeterIslandMode (single|multi), perimeterDetail (currently unused)
 * @returns {Promise<object>} Resolves with the perimeter response JSON
 * @throws {Error} If the HTTP request fails
 */
export async function getPerimeter(modelId, params = {}) {
  const query = new URLSearchParams();
  if (params.plane) query.append('plane', params.plane);
  if (params.outlineStitchPercent !== undefined && params.outlineStitchPercent !== null) {
    query.append('outlineStitchPercent', params.outlineStitchPercent);
  }
  if (params.perimeterIslandMode) query.append('perimeterIslandMode', params.perimeterIslandMode);
  if (params.perimeterDetail) query.append('perimeterDetail', params.perimeterDetail);
  const url = `${API_BASE}/models/${encodeURIComponent(modelId)}/perimeter` + (query.toString() ? `?${query.toString()}` : '');
  const response = await fetch(url, { method: 'GET' });
  if (!response.ok) {
    const text = await safeParseError(response);
    throw new Error(`Failed to compute perimeter: ${text}`);
  }
  return await response.json();
}

/**
 * Delete a model from the backend.
 *
 * @param {string} modelId - Identifier of the model to delete
 * @returns {Promise<void>} Resolves when deletion succeeds
 * @throws {Error} If the HTTP request fails
 */
export async function deleteModel(modelId) {
  const response = await fetch(`${API_BASE}/models/${encodeURIComponent(modelId)}`, {
    method: 'DELETE',
  });
  if (!response.ok) {
    const text = await safeParseError(response);
    throw new Error(`Failed to delete model: ${text}`);
  }
  return;
}

/**
 * Retrieve the base perimeter outline of a model on a given plane.
 *
 * This helper wraps the backend `/models/{model_id}/outline` GET endpoint.
 * It accepts a model identifier and an options object describing the
 * projection plane, height offset and whether to use the model outline
 * (HLR) versus a mesh slice.  Additional parameters control the stitch
 * tolerance and sampling detail for the HLR outline.  The backend
 * responds with an object conforming to the OutlineResponse schema.
 *
 * @param {string} modelId - Identifier of the uploaded model
 * @param {Object} options - Query parameters for the outline
 * @param {string} [options.plane] - One of 'xy', 'xz' or 'yz'
 * @param {number} [options.heightOffset] - Height offset along the orthogonal axis
 * @param {boolean} [options.useModelOutline] - Whether to use HLR projection
 * @param {number|null} [options.outlineStitchTolerance] - Stitch tolerance for HLR
 * @param {string|null} [options.outlineDetail] - Detail level for HLR ('low','medium','high','auto')
 * @returns {Promise<object>} Resolves with OutlineResponse JSON
 * @throws {Error} If the HTTP request fails
 */
export async function getOutline(modelId, options = {}) {
  const params = new URLSearchParams();
  if (options.plane) {
    params.append('plane', options.plane);
  }
  if (options.heightOffset !== undefined && options.heightOffset !== null) {
    params.append('heightOffset', options.heightOffset);
  }
  if (options.useModelOutline) {
    params.append('useModelOutline', options.useModelOutline);
  }
  if (options.outlineStitchTolerance !== undefined && options.outlineStitchTolerance !== null) {
    params.append('outlineStitchTolerance', options.outlineStitchTolerance);
  }
  if (options.outlineDetail) {
    params.append('outlineDetail', options.outlineDetail);
  }
  // New in v0.9.7: pass relative stitch percent and island mode
  if (options.outlineStitchPercent !== undefined && options.outlineStitchPercent !== null) {
    params.append('outlineStitchPercent', options.outlineStitchPercent);
  }
  if (options.perimeterIslandMode) {
    params.append('perimeterIslandMode', options.perimeterIslandMode);
  }
  const query = params.toString();
  const url = `${API_BASE}/models/${encodeURIComponent(modelId)}/outline${query ? '?' + query : ''}`;
  const response = await fetch(url, { method: 'GET' });
  if (!response.ok) {
    const text = await safeParseError(response);
    throw new Error(`Failed to get outline: ${text}`);
  }
  return await response.json();
}

/**
 * Safely parse error responses.  Attempts to read the body as JSON
 * and fall back to plain text if parsing fails.  Used internally
 * when constructing error messages.
 *
 * @param {Response} response - The fetch Response object
 * @returns {Promise<string>} Parsed error message
 */
async function safeParseError(response) {
  try {
    const data = await response.json();
    return data?.detail || JSON.stringify(data);
  } catch (err) {
    try {
      return await response.text();
    } catch (e) {
      return response.statusText;
    }
  }
}
