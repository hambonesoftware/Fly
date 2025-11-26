/*
 * Main entry point for the flythrough planner frontend.
 *
 * This script wires up the HTML controls to the backend API and
 * Three.js viewer.  It handles file uploads, mesh loading and
 * path generation.  Errors are reported in the status div and
 * logged to the browser console.
 */

import {
  uploadModel,
  getMesh,
  createPath,
  validatePath,
  getExportUrl,
  getExportSolidworksUrl,
  listModels,
  getModel,
  deleteModel,
} from './apiClient.js';
import {
  initViewer,
  addMeshToScene,
  addPathToScene,
  showPath,
  playFlythrough,
  showBaseOutline,
  showMinkowskiBoundary,
} from './viewer.js';
import { getCurrentPathPoints } from './pathOverlay.js';
import { setStatus, setBusy } from './ui.js';
// Import getOutline for outline preview requests
// Import getOutline and getPerimeter for outline preview and perimeter extraction
import { getOutline, getPerimeter } from './apiClient.js';

// Default parameters for path computation.  These constants define
// the camera's clearance from the model and the resolution used to
// sample each axis.  They are used in Phase 2 to implement the
// perimeter strategy by default.
const DEFAULT_CAMERA_RADIUS = 20.0;
const DEFAULT_SAMPLING_RES = [80, 80, 80];

// Keep track of the current model and path identifiers
let currentModelId = null;
let currentPathId = null;
// Store last computed path points for validation/export/play
let lastPathPoints = [];

// The setStatus and setBusy helpers are imported from ui.js

/**
 * Set up event listeners once the DOM is fully loaded.
 */
function bootstrap() {
  const fileInput = document.getElementById('fileInput');
  const uploadButton = document.getElementById('uploadButton');
  const computePathButton = document.getElementById('computePathButton');
  const findPerimeterButton = document.getElementById('findPerimeterButton');
  // Strategy selector added in Phase 2.  Allows users to choose
  // between perimeter and orbit strategies (perimeter is default).
  const strategySelect = document.getElementById('strategySelect');
  // Plane selection and height offset controls (Phase 6)
  const planeSelect = document.getElementById('planeSelect');
  const heightOffsetInput = document.getElementById('heightOffsetInput');
  // Perimeter detail and spline controls (Phase 5 + 6b)
  const perimeterDetailSelect = document.getElementById('perimeterDetailSelect');
  const useSplineCheckbox = document.getElementById('useSplineCheckbox');
  const useModelOutlineCheckbox = document.getElementById('useModelOutlineCheckbox');
  const outlineStitchToleranceInput = document.getElementById('outlineStitchToleranceInput');
  const outlineDetailSelect = document.getElementById('outlineDetailSelect');
  // New controls introduced in v0.9.7
  const outlineStitchPercentInput = document.getElementById('outlineStitchPercentInput');
  const perimeterIslandModeSelect = document.getElementById('perimeterIslandModeSelect');
  // New controls introduced in v0.9.11 for rolling‑circle perimeter paths
  const useRollingCircleCheckbox = document.getElementById('useRollingCircleCheckbox');
  const circleDiameterInput = document.getElementById('circleDiameterInput');
  const canvas = document.getElementById('viewerCanvas');
  const modelListDiv = document.getElementById('modelList');
  // Initialize the 3D viewer
  initViewer(canvas);

  /**
   * Update the outline preview shown around the current model.
   *
   * This helper gathers the current plane, height offset and outline
   * options from the UI and queries the backend for an outline.  The
   * returned points are rendered via showBaseOutline().  A status
   * message is displayed indicating the outline source and point count.
   */
  async function updateOutlinePreview() {
    // Only update when a model is loaded
    if (!currentModelId) return;
    // Compose parameters
    const planeVal = planeSelect ? planeSelect.value : 'xy';
    let heightVal = 0.0;
    if (heightOffsetInput && heightOffsetInput.value !== '') {
      const parsed = parseFloat(heightOffsetInput.value);
      if (!isNaN(parsed)) heightVal = parsed;
    }
    const useOutline = !!(useModelOutlineCheckbox && useModelOutlineCheckbox.checked);
    let stitchTol = null;
    if (outlineStitchToleranceInput && outlineStitchToleranceInput.value !== '') {
      const parsedTol = parseFloat(outlineStitchToleranceInput.value);
      if (!Number.isNaN(parsedTol) && parsedTol >= 0) {
        stitchTol = parsedTol;
      }
    }
    let detail = null;
    if (outlineDetailSelect) {
      const d = outlineDetailSelect.value;
      detail = d || null;
    }
    // New relative stitch percent and island mode
    let stitchPercentVal = null;
    if (outlineStitchPercentInput && outlineStitchPercentInput.value !== '') {
      const parsedPct = parseFloat(outlineStitchPercentInput.value);
      if (!Number.isNaN(parsedPct)) {
        stitchPercentVal = parsedPct;
      }
    }
    const islandModeVal = perimeterIslandModeSelect ? perimeterIslandModeSelect.value : 'single';
    // Fetch outline from backend
    setBusy(true);
    setStatus('Updating outline preview…');
    try {
      const resp = await getOutline(currentModelId, {
        plane: planeVal,
        heightOffset: heightVal,
        useModelOutline: useOutline,
        outlineStitchTolerance: stitchTol,
        outlineDetail: detail,
        outlineStitchPercent: stitchPercentVal,
        perimeterIslandMode: islandModeVal,
      });
      if (resp && Array.isArray(resp.points)) {
        // The outline preview currently returns a single list of points.  If
        // future versions return multiple outlines, showBaseOutline can be
        // extended accordingly.
        const pts = resp.points.map((p) => ({ x: p.x, y: p.y, z: p.z }));
        showBaseOutline(pts);
        const md = resp.metadata || {};
        let msg = `Outline preview updated (source=${md.perimeterSource || 'unknown'}, points=${pts.length})`;
        let warn = false;
        if (md.perimeterSource === 'bboxFallback') {
          warn = true;
        }
        setStatus(msg, false, warn);
      }
    } catch (err) {
      console.error(err);
      setStatus(err.message || 'Failed to update outline preview.', true);
    } finally {
      setBusy(false);
    }
  }

  /**
   * Compute and display the perimeter loops for the current model.
   *
   * This helper collects the current perimeter parameters from the UI
   * (plane, relative stitch percent and island mode), invokes the
   * backend perimeter endpoint and draws the returned loops in the
   * viewer.  The status message is updated to reflect the number of
   * loops and points returned.  Any previous perimeter is cleared
   * before drawing the new one.
   */
  async function findPerimeter() {
    if (!currentModelId) {
      setStatus('Upload and load a model first.', true);
      return;
    }
    // Collect parameters from UI
    const planeVal = planeSelect ? planeSelect.value : 'xy';
    let stitchPercentVal = null;
    if (outlineStitchPercentInput && outlineStitchPercentInput.value !== '') {
      const parsedPct = parseFloat(outlineStitchPercentInput.value);
      if (!Number.isNaN(parsedPct)) {
        stitchPercentVal = parsedPct;
      }
    }
    const islandModeVal = perimeterIslandModeSelect ? perimeterIslandModeSelect.value : 'single';
    let detailVal = null;
    if (perimeterDetailSelect) {
      detailVal = perimeterDetailSelect.value || null;
    }
    setBusy(true);
    setStatus('Finding perimeter…');
    try {
      const resp = await getPerimeter(currentModelId, {
        plane: planeVal,
        outlineStitchPercent: stitchPercentVal,
        perimeterIslandMode: islandModeVal,
        perimeterDetail: detailVal,
      });
      if (resp && Array.isArray(resp.loops)) {
        // Convert loops into arrays of {x,y,z} objects
        const loopsPts = resp.loops.map((loop) => {
          return loop.points.map((p) => {
            return { x: p[0], y: p[1], z: p[2] };
          });
        });
        // Draw loops; showBaseOutline accepts an array of loops
        showBaseOutline(loopsPts);
        // Update status
        const md = resp.meta || {};
        const loopsCount = md.totalLoops !== undefined ? md.totalLoops : resp.loops.length;
        const pointCount = md.totalPoints !== undefined ? md.totalPoints : loopsPts.reduce((acc, arr) => acc + arr.length, 0);
        const modeStr = md.islandMode || islandModeVal;
        setStatus(`Perimeter found: loops=${loopsCount}, points=${pointCount} (mode=${modeStr})`);
      } else {
        setStatus('No perimeter loops found.', true);
        showBaseOutline([]);
      }
    } catch (err) {
      console.error(err);
      setStatus(err.message || 'Failed to compute perimeter.', true);
      showBaseOutline([]);
    } finally {
      setBusy(false);
    }
  }

  // Populate the model list on initial load
  refreshModelList();

  // Helper to refresh the list of models from the backend
  async function refreshModelList() {
    if (!modelListDiv) return;
    try {
      const models = await listModels();
      // Clear existing contents
      modelListDiv.innerHTML = '';
      if (!models || models.length === 0) {
        modelListDiv.textContent = 'No models uploaded.';
        return;
      }
      const list = document.createElement('ul');
      list.className = 'model-list';
      models.forEach((model) => {
        const li = document.createElement('li');
        li.className = 'model-item';
        // Display model name and status
        const nameSpan = document.createElement('span');
        nameSpan.textContent = `${model.name || model.modelId} (`;
        const statusSpan = document.createElement('span');
        statusSpan.textContent = model.status;
        statusSpan.className = `status-${model.status}`;
        nameSpan.appendChild(statusSpan);
        nameSpan.append(')');
        li.appendChild(nameSpan);
        // Delete button
        const delBtn = document.createElement('button');
        delBtn.textContent = 'Delete';
        delBtn.className = 'delete-button';
        delBtn.addEventListener('click', async (event) => {
          event.stopPropagation();
          try {
            await deleteModel(model.modelId);
            setStatus(`Deleted model ${model.name || model.modelId}`);
            // If current model was deleted, clear viewer
            if (currentModelId === model.modelId) {
              currentModelId = null;
              // TODO: Clear mesh from scene if necessary
            }
            refreshModelList();
          } catch (err) {
            console.error(err);
            setStatus(err.message || 'Failed to delete model.', true);
          }
        });
        li.appendChild(delBtn);
        // Click handler to load the mesh and set currentModelId
        li.addEventListener('click', async () => {
          setBusy(true);
          setStatus('Loading mesh…');
          try {
            currentModelId = model.modelId;
            const meshResponse = await getMesh(currentModelId);
            addMeshToScene(meshResponse);
            setStatus('Mesh loaded. You can now compute a path.');
            // Show an initial outline preview for the selected model
            updateOutlinePreview();
          } catch (err) {
            console.error(err);
            setStatus(err.message || 'Failed to load mesh.', true);
          } finally {
            setBusy(false);
          }
        });
        list.appendChild(li);
      });
      modelListDiv.appendChild(list);
    } catch (error) {
      console.error(error);
      modelListDiv.textContent = 'Failed to fetch model list.';
    }
  }
  // File upload handler
  uploadButton?.addEventListener('click', async () => {
    if (!fileInput || fileInput.files.length === 0) {
      setStatus('Please choose a .igs/.iges/.stp/.step file to upload.', true);
      return;
    }
    const file = fileInput.files[0];
    // Indicate busy state and update status
    setBusy(true);
    setStatus('Uploading model…');
    try {
      const modelInfo = await uploadModel(file);
      currentModelId = modelInfo.modelId;
      setStatus(`Model uploaded (id: ${currentModelId}). Loading mesh…`);
      const meshResponse = await getMesh(currentModelId);
      addMeshToScene(meshResponse);
      setStatus('Mesh loaded. You can now compute a path.');
      // Refresh the model list so the new model appears with status
      refreshModelList();
    } catch (error) {
      console.error(error);
      setStatus(error.message || 'Failed to upload/load model.', true);
    } finally {
      setBusy(false);
    }
  });
  // Compute path handler
  computePathButton?.addEventListener('click', async () => {
    if (!currentModelId) {
      setStatus('Upload and load a model first.', true);
      return;
    }
    setBusy(true);
    setStatus('Computing path…');
    try {
      // Determine strategy and parameters from UI.  Default to
      // perimeter if the element is missing.
      const strategy = strategySelect ? strategySelect.value : 'perimeter';
      const cameraRadius = DEFAULT_CAMERA_RADIUS;
      const samplingResolution = DEFAULT_SAMPLING_RES;
      // Read bulge and smoothness inputs from the UI.  These are optional
      // tuning parameters for the perimeter strategy introduced in Phase 4.
      const bulgeInput = document.getElementById('bulgeInput');
      const smoothnessInput = document.getElementById('smoothnessInput');
      const bulgeVal = bulgeInput ? parseFloat(bulgeInput.value) : 0.0;
      const smoothVal = smoothnessInput ? parseFloat(smoothnessInput.value) : 0.5;
      // Determine plane and height offset from the UI.  These fields are
      // optional and are only used by the perimeter strategy; the
      // backend provides defaults when omitted.  Height offset may
      // parse to NaN if the input is empty; guard against this.
      const planeVal = planeSelect ? planeSelect.value : 'xy';
      let heightVal = 0.0;
      if (heightOffsetInput && heightOffsetInput.value !== '') {
        const parsed = parseFloat(heightOffsetInput.value);
        if (!isNaN(parsed)) heightVal = parsed;
      }
      // Read advanced perimeter options.  If the select or checkbox is
      // missing (e.g. in older UIs), the values will be undefined and
      // omitted from the request body.  The backend treats undefined
      // perimeterDetail and useSpline as defaults.
      let detailVal;
      if (perimeterDetailSelect) {
        detailVal = perimeterDetailSelect.value;
      }
      let useSplineVal;
      if (useSplineCheckbox) {
        useSplineVal = useSplineCheckbox.checked;
      }
      let outlineStitchTolVal = null;
      if (outlineStitchToleranceInput && outlineStitchToleranceInput.value !== '') {
        const parsedOutlineTol = parseFloat(outlineStitchToleranceInput.value);
        if (!Number.isNaN(parsedOutlineTol) && parsedOutlineTol >= 0) {
          outlineStitchTolVal = parsedOutlineTol;
        }
      }
      let outlineDetailVal = null;
      if (outlineDetailSelect) {
        const od = outlineDetailSelect.value;
        outlineDetailVal = od || null;
      }
      // Read new outline stitch percent and island mode values (v0.9.7)
      let outlineStitchPercentVal = null;
      if (outlineStitchPercentInput && outlineStitchPercentInput.value !== '') {
        const parsedPct = parseFloat(outlineStitchPercentInput.value);
        if (!Number.isNaN(parsedPct)) {
          outlineStitchPercentVal = parsedPct;
        }
      }
      const perimeterIslandModeVal = perimeterIslandModeSelect ? perimeterIslandModeSelect.value : 'single';
      // Read rolling‑circle perimeter options.  These fields are optional and
      // default to false/auto when not provided.  The numeric field
      // accepts any positive value; non‑numeric or empty values result in null.
      const useRollingCircle = useRollingCircleCheckbox ? useRollingCircleCheckbox.checked : false;
      let circleDiameterVal = null;
      if (circleDiameterInput && circleDiameterInput.value !== '') {
        const parsedCircle = parseFloat(circleDiameterInput.value);
        if (!Number.isNaN(parsedCircle)) {
          circleDiameterVal = parsedCircle;
        }
      }
      const params = {
        cameraRadius: cameraRadius,
        samplingResolution: samplingResolution,
        strategy: strategy,
        bulgeFactor: bulgeVal,
        smoothness: smoothVal,
        plane: planeVal,
        heightOffset: heightVal,
        perimeterDetail: detailVal,
        useSpline: useSplineVal,
        useModelOutline: !!(useModelOutlineCheckbox && useModelOutlineCheckbox.checked),
        outlineStitchTolerance: outlineStitchTolVal,
        outlineDetail: outlineDetailVal,
        outlineStitchPercent: outlineStitchPercentVal,
        perimeterIslandMode: perimeterIslandModeVal,
        perimeterUseRollingCircle: useRollingCircle,
        perimeterCircleDiameter: circleDiameterVal,
      };
      const pathResponse = await createPath(currentModelId, params);
      currentPathId = pathResponse.pathId;
      lastPathPoints = pathResponse.points.map((p) => ({ x: p.x, y: p.y, z: p.z }));
      // Display path and enable editing
      showPath(lastPathPoints);
      // Render Minkowski boundary loops when available
      const minkowskiLoops = pathResponse.minkowskiBoundaryLoops;
      if (minkowskiLoops && Array.isArray(minkowskiLoops) && minkowskiLoops.length > 0) {
        const normalizedLoops = minkowskiLoops.map((loop) =>
          loop.map((p) => ({ x: p.x, y: p.y, z: p.z }))
        );
        showMinkowskiBoundary(normalizedLoops);
      } else {
        showMinkowskiBoundary([]);
      }
      // Compose status message including plane and height offset when using the
      // perimeter strategy.  For orbit strategy we omit these details.  We
      // also surface extra metadata such as self‑intersection and offset
      // adjustments to help users understand any warnings.
      let statusMsg = `Path computed using '${strategy}' strategy (radius=${cameraRadius}, points=${lastPathPoints.length})`;
      let isWarning = false;
      if (strategy === 'perimeter') {
        statusMsg += ` on ${planeVal.toUpperCase()} plane at offset ${heightVal}`;
        if (pathResponse.metadata) {
          const md = pathResponse.metadata;
          // Indicate where the perimeter came from (slice, outline or bbox)
          if (md.perimeterSource) {
            statusMsg += `, source=${md.perimeterSource}`;
            if (md.perimeterSource === 'bboxFallback') {
              isWarning = true;
            }
          }
          // Show sample counts when available
          if (md.sampledPointCount !== undefined && md.controlPointCount !== undefined) {
            statusMsg += `, points=${md.sampledPointCount} (control ${md.controlPointCount})`;
          }
          // Show self‑intersection and offset adjustments
          if (md.selfIntersection) {
            statusMsg += ', selfIntersection=true';
            isWarning = true;
          }
          if (md.offsetAdjusted) {
            const eff = md.effectiveOffset ?? 0;
            const effStr = typeof eff === 'number' ? eff.toFixed(4) : eff;
            statusMsg += `, offsetAdjusted=true (effectiveOffset=${effStr})`;
            isWarning = true;
          }
        }
      }
      if (isWarning) {
        statusMsg = `⚠ ${statusMsg}`;
      }
      statusMsg += '. You can drag the green spheres to edit it.';
      setStatus(statusMsg, false, isWarning);
      // Show the base outline from the response metadata if available.  This
      // provides a visual reference for the clearance used when generating
      // the path.  If present, the base outline is a list of PathPoint
      // objects that we convert into simple {x,y,z} objects.
      if (pathResponse.metadata && Array.isArray(pathResponse.metadata.baseOutline)) {
        const baseOutlinePts = pathResponse.metadata.baseOutline.map((p) => ({ x: p.x, y: p.y, z: p.z }));
        showBaseOutline(baseOutlinePts);
      }
    } catch (error) {
      console.error(error);
      setStatus(error.message || 'Failed to compute path.', true);
      showMinkowskiBoundary([]);
    } finally {
      setBusy(false);
    }
  });

  // Find perimeter handler
  findPerimeterButton?.addEventListener('click', findPerimeter);

  // Validate path handler
  const validateButton = document.getElementById('validatePathButton');
  validateButton?.addEventListener('click', async () => {
    if (!currentModelId || !currentPathId) {
      setStatus('Compute a path before validation.', true);
      return;
    }
    setBusy(true);
    try {
      const points = getCurrentPathPoints();
      lastPathPoints = points;
      setStatus('Validating path…');
      // Use the same camera radius as used for path creation
      const result = await validatePath(currentModelId, currentPathId, points, DEFAULT_CAMERA_RADIUS);
      if (result.valid) {
        setStatus(`Path is valid. Minimum clearance: ${result.minClearance.toFixed(2)}`);
      } else {
        setStatus(
          `Path invalid. Minimum clearance: ${result.minClearance.toFixed(
            2
          )}. Violations: ${result.violations.length}`,
          true
        );
      }
    } catch (error) {
      console.error(error);
      setStatus(error.message || 'Path validation failed.', true);
    } finally {
      setBusy(false);
    }
  });

  // Export path handler
  const exportButton = document.getElementById('exportPathButton');
  exportButton?.addEventListener('click', () => {
    if (!currentModelId || !currentPathId) {
      setStatus('Compute a path before exporting.', true);
      return;
    }
    const url = getExportUrl(currentModelId, currentPathId);
    // Create a temporary link element to trigger download
    const link = document.createElement('a');
    link.href = url;
    link.download = `path_${currentPathId}.csv`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    setStatus('Exported path CSV.');
  });

  // SolidWorks export handler
  const exportSolidworksButton = document.getElementById('exportSolidworksButton');
  exportSolidworksButton?.addEventListener('click', () => {
    if (!currentModelId || !currentPathId) {
      setStatus('Compute a path before exporting.', true);
      return;
    }
    const url = getExportSolidworksUrl(currentModelId, currentPathId);
    // Create a temporary link element to trigger download
    const link = document.createElement('a');
    link.href = url;
    link.download = `path_${currentPathId}_solidworks.csv`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    setStatus('Exported SolidWorks CSV.');
  });

  // Play flythrough handler
  const playButton = document.getElementById('playFlythroughButton');
  playButton?.addEventListener('click', () => {
    if (!lastPathPoints || lastPathPoints.length === 0) {
      setStatus('Compute a path before playing flythrough.', true);
      return;
    }
    playFlythrough(lastPathPoints, 10);
    setStatus('Playing flythrough…');
  });

  // In v0.9.9 the outline preview is no longer recomputed automatically
  // when perimeter parameters change.  Users must click the Find Perimeter
  // button to trigger a perimeter computation.  Therefore, the event
  // listeners that previously invoked updateOutlinePreview are intentionally
  // omitted.  Parameter changes will only affect the next perimeter
  // computation when the user clicks Find Perimeter.
}

// Ensure bootstrap runs after DOM is ready
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', bootstrap);
} else {
  bootstrap();
}
