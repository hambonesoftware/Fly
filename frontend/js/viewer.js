/*
 * Three.js viewer module.
 *
 * This module encapsulates the setup and rendering of a Three.js
 * scene used to display CAD meshes and flythrough paths.  It
 * exports functions to initialize the scene, add a mesh and add a
 * path.  Future phases may augment this file with interaction
 * features such as path editing and camera animation.
 */

import * as THREE from '../vendor/three.module.js';
import { OrbitControls } from '../vendor/OrbitControls.js';
import { createPathObjects, enablePathEditing } from './pathOverlay.js';

// Blend factor for camera look‑at behaviour in flythrough animations.
// 0.0 means the camera always looks at the centre of the path/model.
// 1.0 means the camera always looks ahead along the path (previous behaviour).
// Values in between blend between the centre and a small look‑ahead point.
const LOOKAHEAD_BLEND = 0.2;

// Scene, camera, renderer and controls are module‑level variables
let scene;
let camera;
let renderer;
let controls;
// Additional lights used in the enhanced lighting rig.  keyLight follows
// the camera to provide a head‑mounted illumination and hemiLight adds a
// gentle sky/ground fill.  These are initialised in initViewer().
let keyLight;
let hemiLight;

// Axes helper overlay for orientation.  The triad will be rendered
// in a small viewport overlay and oriented to match the current
// mesh.  See initViewer() for initialization.
let axesScene;
let axesCamera;
let axesHelper;

// Keep references to the current mesh and path objects so we can
// remove or update them when new data arrives
let currentMesh = null;
let currentPathLine = null;
let currentPathSpheres = [];

// Base outline line objects.  Multiple loops may be drawn for the
// perimeter preview, so we maintain an array of line objects.  When
// replacing the outline (e.g. when loading a new mesh or computing a
// new perimeter) the existing line objects are removed and disposed.
let baseOutlineLines = [];

// Store the raw perimeter loops (arrays of {x,y,z}) for animations
// When showBaseOutline is called, we populate this array with
// copies of the perimeter loops for later use (e.g. rolling-circle
// visualisations).  Each element is an array of point objects.
let baseOutlineLoops = [];

// Rolling circle visualisation state.  When a rolling-circle
// perimeter path is generated, we can animate a red circle along
// the path.  These variables track the circle mesh, the path
// points (centre positions), radius, speed and current parameter.
// When rollingCircleActive is false the circle is not animated.
let rollingCircleMesh = null;
let rollingCirclePathPoints = [];
let rollingCircleRadius = 1.0;
let rollingCircleSpeed = 0.05; // fraction of path per frame (approximate)
let rollingCircleT = 0.0;
let rollingCircleActive = false;

/**
 * Create or recreate the rolling circle mesh with the given
 * radius and orientation plane.  Removes any existing circle from
 * the scene.  The plane parameter determines which principal plane
 * the circle lies in: 'xy', 'xz' or 'yz'.
 *
 * @param {number} radius - Circle radius
 * @param {string} plane - One of 'xy', 'xz', 'yz'
 */
function createRollingCircleMesh(radius, plane) {
  // Remove existing circle mesh if present
  if (rollingCircleMesh) {
    scene.remove(rollingCircleMesh);
    if (rollingCircleMesh.geometry) rollingCircleMesh.geometry.dispose();
    if (rollingCircleMesh.material) rollingCircleMesh.material.dispose();
    rollingCircleMesh = null;
  }
  // Build a circle geometry.  Use sufficient segments for a smooth outline.
  const segments = 64;
  const geometry = new THREE.CircleGeometry(radius, segments);
  // Use a wireframe material so the viewer can see through the circle
  const material = new THREE.MeshBasicMaterial({ color: 0xff0000, wireframe: true });
  const mesh = new THREE.Mesh(geometry, material);
  // Orient the circle so its normal aligns with the chosen plane.  By
  // default, CircleGeometry is centred in the XY plane facing +Z.
  if (plane === 'xy') {
    // Default orientation: no rotation needed
  } else if (plane === 'xz') {
    // Rotate -90° about X so the normal points along +Y
    mesh.rotation.x = -Math.PI / 2;
  } else if (plane === 'yz') {
    // Rotate 90° about Y so the normal points along +X
    mesh.rotation.y = Math.PI / 2;
  }
  mesh.matrixAutoUpdate = true;
  scene.add(mesh);
  rollingCircleMesh = mesh;
}

/**
 * Start the rolling-circle perimeter animation.  The caller must
 * provide the path points (centre positions), circle radius and the
 * plane in which the circle should lie.  Optionally a speed can be
 * provided to adjust how fast the circle moves along the path.  The
 * path points are treated as a closed loop; the circle wraps
 * around when it reaches the end.
 *
 * @param {object} options - Animation options
 * @param {Array<{x:number,y:number,z:number}>} options.pathPoints - Centre path points
 * @param {number} options.radius - Circle radius
 * @param {string} options.plane - Plane ('xy', 'xz', 'yz') for circle orientation
 * @param {number} [options.speed] - Speed as fraction of path per frame (approx.)
 */
export function startPerimeterCircleAnimation(options = {}) {
  if (!scene) return;
  const { pathPoints, radius, plane, speed } = options;
  if (!pathPoints || !Array.isArray(pathPoints) || pathPoints.length < 2) {
    console.warn('startPerimeterCircleAnimation: invalid pathPoints');
    return;
  }
  // Copy path points into internal array to avoid external mutation
  rollingCirclePathPoints = pathPoints.map((p) => ({ x: p.x, y: p.y, z: p.z }));
  // Set radius and speed, using defaults when inputs are invalid
  rollingCircleRadius = typeof radius === 'number' && radius > 0 ? radius : 1.0;
  rollingCircleSpeed = typeof speed === 'number' && speed > 0 ? speed : 0.05;
  rollingCircleT = 0.0;
  rollingCircleActive = true;
  // Create the circle mesh oriented to the selected plane
  const planeVal = plane || 'xy';
  createRollingCircleMesh(rollingCircleRadius, planeVal);
}

/**
 * Initialize the Three.js viewer.
 *
 * This function sets up the scene, camera, renderer and orbit
 * controls.  It also starts the render loop.  It must be called
 * once when the application starts.  Subsequent calls will have no
 * effect.
 *
 * @param {HTMLCanvasElement} canvasElement - The canvas element to
 *   render into
 */
export function initViewer(canvasElement) {
  if (scene) {
    // Already initialized
    return;
  }
  // Create scene
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x101010);
  // Create camera
  const aspect = canvasElement.clientWidth / canvasElement.clientHeight;
  camera = new THREE.PerspectiveCamera(45, aspect, 0.1, 1000);
  // Position the camera far enough away to view unit cube by default
  camera.position.set(5, 5, 5);
  // Create renderer
  renderer = new THREE.WebGLRenderer({ canvas: canvasElement, antialias: true });
  renderer.setSize(canvasElement.clientWidth, canvasElement.clientHeight);
  renderer.setPixelRatio(window.devicePixelRatio);
  // Enable physically correct lighting and sRGB output encoding so lights
  // behave realistically and colours appear consistent across devices.
  renderer.physicallyCorrectLights = true;
  renderer.outputEncoding = THREE.sRGBEncoding;
  // Add lights
  // Ambient light provides low‑level fill so unlit surfaces are visible.
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.3);
  scene.add(ambientLight);
  // Directional key light simulates a strong light source.  It will
  // follow the camera position in the render loop to keep important
  // geometry illuminated.  Assign to module‑level variable so it can
  // be updated dynamically.
  keyLight = new THREE.DirectionalLight(0xffffff, 0.8);
  keyLight.position.set(10, 20, 10);
  keyLight.castShadow = false;
  scene.add(keyLight);
  // Hemisphere light adds subtle sky (top) and ground (bottom) colours.
  hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.4);
  hemiLight.position.set(0, 20, 0);
  scene.add(hemiLight);
  // Create controls
  controls = new OrbitControls(camera, canvasElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;

  // Initialize axes overlay scene, camera and helper.  The axes
  // helper draws three coloured lines representing the X, Y and Z
  // directions of the current mesh.  We keep it in a separate scene
  // so it can be rendered independently as an overlay.  The camera is
  // orthographic for a consistent appearance and uses a square
  // aspect ratio.
  axesScene = new THREE.Scene();
  axesCamera = new THREE.PerspectiveCamera(45, 1, 0.1, 10);
  axesHelper = new THREE.AxesHelper(0.75);
  axesScene.add(axesHelper);
  // Handle resize
  window.addEventListener('resize', () => {
    if (!renderer || !camera) return;
    const width = canvasElement.clientWidth;
    const height = canvasElement.clientHeight;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
  });
  // Render loop
  function animate() {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    // Make the key light follow the camera position to create a
    // head‑mounted lighting effect.  This keeps the scene well lit
    // during path playback.  Do this before rendering so the updated
    // position is applied each frame.
    if (keyLight && camera) {
      keyLight.position.copy(camera.position);
    }
    // Update rolling-circle position if active before rendering.  We
    // advance a parametric value along the path by a small increment
    // each frame and interpolate between successive points.  The path
    // is treated as a closed loop.
    if (rollingCircleActive && rollingCircleMesh && rollingCirclePathPoints.length > 1) {
      // Advance parameter along the path.  The speed is the fraction of the
      // full path traversed per frame (approximate).  Wrap around at 1.0.
      rollingCircleT += rollingCircleSpeed / rollingCirclePathPoints.length;
      if (rollingCircleT >= 1.0) {
        rollingCircleT -= 1.0;
      }
      const totalSegments = rollingCirclePathPoints.length;
      const fIndex = rollingCircleT * totalSegments;
      const i0 = Math.floor(fIndex) % totalSegments;
      const i1 = (i0 + 1) % totalSegments;
      const localT = fIndex - Math.floor(fIndex);
      const p0 = rollingCirclePathPoints[i0];
      const p1 = rollingCirclePathPoints[i1];
      const x = p0.x + (p1.x - p0.x) * localT;
      const y = p0.y + (p1.y - p0.y) * localT;
      const z = p0.z + (p1.z - p0.z) * localT;
      rollingCircleMesh.position.set(x, y, z);
    }

    // Determine canvas dimensions on each frame
    const width = renderer.domElement.clientWidth;
    const height = renderer.domElement.clientHeight;
    // Reset viewport and scissor to full canvas before rendering the main scene
    renderer.setViewport(0, 0, width, height);
    renderer.setScissor(0, 0, width, height);
    renderer.setScissorTest(false);
    renderer.render(scene, camera);
    // Render axes overlay.  We draw the axes helper in a small
    // viewport on top of the main scene.  Disable autoClear to
    // preserve the colour buffer from the main render but clear the
    // depth buffer so the axes are drawn on top.
    if (axesScene && axesCamera && axesHelper) {
      // Copy orientation from current mesh (if any) so the axes reflect
      // the model's rotation.
      if (currentMesh && currentMesh.quaternion) {
        axesHelper.quaternion.copy(currentMesh.quaternion);
      }
      const viewportSize = 80;
      const margin = 8;
      // Place overlay in the bottom‑left corner.  Adjust Y coordinate
      // because Three.js' viewport origin is bottom‑left.
      const overlayX = margin;
      const overlayY = margin;
      renderer.autoClear = false;
      renderer.clearDepth();
      renderer.setScissorTest(true);
      renderer.setViewport(overlayX, overlayY, viewportSize, viewportSize);
      renderer.setScissor(overlayX, overlayY, viewportSize, viewportSize);
      // Set up a simple camera to view the axes.  Position it along the
      // positive Z axis looking at the origin so the triad appears
      // centred.  We don't use controls here.
      axesCamera.position.set(0, 0, 2);
      axesCamera.lookAt(new THREE.Vector3(0, 0, 0));
      renderer.render(axesScene, axesCamera);
      renderer.setScissorTest(false);
      renderer.autoClear = true;
    }
  }
  animate();
}

/**
 * Add or replace the mesh in the scene based on the provided
 * MeshResponse.  Any previously displayed mesh will be removed.
 *
 * @param {object} meshResponse - The JSON response from the backend
 */
export function addMeshToScene(meshResponse) {
  if (!scene) {
    throw new Error('Viewer has not been initialized. Call initViewer() first.');
  }
  // Remove existing mesh if present
  if (currentMesh) {
    scene.remove(currentMesh);
    currentMesh.geometry.dispose();
    if (Array.isArray(currentMesh.material)) {
      currentMesh.material.forEach((m) => m.dispose());
    } else {
      currentMesh.material.dispose();
    }
    currentMesh = null;
  }
  // Remove any existing base outline loops when a new mesh is loaded
  if (baseOutlineLines.length) {
    baseOutlineLines.forEach((line) => {
      scene.remove(line);
      if (line.geometry) line.geometry.dispose();
      if (line.material) line.material.dispose();
    });
    baseOutlineLines = [];
  }
  // Build geometry from flat arrays
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(meshResponse.vertices, 3));
  geometry.setIndex(meshResponse.indices);
  // Normals may be missing or all zeros; compute if not provided
  if (meshResponse.normals && meshResponse.normals.length === meshResponse.vertices.length) {
    geometry.setAttribute('normal', new THREE.Float32BufferAttribute(meshResponse.normals, 3));
  } else {
    geometry.computeVertexNormals();
  }
  // Material: use a basic metal‑like material with some roughness
  const material = new THREE.MeshStandardMaterial({
    color: 0x0077ff,
    metalness: 0.2,
    roughness: 0.7,
  });
  currentMesh = new THREE.Mesh(geometry, material);
  scene.add(currentMesh);
  // Ensure the axes helper orientation matches the newly added mesh.  We
  // defer copying the quaternion until render time but set an initial
  // orientation here as well.
  if (axesHelper && currentMesh) {
    axesHelper.quaternion.copy(currentMesh.quaternion);
  }
  // Frame the mesh: compute bounding box and reposition camera/controls
  const bboxMin = meshResponse.bbox.min;
  const bboxMax = meshResponse.bbox.max;
  const size = new THREE.Vector3(
    bboxMax[0] - bboxMin[0],
    bboxMax[1] - bboxMin[1],
    bboxMax[2] - bboxMin[2]
  );
  const center = new THREE.Vector3(
    (bboxMin[0] + bboxMax[0]) / 2,
    (bboxMin[1] + bboxMax[1]) / 2,
    (bboxMin[2] + bboxMax[2]) / 2
  );
  // Set controls target to the center of the mesh
  controls.target.copy(center);
  // Position camera along a diagonal away from the center
  const maxDimension = Math.max(size.x, size.y, size.z);
  const distance = maxDimension * 2.5 + 1.0; // factor ensures object fits nicely
  // Adjust camera far plane if necessary to ensure large models are visible.
  const desiredFar = distance * 3;
  if (camera.far < desiredFar) {
    camera.far = desiredFar;
    camera.updateProjectionMatrix();
  }
  // Also adjust near plane proportionally to avoid depth precision issues for huge models
  const desiredNear = Math.max(0.1, distance * 0.001);
  if (camera.near !== desiredNear) {
    camera.near = desiredNear;
    camera.updateProjectionMatrix();
  }
  camera.position.set(center.x + distance, center.y + distance, center.z + distance);
  camera.lookAt(center);
  controls.update();
}

/**
 * Display the base perimeter outline in the scene.
 *
 * This function draws a closed loop around the model representing the
 * cross‑section (slice or outline) used to generate the perimeter
 * path.  It will replace any previously drawn outline.  The outline
 * is rendered as a thin yellow line loop.  If no points are
 * provided or fewer than two points are supplied, the outline is
 * removed.
 *
 * @param {Array<{x:number,y:number,z:number}>} points - Ordered list of 3D points
 */
export function showBaseOutline(points) {
  if (!scene) return;
  // Remove existing outline lines
  if (baseOutlineLines.length) {
    baseOutlineLines.forEach((line) => {
      scene.remove(line);
      if (line.geometry) line.geometry.dispose();
      if (line.material) line.material.dispose();
    });
    baseOutlineLines = [];
  }
  // Validate input; handle single loop or multiple loops.  If the first
  // element is an array (and not an object with x property) treat the
  // argument as an array of loops.  Otherwise treat it as a single loop.
  let loops = [];
  if (!points) {
    // Clear stored outline loops when no points are supplied
    baseOutlineLoops = [];
    return;
  }
  if (Array.isArray(points) && points.length > 0) {
    const firstEl = points[0];
    if (Array.isArray(firstEl)) {
      // Already an array of loops
      loops = points;
    } else if (typeof firstEl === 'object' && 'x' in firstEl) {
      // Single loop provided
      loops = [points];
    } else {
      // Unexpected input shape; do nothing
      return;
    }
  } else {
    return;
  }
  // Store loops for later (e.g. rolling-circle visualisation)
  baseOutlineLoops = loops.map((loop) =>
    loop.map((p) => ({ x: p.x, y: p.y, z: p.z }))
  );

  // Draw each loop as a LineLoop.  Use yellow colour to distinguish
  // perimeter loops from the path (which is red/green).
  loops.forEach((loop) => {
    if (!loop || loop.length < 2) return;
    // Ensure closed loop
    let pts = loop.slice();
    const firstPt = pts[0];
    const lastPt = pts[pts.length - 1];
    if (firstPt.x !== lastPt.x || firstPt.y !== lastPt.y || firstPt.z !== lastPt.z) {
      pts.push({ x: firstPt.x, y: firstPt.y, z: firstPt.z });
    }
    const positions = new Float32Array(pts.length * 3);
    for (let i = 0; i < pts.length; i++) {
      const p = pts[i];
      positions[i * 3 + 0] = p.x;
      positions[i * 3 + 1] = p.y;
      positions[i * 3 + 2] = p.z;
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const material = new THREE.LineBasicMaterial({
      color: 0xffff00,
      linewidth: 1,
    });
    const line = new THREE.LineLoop(geometry, material);
    baseOutlineLines.push(line);
    scene.add(line);
  });
}

/**
 * Add a flythrough path to the scene.  Removes any existing path.
 *
 * @param {Array<{x:number, y:number, z:number}>} points - Array of
 *   point objects returned by the backend
 */
export function addPathToScene(points) {
  // Remove in-scene path via path overlay and create a new one
  if (!scene) {
    throw new Error('Viewer has not been initialized. Call initViewer() first.');
  }
  // Delegate to path overlay module; showPath will handle creation and editing
  createPathObjects(points, scene);
  enablePathEditing(camera, renderer.domElement);
}

/**
 * Show a path in the scene and enable editing.  This is an alias
 * around the path overlay functions to mirror the API described in
 * Phase 3.  It replaces any existing path and sets up editing.
 *
 * @param {Array<{x:number,y:number,z:number}>} points - Path points
 */
export function showPath(points) {
  if (!scene) {
    throw new Error('Viewer has not been initialized. Call initViewer() first.');
  }
  createPathObjects(points, scene);
  enablePathEditing(camera, renderer.domElement);
}

/**
 * Update the existing path with new points.
 *
 * In this simple implementation we replace the path entirely.  More
 * advanced versions could update geometry in place.
 *
 * @param {Array<{x:number,y:number,z:number}>} points - Path points
 */
export function updatePath(points) {
  showPath(points);
}

/**
 * Play a flythrough animation along the given path.
 *
 * Moves the camera smoothly along the path over the specified
 * duration.  The camera looks at the centre of the model or the
 * next point along the path.  If no path is provided nothing
 * happens.
 *
 * @param {Array<{x:number,y:number,z:number}>} points - Path points
 * @param {number} durationSeconds - Duration of the animation
 */
export function playFlythrough(points, durationSeconds = 10) {
  if (!scene || !camera || !points || points.length === 0) return;
  const startTime = performance.now();
  const totalTime = durationSeconds * 1000;
  const numPoints = points.length;
  // Compute centre of path to use as lookAt target
  const centre = new THREE.Vector3();
  points.forEach((p) => {
    centre.x += p.x;
    centre.y += p.y;
    centre.z += p.z;
  });
  centre.divideScalar(numPoints);
  function animate() {
    const elapsed = performance.now() - startTime;
    const t = Math.min(1.0, elapsed / totalTime);
    // Parameter along path (wrap around)
    const f = t * numPoints;
    const indexA = Math.floor(f) % numPoints;
    const indexB = (indexA + 1) % numPoints;
    const localT = f - Math.floor(f);
    // Linear interpolation between points A and B
    const pA = points[indexA];
    const pB = points[indexB];
    const x = pA.x + (pB.x - pA.x) * localT;
    const y = pA.y + (pB.y - pA.y) * localT;
    const z = pA.z + (pB.z - pA.z) * localT;
    camera.position.set(x, y, z);
    // Blend between the path centre and a small look‑ahead point.  The
    // centre keeps the model anchored in view, while the ahead
    // vector provides some directional context.  The blend factor is
    // controlled by LOOKAHEAD_BLEND (0.0 = centre only, 1.0 = ahead only).
    const nextIndex = (indexA + 2) % numPoints;
    const ahead = new THREE.Vector3(
      points[nextIndex].x,
      points[nextIndex].y,
      points[nextIndex].z
    );
    // Compute blended target
    const target = centre.clone().lerp(ahead, LOOKAHEAD_BLEND);
    camera.lookAt(target);
    // Ensure controls are updated if damping is enabled
    if (controls) controls.update();
    renderer.render(scene, camera);
    if (t < 1.0) {
      requestAnimationFrame(animate);
    }
  }
  animate();
}
