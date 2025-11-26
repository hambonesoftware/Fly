/*
 * Path Overlay Module
 *
 * Provides functions to create visual representations of a path and
 * enable interactive editing by dragging control points in 3D space.
 * The module stores the current path points and updates the geometry
 * whenever a control point is moved.  It uses a plane aligned with
 * the camera view to constrain dragging so that movement feels
 * intuitive.  Edited points can be retrieved via
 * `getCurrentPathPoints()` for validation or export.
 */

import * as THREE from '../vendor/three.module.js';

// Module state
let pathPoints = [];
let line = null;
let spheres = [];
let scene = null;
let camera = null;
let domElement = null;
let raycaster = new THREE.Raycaster();
let mouse = new THREE.Vector2();
let plane = new THREE.Plane();
let planeIntersect = new THREE.Vector3();
let offset = new THREE.Vector3();
let selectedSphere = null;
let editingEnabled = false;

/**
 * Create or replace path objects in the scene.
 *
 * This function removes any existing path visualization and constructs
 * a new line and spheres from the supplied points.  The points are
 * stored internally as a deep copy to allow editing.
 *
 * @param {Array<{x:number,y:number,z:number}>} points - Path points
 * @param {THREE.Scene} targetScene - Scene to which objects should be added
 */
export function createPathObjects(points, targetScene) {
  // Remove existing objects
  if (line) {
    targetScene.remove(line);
    line.geometry.dispose();
    line.material.dispose();
    line = null;
  }
  if (spheres.length) {
    spheres.forEach((s) => {
      targetScene.remove(s);
      s.geometry.dispose();
      s.material.dispose();
    });
    spheres = [];
  }
  // Deep copy points to avoid mutating original array
  pathPoints = points.map((p) => ({ x: p.x, y: p.y, z: p.z }));
  // Create line geometry
  const vertices = [];
  pathPoints.forEach((p) => {
    vertices.push(p.x, p.y, p.z);
  });
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  const material = new THREE.LineBasicMaterial({ color: 0xff5555 });
  line = new THREE.Line(geometry, material);
  targetScene.add(line);
  // Create spheres
  const sphereGeom = new THREE.SphereGeometry(0.05, 8, 8);
  const sphereMat = new THREE.MeshStandardMaterial({ color: 0x55ff55 });
  pathPoints.forEach((p, idx) => {
    const sphere = new THREE.Mesh(sphereGeom, sphereMat.clone());
    sphere.position.set(p.x, p.y, p.z);
    sphere.userData.index = idx;
    spheres.push(sphere);
    targetScene.add(sphere);
  });
  // Store scene reference for later removal
  scene = targetScene;
}

/**
 * Enable interactive editing of the path.
 *
 * Sets up event listeners on the provided DOM element to detect
 * pointer interactions with the control spheres.  Once enabled,
 * control points can be selected and dragged to new positions.
 *
 * @param {THREE.Camera} cam - The active camera used for raycasting
 * @param {HTMLElement} domEl - DOM element receiving pointer events (typically renderer.domElement)
 */
export function enablePathEditing(cam, domEl) {
  camera = cam;
  domElement = domEl;
  if (editingEnabled) return;
  editingEnabled = true;
  domElement.addEventListener('pointerdown', onPointerDown);
  domElement.addEventListener('pointermove', onPointerMove);
  domElement.addEventListener('pointerup', onPointerUp);
  domElement.addEventListener('pointerleave', onPointerUp);
}

/**
 * Get the current path points.
 *
 * Returns a deep copy of the internal path points array.  Use this
 * function to send edited paths back to the backend for validation
 * or export.
 *
 * @returns {Array<{x:number,y:number,z:number}>} Path points
 */
export function getCurrentPathPoints() {
  return pathPoints.map((p) => ({ x: p.x, y: p.y, z: p.z }));
}

function getMouse(event) {
  const rect = domElement.getBoundingClientRect();
  mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
}

function onPointerDown(event) {
  if (!camera || !domElement) return;
  getMouse(event);
  raycaster.setFromCamera(mouse, camera);
  const intersects = raycaster.intersectObjects(spheres, false);
  if (intersects.length > 0) {
    selectedSphere = intersects[0].object;
    // Set a plane for movement aligned with camera direction and through the point
    plane.setFromNormalAndCoplanarPoint(camera.getWorldDirection(plane.normal), selectedSphere.position);
    // Compute offset between intersection point and sphere position
    if (raycaster.ray.intersectPlane(plane, planeIntersect)) {
      offset.copy(selectedSphere.position).sub(planeIntersect);
    }
    domElement.style.cursor = 'move';
  }
}

function onPointerMove(event) {
  if (!selectedSphere || !camera || !domElement) return;
  getMouse(event);
  raycaster.setFromCamera(mouse, camera);
  if (raycaster.ray.intersectPlane(plane, planeIntersect)) {
    const newPos = planeIntersect.clone().add(offset);
    selectedSphere.position.copy(newPos);
    const idx = selectedSphere.userData.index;
    // Update pathPoints
    pathPoints[idx].x = newPos.x;
    pathPoints[idx].y = newPos.y;
    pathPoints[idx].z = newPos.z;
    // Update line geometry positions attribute
    const positionAttr = line.geometry.getAttribute('position');
    positionAttr.setXYZ(idx, newPos.x, newPos.y, newPos.z);
    positionAttr.needsUpdate = true;
    // If closed loop and editing first/last point, update both ends
    // (optional) – not implemented for simplicity
  }
}

function onPointerUp() {
  if (selectedSphere) {
    selectedSphere = null;
    domElement.style.cursor = 'auto';
  }
}
