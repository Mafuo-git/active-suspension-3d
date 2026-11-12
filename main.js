// Active Suspension 3D – interactive model of a servo-actuated rocker suspension
// with a ball lock pin mode selector (Active mode / Manual mode).
//
// Units: 1 scene unit = 10 mm. The model is planar (x-y plane) with layers stacked along z:
//   z = -1.4 chassis side panel, z = 0 chassis bracket, z = 1 coilover / rocker plane, z = 2 servo arm.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

// ---------------------------------------------------------------------------
// Geometry constants
// ---------------------------------------------------------------------------
const UNIT_MM = 10;
const Z_EYE = 1;                                       // coilover / rocker plane
const Z_ARM = 2;                                       // servo arm plane
const SERVO_PIVOT = new THREE.Vector2(-5, 0);          // servo output shaft axis
const ARM_LENGTH = 5;                                  // servo arm: pivot to pin hole
const ROCKER_PIVOT = new THREE.Vector2(1.2, 7.6);
const ROCKER_COILOVER = new THREE.Vector2(-2.4, -1.6); // coilover mount, relative to rocker pivot
const ROCKER_PUSHROD = new THREE.Vector2(2.6, 1.2);    // pushrod mount, relative to rocker pivot
const UPRIGHT_X = 9.6;
const PUSHROD_BOTTOM_X = 8.8;
const PUSHROD_BOTTOM_Y0 = -0.6;                        // pushrod lower end at zero wheel travel
const PIN_Z = { active: 1.45, manual: 0.45 };          // ball lock pin centre position along z
const SERVO_RANGE_DEG = 12;
const TRAVEL_RANGE_MM = { min: -10, max: 15 };

// ---------------------------------------------------------------------------
// Planar kinematics helpers
// ---------------------------------------------------------------------------
const rotate = (v, a) => new THREE.Vector2(v.x * Math.cos(a) - v.y * Math.sin(a), v.x * Math.sin(a) + v.y * Math.cos(a));
const rockerPoint = (rel, phi) => ROCKER_PIVOT.clone().add(rotate(rel, phi));
const armHole = (theta) => new THREE.Vector2(SERVO_PIVOT.x + ARM_LENGTH * Math.cos(theta), SERVO_PIVOT.y + ARM_LENGTH * Math.sin(theta));
const BRACKET_HOLE = armHole(0);                       // bracket hole is coaxial with the arm hole at neutral

const PUSHROD_LENGTH = rockerPoint(ROCKER_PUSHROD, 0).distanceTo(new THREE.Vector2(PUSHROD_BOTTOM_X, PUSHROD_BOTTOM_Y0));
const SPRING_FREE = rockerPoint(ROCKER_COILOVER, 0).distanceTo(BRACKET_HOLE); // coilover length at neutral

// Lower pushrod end height for a given rocker angle (the end slides vertically with the upright).
function pushrodBottomY(phi) {
  const r = rockerPoint(ROCKER_PUSHROD, phi);
  const dx = r.x - PUSHROD_BOTTOM_X;
  return r.y - Math.sqrt(Math.max(PUSHROD_LENGTH ** 2 - dx * dx, 0));
}

// Generic monotonic bisection solver on the rocker angle.
function solvePhi(f, target, lo = -0.8, hi = 0.8) {
  const increasing = f(hi) > f(lo);
  for (let i = 0; i < 40; i++) {
    const mid = (lo + hi) / 2;
    if ((f(mid) < target) === increasing) lo = mid; else hi = mid;
  }
  return (lo + hi) / 2;
}

// Manual mode: wheel travel imposed by hand, coilover anchor locked to the chassis.
const phiFromTravel = (travel) => solvePhi(pushrodBottomY, PUSHROD_BOTTOM_Y0 + travel);
// Active mode: coilover length held at its static length, servo moves the anchor.
const phiFromAnchor = (anchor) => solvePhi((p) => rockerPoint(ROCKER_COILOVER, p).distanceTo(anchor), SPRING_FREE);

// ---------------------------------------------------------------------------
// Renderer, scene, camera
// ---------------------------------------------------------------------------
const container = document.getElementById('viewport');
const isSmallScreen = () => window.matchMedia('(max-width: 720px)').matches;

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'low-power' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, isSmallScreen() ? 1.5 : 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
container.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;

const camera = new THREE.PerspectiveCamera(38, 1, 0.1, 300);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 4;
controls.maxDistance = 90;

const keyLight = new THREE.DirectionalLight(0xffffff, 1.4);
keyLight.position.set(8, 14, 12);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(1024, 1024);
Object.assign(keyLight.shadow.camera, { left: -14, right: 14, top: 14, bottom: -10, near: 1, far: 50 });
keyLight.shadow.bias = -0.0008;
scene.add(keyLight, new THREE.HemisphereLight(0xffffff, 0x404048, 0.35));

// ---------------------------------------------------------------------------
// Materials
// ---------------------------------------------------------------------------
const mat = (color, opts = {}) => new THREE.MeshStandardMaterial({ color, metalness: 0.2, roughness: 0.5, ...opts });
const M = {
  carbon: mat(0x2a2a2e, { roughness: 0.65 }),
  alu: mat(0xb9b8b2, { metalness: 0.85, roughness: 0.32 }),
  steel: mat(0xdcdcd6, { metalness: 0.95, roughness: 0.2 }),
  servo: mat(0x534ab7, { roughness: 0.45 }),
  servoArm: mat(0x7f77dd, { metalness: 0.6, roughness: 0.3 }),
  coilover: mat(0x1d9e75, { metalness: 0.5, roughness: 0.3 }),
  spring: mat(0x9fe1cb, { metalness: 0.4, roughness: 0.35 }),
  pin: mat(0xef9f27, { metalness: 0.75, roughness: 0.25 }),
  rubber: mat(0x1b1b1d, { roughness: 0.9 }),
  rim: mat(0x8a8a86, { metalness: 0.9, roughness: 0.25 }),
  switchOk: mat(0x639922, { emissive: 0x2c4a0c, emissiveIntensity: 0.6 }),
  switchOff: mat(0xe24b4a, { emissive: 0x7a1212, emissiveIntensity: 0.8 }),
};

// ---------------------------------------------------------------------------
// Geometry builders
// ---------------------------------------------------------------------------
function mesh(geometry, material, x = 0, y = 0, z = 0, parent = scene) {
  const m = new THREE.Mesh(geometry, material);
  m.position.set(x, y, z);
  m.castShadow = true;
  m.receiveShadow = true;
  parent.add(m);
  return m;
}

// Extruded plate from a 2D outline, with optional circular holes, centred on its thickness.
function extrude(shape, depth, holes = []) {
  for (const [hx, hy, r] of holes) {
    const p = new THREE.Path();
    p.absarc(hx, hy, r, 0, Math.PI * 2, true);
    shape.holes.push(p);
  }
  const g = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: true, bevelThickness: 0.03, bevelSize: 0.03, bevelSegments: 2, curveSegments: 32 });
  g.translate(0, 0, -depth / 2);
  return g;
}

function roundedRect(w, h, r) {
  const s = new THREE.Shape();
  s.moveTo(-w / 2 + r, -h / 2);
  s.lineTo(w / 2 - r, -h / 2); s.quadraticCurveTo(w / 2, -h / 2, w / 2, -h / 2 + r);
  s.lineTo(w / 2, h / 2 - r); s.quadraticCurveTo(w / 2, h / 2, w / 2 - r, h / 2);
  s.lineTo(-w / 2 + r, h / 2); s.quadraticCurveTo(-w / 2, h / 2, -w / 2, h / 2 - r);
  s.lineTo(-w / 2, -h / 2 + r); s.quadraticCurveTo(-w / 2, -h / 2, -w / 2 + r, -h / 2);
  return s;
}

// Capsule outline between (0,0) and (len,0).
function capsule(len, r) {
  const s = new THREE.Shape();
  s.moveTo(0, -r);
  s.lineTo(len, -r);
  s.absarc(len, 0, r, -Math.PI / 2, Math.PI / 2, false);
  s.lineTo(0, r);
  s.absarc(0, 0, r, Math.PI / 2, Math.PI * 1.5, false);
  return s;
}

// Convex outline wrapping a set of circles [x, y, r] (used for the rocker plate).
function hullAroundCircles(circles, samples = 24) {
  const pts = [];
  for (const [x, y, r] of circles) {
    for (let i = 0; i < samples; i++) {
      const a = (i / samples) * Math.PI * 2;
      pts.push([x + r * Math.cos(a), y + r * Math.sin(a)]);
    }
  }
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [], upper = [];
  for (const pt of pts) { while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], pt) <= 0) lower.pop(); lower.push(pt); }
  for (const pt of [...pts].reverse()) { while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], pt) <= 0) upper.pop(); upper.push(pt); }
  const hull = lower.slice(0, -1).concat(upper.slice(0, -1));
  const s = new THREE.Shape();
  hull.forEach(([x, y], i) => (i ? s.lineTo(x, y) : s.moveTo(x, y)));
  s.closePath();
  return s;
}

const cylZ = (r, len, seg = 24) => new THREE.CylinderGeometry(r, r, len, seg).rotateX(Math.PI / 2);

// Places a unit-height Y-aligned mesh between two points.
function stretchBetween(obj, a, b) {
  const dir = new THREE.Vector3().subVectors(b, a);
  obj.position.copy(a).addScaledVector(dir, 0.5);
  obj.scale.set(1, dir.length(), 1);
  obj.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
}

// ---------------------------------------------------------------------------
// Static parts: chassis panel, bracket, micro-switch, servo
// ---------------------------------------------------------------------------
mesh(extrude(roundedRect(16, 12, 0.8), 0.3, [[5.5, 1.5, 1.4], [-5.5, 3.5, 1.1]]), M.carbon, 1.5, 4.2, -1.4);

// Shadow catcher under the assembly
const ground = new THREE.Mesh(new THREE.PlaneGeometry(60, 60), new THREE.ShadowMaterial({ opacity: 0.25 }));
ground.rotation.x = -Math.PI / 2;
ground.position.y = -2.6;
ground.receiveShadow = true;
scene.add(ground);

// Chassis bracket (fixed) with the manual-mode pin hole, on four standoffs
mesh(extrude(roundedRect(3, 2.2, 0.3), 0.3, [[0, 0, 0.27]]), M.alu, BRACKET_HOLE.x, BRACKET_HOLE.y, 0);
for (const [dx, dy] of [[-1.15, -0.8], [1.15, -0.8], [-1.15, 0.8], [1.15, 0.8]]) {
  mesh(cylZ(0.14, 1.1, 16), M.steel, BRACKET_HOLE.x + dx, BRACKET_HOLE.y + dy, -0.7);
}

// Micro-switch behind the bracket hole: pressed by the pin tip in manual mode
const switchBody = mesh(new THREE.BoxGeometry(0.9, 0.7, 0.45), M.switchOk, BRACKET_HOLE.x, BRACKET_HOLE.y, -1.02);
const plunger = mesh(cylZ(0.11, 0.3, 12), M.steel, BRACKET_HOLE.x, BRACKET_HOLE.y, -0.66);

// Servo body with mounting ears and a stand bolted to the chassis panel
const servoGroup = new THREE.Group();
scene.add(servoGroup);
mesh(extrude(roundedRect(2.2, 2.6, 0.15), 1.6), M.servo, SERVO_PIVOT.x, SERVO_PIVOT.y + 0.35, 0.95, servoGroup);
mesh(new THREE.BoxGeometry(3.4, 0.5, 0.25), M.servo, SERVO_PIVOT.x, SERVO_PIVOT.y + 0.35, 1.45, servoGroup);
mesh(new THREE.BoxGeometry(1.6, 2.0, 1.1), M.alu, SERVO_PIVOT.x, SERVO_PIVOT.y + 0.35, -0.7, servoGroup);
mesh(cylZ(0.45, 0.35, 32), M.alu, SERVO_PIVOT.x, SERVO_PIVOT.y, 1.9, servoGroup);

// ---------------------------------------------------------------------------
// Moving parts
// ---------------------------------------------------------------------------
// Servo arm (rotates about the servo axis), with the active-mode pin hole at its tip
const servoArm = new THREE.Group();
servoArm.position.set(SERVO_PIVOT.x, SERVO_PIVOT.y, Z_ARM);
scene.add(servoArm);
mesh(extrude(capsule(ARM_LENGTH, 0.6), 0.3, [[ARM_LENGTH, 0, 0.27]]), M.servoArm, 0, 0, 0, servoArm);
mesh(cylZ(0.5, 0.5, 32), M.servoArm, 0, 0, 0, servoArm);

// Rocker (rotates about its pivot), with pivot bolt through the chassis
const rocker = new THREE.Group();
rocker.position.set(ROCKER_PIVOT.x, ROCKER_PIVOT.y, Z_EYE);
scene.add(rocker);
{
  const c = ROCKER_COILOVER, p = ROCKER_PUSHROD;
  const outline = hullAroundCircles([[0, 0, 0.6], [c.x, c.y, 0.5], [p.x, p.y, 0.45]]);
  mesh(extrude(outline, 0.3, [[0, 0, 0.22], [c.x, c.y, 0.18], [p.x, p.y, 0.15]]), M.alu, 0, 0, 0, rocker);
}
mesh(cylZ(0.2, 2.9, 16), M.steel, ROCKER_PIVOT.x, ROCKER_PIVOT.y, -0.15);

// Coilover: damper body near the anchor eye, shaft, perches and spring, scaled to its length
const coilover = new THREE.Group();
scene.add(coilover);
const eyeGeo = new THREE.TorusGeometry(0.4, 0.14, 12, 32);
const eyeLow = mesh(eyeGeo, M.coilover, 0, 0, 0, coilover);
const eyeHigh = mesh(eyeGeo, M.coilover, 0, 0, 0, coilover);
const damperBody = mesh(new THREE.CylinderGeometry(0.38, 0.38, 1, 32), M.coilover, 0, 0, 0, coilover);
const damperShaft = mesh(new THREE.CylinderGeometry(0.13, 0.13, 1, 16), M.steel, 0, 0, 0, coilover);
const perchLow = mesh(new THREE.CylinderGeometry(0.72, 0.72, 0.12, 32), M.coilover, 0, 0, 0, coilover);
const perchHigh = mesh(new THREE.CylinderGeometry(0.72, 0.72, 0.12, 32), M.coilover, 0, 0, 0, coilover);
// Helical spring rebuilt for its actual length, so the wire keeps a round section when compressed.
function springGeometry(length, turns = 8, radius = 0.6, wire = 0.075) {
  const pts = [];
  for (let i = 0; i <= 320; i++) {
    const t = i / 320;
    pts.push(new THREE.Vector3(radius * Math.cos(t * Math.PI * 2 * turns), t * length, radius * Math.sin(t * Math.PI * 2 * turns)));
  }
  return new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 400, wire, 8, false);
}
let springLength = 4;
const spring = mesh(springGeometry(springLength), M.spring, 0, 0, 0, coilover);

// Pushrod with rod-end balls
const pushrod = mesh(new THREE.CylinderGeometry(0.11, 0.11, 1, 12), M.steel);
const rodEndTop = mesh(new THREE.SphereGeometry(0.2, 16, 16), M.steel);
const rodEndBottom = mesh(new THREE.SphereGeometry(0.2, 16, 16), M.steel);

// Upright, wishbones and wheel (move with wheel travel)
const upright = mesh(new THREE.BoxGeometry(0.7, 3.4, 0.6), M.alu);
const wishboneUpper = mesh(new THREE.CylinderGeometry(0.1, 0.1, 1, 12), M.steel);
const wishboneLower = mesh(new THREE.CylinderGeometry(0.1, 0.1, 1, 12), M.steel);
const WISHBONE_INBOARD = { upper: new THREE.Vector3(4.2, 2.4, Z_EYE), lower: new THREE.Vector3(4.2, -1.1, Z_EYE) };
for (const p of Object.values(WISHBONE_INBOARD)) mesh(new THREE.SphereGeometry(0.18, 16, 16), M.steel, p.x, p.y, p.z);
const wheel = new THREE.Group();
scene.add(wheel);
mesh(new THREE.TorusGeometry(2.35, 0.85, 20, 64), M.rubber, 0, 0, 0, wheel);
mesh(cylZ(1.75, 1.2, 48), M.rim, 0, 0, 0, wheel);
for (let i = 0; i < 5; i++) {
  const arm = new THREE.Group();
  arm.rotation.z = (i / 5) * Math.PI * 2;
  wheel.add(arm);
  mesh(new THREE.BoxGeometry(0.35, 1.4, 0.25), M.alu, 0, 0.9, 0.65, arm);
}
mesh(cylZ(0.4, 1.4, 24), M.steel, 0, 0, 0.1, wheel);

// Ball lock pin: shaft, locking balls, head and pull ring
const pin = new THREE.Group();
scene.add(pin);
mesh(cylZ(0.25, 2.1, 32), M.pin, 0, 0, 0, pin);
mesh(cylZ(0.5, 0.3, 32), M.pin, 0, 0, 1.2, pin);
mesh(cylZ(0.2, 0.15, 24), M.steel, 0, 0, 1.42, pin);
const ring = mesh(new THREE.TorusGeometry(0.45, 0.07, 12, 40), M.pin, 0, 0, 1.75, pin);
ring.rotation.y = Math.PI / 2;
for (const x of [0.23, -0.23]) mesh(new THREE.SphereGeometry(0.09, 12, 12), M.steel, x, 0, -0.9, pin);

// ---------------------------------------------------------------------------
// Labels (HTML elements projected from 3D anchors)
// ---------------------------------------------------------------------------
const labels = [];
function addLabel(text, anchor, minor = false) {
  const el = document.createElement('div');
  el.className = 'label' + (minor ? ' minor' : '');
  el.textContent = text;
  document.body.appendChild(el);
  labels.push({ el, anchor });
}

// ---------------------------------------------------------------------------
// State and kinematics update
// ---------------------------------------------------------------------------
const state = {
  mode: 'active',
  servoTarget: 0,   // rad, user command
  travelTarget: 0,  // units, user command in manual mode
  theta: 0,         // current servo angle
  travel: 0,        // current wheel travel
  pinZ: PIN_Z.active,
  demo: false,
  demoClock: 0,
  labels: true,
  xray: false,
};

const v3 = (v2, z) => new THREE.Vector3(v2.x, v2.y, z);
const live = { anchor: BRACKET_HOLE.clone(), springLength: SPRING_FREE, phi: 0 };

function updateAssembly() {
  // The pin links the coilover eye to the servo arm while it is engaged in the arm (z > 0.95)
  const linked = state.pinZ > 0.95;
  const anchor = linked ? armHole(state.theta) : BRACKET_HOLE;
  servoArm.rotation.z = state.theta;

  // Rocker angle: imposed by the servo anchor (linked) or by the wheel pushed by hand (unlinked)
  let phi;
  if (linked) {
    phi = phiFromAnchor(anchor);
    state.travel = pushrodBottomY(phi) - PUSHROD_BOTTOM_Y0;
  } else {
    phi = phiFromTravel(state.travel);
  }
  rocker.rotation.z = phi;

  // Coilover between anchor eye and rocker eye
  const top = rockerPoint(ROCKER_COILOVER, phi);
  const length = top.distanceTo(anchor);
  coilover.position.set(anchor.x, anchor.y, Z_EYE);
  coilover.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(top.x - anchor.x, top.y - anchor.y, 0).normalize());
  eyeLow.position.y = 0;
  eyeHigh.position.y = length;
  eyeLow.rotation.set(0, 0, 0); eyeHigh.rotation.set(0, 0, 0);
  const bodyLen = 2.6;
  damperBody.scale.y = bodyLen; damperBody.position.y = 0.55 + bodyLen / 2;
  const shaftStart = 0.55 + bodyLen, shaftEnd = length - 0.5;
  damperShaft.scale.y = Math.max(shaftEnd - shaftStart, 0.05); damperShaft.position.y = (shaftStart + shaftEnd) / 2;
  perchLow.position.y = 1.4; perchHigh.position.y = length - 0.9;
  spring.position.y = 1.46;
  const newSpringLength = Math.max(length - 0.9 - 1.46 - 0.06, 0.1);
  if (Math.abs(newSpringLength - springLength) > 0.005) {
    spring.geometry.dispose();
    spring.geometry = springGeometry(newSpringLength);
    springLength = newSpringLength;
  }

  // Pushrod, upright, wishbones, wheel
  const rTop = v3(rockerPoint(ROCKER_PUSHROD, phi), Z_EYE);
  const rBottom = new THREE.Vector3(PUSHROD_BOTTOM_X, PUSHROD_BOTTOM_Y0 + state.travel, Z_EYE);
  stretchBetween(pushrod, rTop, rBottom);
  rodEndTop.position.copy(rTop); rodEndBottom.position.copy(rBottom);
  const uprightY = 0.65 + state.travel;
  upright.position.set(UPRIGHT_X, uprightY, Z_EYE + 0.3);
  stretchBetween(wishboneUpper, WISHBONE_INBOARD.upper, new THREE.Vector3(UPRIGHT_X - 0.3, uprightY + 1.5, Z_EYE));
  stretchBetween(wishboneLower, WISHBONE_INBOARD.lower, new THREE.Vector3(UPRIGHT_X - 0.3, uprightY - 1.5, Z_EYE));
  wheel.position.set(UPRIGHT_X, uprightY, 3.4);

  // Pin follows the coilover eye, sliding along z between the two positions
  pin.position.set(anchor.x, anchor.y, state.pinZ);
  const pressed = state.pinZ < 0.6;
  switchBody.material = pressed ? M.switchOff : M.switchOk;
  plunger.position.z = pressed ? -0.74 : -0.66;

  live.anchor = anchor; live.springLength = length; live.phi = phi;
}

// Label anchors (evaluated each frame)
addLabel('Chassis', () => new THREE.Vector3(-5.6, 9.4, -1.4), true);
addLabel('Chassis bracket', () => new THREE.Vector3(BRACKET_HOLE.x + 1.6, BRACKET_HOLE.y - 1.5, 0));
addLabel('Micro-switch', () => new THREE.Vector3(BRACKET_HOLE.x - 1.1, BRACKET_HOLE.y - 1.0, -1.0), true);
addLabel('Servo', () => new THREE.Vector3(SERVO_PIVOT.x, SERVO_PIVOT.y + 2.2, 1.0));
addLabel('Servo arm', () => { const h = armHole(state.theta); return new THREE.Vector3((h.x + SERVO_PIVOT.x) / 2, (h.y + SERVO_PIVOT.y) / 2 - 1.0, Z_ARM); });
addLabel('Coilover', () => { const t = rockerPoint(ROCKER_COILOVER, live.phi); return new THREE.Vector3((t.x + live.anchor.x) / 2 - 1.3, (t.y + live.anchor.y) / 2, Z_EYE); });
addLabel('Ball lock pin', () => new THREE.Vector3(live.anchor.x - 0.6, live.anchor.y + 1.0, state.pinZ + 1.5));
addLabel('Rocker', () => new THREE.Vector3(ROCKER_PIVOT.x + 0.6, ROCKER_PIVOT.y + 1.4, Z_EYE));
addLabel('Pushrod', () => { const t = rockerPoint(ROCKER_PUSHROD, live.phi); return new THREE.Vector3((t.x + PUSHROD_BOTTOM_X) / 2 + 1.0, (t.y + PUSHROD_BOTTOM_Y0 + state.travel) / 2, Z_EYE); }, true);
addLabel('Wheel', () => new THREE.Vector3(UPRIGHT_X, 0.65 + state.travel - 3.6, 3.4), true);

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------
const $ = (id) => document.getElementById(id);
const ui = {
  slider: $('slider'), sliderLabel: $('sliderLabel'), sliderValue: $('sliderValue'),
  btnActive: $('btnActive'), btnManual: $('btnManual'), btnDemo: $('btnDemo'),
  btnLabels: $('btnLabels'), btnXray: $('btnXray'), hint: $('hint'),
};

function setReadout(id, text, cls = '') { const el = $(id); el.textContent = text; el.className = cls; }

function refreshUI() {
  const manual = state.mode === 'manual';
  ui.btnActive.classList.toggle('on', !manual);
  ui.btnManual.classList.toggle('on', manual);
  ui.btnDemo.textContent = state.demo ? '❚❚ Stop demo' : '▶ Play demo';
  ui.btnDemo.classList.toggle('on', state.demo);
  if (manual) {
    ui.sliderLabel.textContent = 'Wheel travel (push by hand)';
    Object.assign(ui.slider, { min: TRAVEL_RANGE_MM.min, max: TRAVEL_RANGE_MM.max, step: 0.5 });
    ui.slider.value = state.travelTarget * UNIT_MM;
    ui.sliderValue.textContent = `${(state.travelTarget * UNIT_MM).toFixed(1)} mm`;
    ui.hint.textContent = 'Manual mode: the servo arm returns to neutral, then the pin slides back into the chassis bracket and presses the micro-switch (servo power OFF). The suspension is passive and safe to move by hand.';
  } else {
    ui.sliderLabel.textContent = 'Servo angle';
    Object.assign(ui.slider, { min: -SERVO_RANGE_DEG, max: SERVO_RANGE_DEG, step: 0.5 });
    ui.slider.value = THREE.MathUtils.radToDeg(state.servoTarget);
    ui.sliderValue.textContent = `${THREE.MathUtils.radToDeg(state.servoTarget).toFixed(1)}°`;
    ui.hint.textContent = 'Active mode: the pin links the coilover eye to the servo arm. Rotating the servo moves the coilover anchor, which changes ride height and attitude (pitch / roll).';
  }
}

function refreshReadouts() {
  const linked = state.pinZ > 0.95;
  const pressed = state.pinZ < 0.6;
  setReadout('rLink', linked ? 'Connected' : 'Disconnected', linked ? 'ok' : 'ko');
  setReadout('rAnchor', linked ? 'Servo arm' : 'Chassis bracket');
  setReadout('rPower', pressed ? 'OFF' : 'ON', pressed ? 'ko' : 'ok');
  setReadout('rTravel', `${(state.travel * UNIT_MM).toFixed(1)} mm`);
  setReadout('rSpring', `${(live.springLength * UNIT_MM).toFixed(1)} mm`);
}

function setMode(mode) {
  state.mode = mode;
  if (mode === 'manual') state.travelTarget = 0;
  refreshUI();
  requestRender();
}

ui.btnActive.addEventListener('click', () => setMode('active'));
ui.btnManual.addEventListener('click', () => setMode('manual'));
ui.slider.addEventListener('input', () => {
  const v = parseFloat(ui.slider.value);
  state.demo = false;
  if (state.mode === 'manual') state.travelTarget = v / UNIT_MM;
  else state.servoTarget = THREE.MathUtils.degToRad(v);
  refreshUI();
  requestRender();
});
ui.btnDemo.addEventListener('click', () => { state.demo = !state.demo; state.demoClock = 0; refreshUI(); requestRender(); });
ui.btnLabels.addEventListener('click', () => {
  state.labels = !state.labels;
  ui.btnLabels.classList.toggle('on', state.labels);
  requestRender();
});
ui.btnXray.addEventListener('click', () => {
  state.xray = !state.xray;
  ui.btnXray.classList.toggle('on', state.xray);
  for (const [m, o] of [[M.carbon, 0.22], [M.servo, 0.35]]) {
    m.transparent = state.xray;
    m.opacity = state.xray ? o : 1;
    m.depthWrite = !state.xray;
    m.needsUpdate = true;
  }
  requestRender();
});

// Collapsible panel (bottom sheet on phones)
const panel = $('panel');
$('panelToggle').addEventListener('click', () => {
  const collapsed = panel.classList.toggle('collapsed');
  $('panelToggle').setAttribute('aria-expanded', String(!collapsed));
  $('panelToggleText').textContent = collapsed ? 'Show controls' : 'Hide controls';
  updateViewOffset();
  requestRender();
});
if (isSmallScreen()) {
  panel.classList.add('collapsed');
  $('panelToggleText').textContent = 'Show controls';
}

// Camera presets with smooth transitions.
// Iso / Front / Top frame the whole assembly automatically for the visible area (screen minus control panel).
const FRAME = { center: new THREE.Vector3(2.9, 3.7, 1), radius: 11 };
const VIEWS = {
  iso: { dir: [0.55, 0.35, 0.85] },
  front: { dir: [0, 0.05, 1] },
  top: { dir: [0, 1, 0.12] },
  pin: { pos: [4.5, 3.2, 7], target: [0, 0, 0.8] },
};

// Width hidden by the floating panel on large screens (0 on phones or when collapsed)
const panelOffset = () => (!isSmallScreen() && !panel.classList.contains('collapsed') ? panel.offsetWidth + 14 : 0);

// Shift the projection so the model is centred in the area left of the panel
function updateViewOffset() {
  const w = container.clientWidth, h = container.clientHeight, off = panelOffset();
  if (off) camera.setViewOffset(w, h, off / 2, 0, w, h);
  else camera.clearViewOffset();
}

function fitDistance() {
  const w = Math.max(container.clientWidth - panelOffset(), 1), h = Math.max(container.clientHeight, 1);
  const vFov = THREE.MathUtils.degToRad(camera.fov);
  const hFov = 2 * Math.atan(Math.tan(vFov / 2) * (w / h));
  return (FRAME.radius / Math.sin(Math.min(vFov, hFov) / 2)) * 0.9;
}

let camTween = null;
function goToView(name, instant = false) {
  const v = VIEWS[name];
  const to = v.dir
    ? { target: FRAME.center.clone(), pos: FRAME.center.clone().addScaledVector(new THREE.Vector3(...v.dir).normalize(), fitDistance()) }
    : { pos: new THREE.Vector3(...v.pos), target: new THREE.Vector3(...v.target) };
  if (instant) { camera.position.copy(to.pos); controls.target.copy(to.target); controls.update(); return; }
  camTween = { from: { pos: camera.position.clone(), target: controls.target.clone() }, to, t: 0 };
  requestRender();
}
document.querySelectorAll('[data-view]').forEach((b) => b.addEventListener('click', () => goToView(b.dataset.view)));

// ---------------------------------------------------------------------------
// Render loop (renders on demand to save battery on mobile devices)
// ---------------------------------------------------------------------------
let running = false;
let last = performance.now();
const tmp = new THREE.Vector3();

function requestRender() {
  if (!running) { running = true; last = performance.now(); requestAnimationFrame(tick); }
}

function approach(current, target, rate, dt) {
  return current + (target - current) * (1 - Math.exp(-rate * dt));
}

function stepSimulation(dt) {
  if (state.demo) {
    state.demoClock += dt;
    const s = Math.sin(state.demoClock * 2.2);
    if (state.mode === 'active') state.servoTarget = THREE.MathUtils.degToRad(10 * s);
    else state.travelTarget = (s > 0 ? 14 * s : 9 * s) / UNIT_MM;
  }

  // Sequencing: in manual mode the arm returns to neutral before the pin moves;
  // in active mode the pin re-engages the arm before the servo moves.
  let thetaTarget, pinTarget;
  if (state.mode === 'manual') {
    thetaTarget = 0;
    pinTarget = Math.abs(state.theta) < 0.004 ? PIN_Z.manual : PIN_Z.active;
  } else {
    pinTarget = PIN_Z.active;
    thetaTarget = state.pinZ > PIN_Z.active - 0.02 ? state.servoTarget : 0;
    if (state.pinZ < 0.95) state.travel = approach(state.travel, 0, 8, dt);
  }
  state.theta = approach(state.theta, thetaTarget, state.demo ? 14 : 7, dt);
  state.pinZ = approach(state.pinZ, pinTarget, 6, dt);
  if (state.mode === 'manual' && state.pinZ <= 0.95) state.travel = approach(state.travel, state.travelTarget, state.demo ? 14 : 9, dt);

  updateAssembly();
  const settled = Math.abs(state.theta - thetaTarget) < 1e-4 && Math.abs(state.pinZ - pinTarget) < 1e-3 &&
    (state.mode !== 'manual' || Math.abs(state.travel - state.travelTarget) < 1e-4);
  return !settled || state.demo;
}

function stepCamera(dt) {
  if (!camTween) return false;
  camTween.t = Math.min(camTween.t + dt / 0.8, 1);
  const k = 1 - Math.pow(1 - camTween.t, 3);
  camera.position.lerpVectors(camTween.from.pos, camTween.to.pos, k);
  controls.target.lerpVectors(camTween.from.target, camTween.to.target, k);
  if (camTween.t >= 1) camTween = null;
  return true;
}

function updateLabels() {
  const w = container.clientWidth, h = container.clientHeight;
  for (const { el, anchor } of labels) {
    tmp.copy(anchor()).project(camera);
    const visible = state.labels && tmp.z < 1 && Math.abs(tmp.x) < 1.05 && Math.abs(tmp.y) < 1.05;
    el.style.display = visible ? '' : 'none';
    if (visible) el.style.transform = `translate(${((tmp.x + 1) / 2) * w}px, ${((1 - tmp.y) / 2) * h}px) translate(-50%, -50%)`;
  }
}

function tick(now) {
  const dt = Math.min((now - last) / 1000, 0.05);
  last = now;
  const animating = stepSimulation(dt);
  const cameraMoving = stepCamera(dt);
  const orbiting = controls.update();
  renderer.render(scene, camera);
  updateLabels();
  refreshReadouts();
  if (state.demo) refreshUI();
  if (animating || cameraMoving || orbiting) requestAnimationFrame(tick);
  else running = false;
}

controls.addEventListener('change', requestRender);
controls.addEventListener('start', requestRender);

function resize() {
  const w = container.clientWidth, h = container.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  updateViewOffset();
  camera.updateProjectionMatrix();
  requestRender();
}
new ResizeObserver(resize).observe(container);

// ---------------------------------------------------------------------------
// Start
// ---------------------------------------------------------------------------
updateAssembly();
refreshUI();
resize();
goToView('iso', true);
document.getElementById('loading').classList.add('done');
