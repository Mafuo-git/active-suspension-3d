// Active Suspension 3D – suspension lab of a 1/4-scale F2-style car.
//
// The whole car is modelled: double wishbones on all four corners, steering rack and column,
// rear toe links and driveshafts, push- or pullrods, rockers and dampers. Every pickup point and
// alignment setting can be changed; the kinematics (suspension.js) are solved live.
//
// Each rocker is coaxial with a servo. A ball lock pin couples the rocker to the servo drive disc
// (active mode) or is parked in a holster that cuts servo power (manual mode, moved by hand).
// The kinematics are identical in both modes: only the input changes.
//
// Frames: the kinematics use ISO coordinates in mm (x forward, y left, z up). The scene uses
// Three.js coordinates (y up) with 1 unit = 10 mm; `iso` converts between them.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import * as S from './suspension.js';
import { buildBody } from './body.js';

const DEG = Math.PI / 180;
const V = (p) => new THREE.Vector3(p.x, p.y, p.z);

// ---------------------------------------------------------------------------
// Renderer, scene, camera, lights
// ---------------------------------------------------------------------------
const container = document.getElementById('viewport');
const isSmallScreen = () => window.matchMedia('(max-width: 720px)').matches;

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, isSmallScreen() ? 1.5 : 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
container.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;

const camera = new THREE.PerspectiveCamera(34, 1, 1, 2000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 8;
controls.maxDistance = 420;
controls.maxPolarAngle = Math.PI * 0.495;

const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(30, 70, 40);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -90, right: 90, top: 90, bottom: -90, near: 10, far: 220 });
sun.shadow.bias = -0.0005;
sun.target.position.set(-39, 0, 0);
scene.add(sun, sun.target, new THREE.HemisphereLight(0xffffff, 0x3a3a40, 0.35));

const ground = new THREE.Mesh(new THREE.PlaneGeometry(1200, 1200), new THREE.ShadowMaterial({ opacity: 0.3 }));
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);
const grid = new THREE.GridHelper(400, 40, 0x55555f, 0x3a3a42);
grid.material.transparent = true;
grid.material.opacity = 0.32;
grid.position.set(-39, 0.02, 0);
scene.add(grid);

// ISO (mm) -> scene: x -> x, y -> -z, z -> y, scaled by 1/10
const iso = new THREE.Group();
iso.matrixAutoUpdate = false;
iso.matrix.set(0.1, 0, 0, 0, 0, 0, 0.1, 0, 0, -0.1, 0, 0, 0, 0, 0, 1);
scene.add(iso);

// Chassis: everything sprung or attached to the chassis, posed by heave / roll / pitch
const chassis = new THREE.Group();
chassis.matrixAutoUpdate = false;
iso.add(chassis);

// ---------------------------------------------------------------------------
// Materials
// ---------------------------------------------------------------------------
const mat = (color, opts = {}) => new THREE.MeshStandardMaterial({ color, metalness: 0.2, roughness: 0.5, ...opts });
const M = {
  body: mat(0x2b2a3a, { roughness: 0.35, metalness: 0.3 }),
  accent: mat(0x7f77dd, { roughness: 0.35, metalness: 0.3 }),
  carbon: mat(0x1f1f24, { roughness: 0.5, metalness: 0.2 }),
  dark: mat(0x09090b, { roughness: 0.9 }),
  cockpit: mat(0x0e0e11, { roughness: 0.8 }),
  halo: mat(0x34343a, { metalness: 0.7, roughness: 0.35 }),
  helmet: mat(0xef9f27, { roughness: 0.25, metalness: 0.2 }),
  engine: mat(0x55545c, { metalness: 0.7, roughness: 0.4 }),
  alu: mat(0xbdbcb5, { metalness: 0.85, roughness: 0.3 }),
  steel: mat(0xdcdcd6, { metalness: 0.95, roughness: 0.2 }),
  wishbone: mat(0x3d3d46, { metalness: 0.6, roughness: 0.35 }),
  upright: mat(0x8e8d87, { metalness: 0.8, roughness: 0.35 }),
  rod: mat(0xd8d6ce, { metalness: 0.9, roughness: 0.22 }),
  tieRod: mat(0x5dcaa5, { metalness: 0.5, roughness: 0.35 }),
  servo: mat(0x534ab7, { roughness: 0.45 }),
  disc: mat(0x7f77dd, { metalness: 0.6, roughness: 0.3 }),
  rocker: mat(0xc9c8c1, { metalness: 0.85, roughness: 0.28 }),
  coilover: mat(0x1d9e75, { metalness: 0.5, roughness: 0.3 }),
  spring: mat(0x9fe1cb, { metalness: 0.4, roughness: 0.35 }),
  pin: mat(0xef9f27, { metalness: 0.75, roughness: 0.25 }),
  holster: mat(0x5f5e5a, { metalness: 0.6, roughness: 0.4 }),
  tyre: mat(0x18181a, { roughness: 0.92 }),
  stripe: mat(0xef9f27, { roughness: 0.6 }),
  rim: mat(0x9a9a96, { metalness: 0.9, roughness: 0.25, side: THREE.DoubleSide }),
  brake: mat(0x6b6a66, { metalness: 0.8, roughness: 0.45 }),
  caliper: mat(0xe24b4a, { roughness: 0.4 }),
  ledOn: mat(0x639922, { emissive: 0x3b6d11, emissiveIntensity: 1.2 }),
  ledOff: mat(0xe24b4a, { emissive: 0xa32d2d, emissiveIntensity: 1.2 }),
};
const bodyMaterials = [M.body, M.accent, M.carbon, M.dark, M.cockpit, M.halo];

// ---------------------------------------------------------------------------
// Geometry helpers (all in ISO mm inside `chassis`)
// ---------------------------------------------------------------------------
function mesh(geometry, material, parent, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(geometry, material);
  m.position.set(x, y, z);
  m.castShadow = true;
  m.receiveShadow = true;
  parent.add(m);
  return m;
}
const UP = new THREE.Vector3(0, 1, 0);
const tmpA = new THREE.Vector3(), tmpB = new THREE.Vector3();

// Unit-height Y-aligned cylinder stretched between two points
function rod(radius, material, parent, seg = 12) {
  const m = mesh(new THREE.CylinderGeometry(radius, radius, 1, seg), material, parent);
  m.update = (a, b) => {
    tmpA.set(a.x, a.y, a.z); tmpB.set(b.x, b.y, b.z);
    const dir = tmpB.sub(tmpA);
    const L = dir.length();
    m.position.copy(tmpA).addScaledVector(dir, 0.5);
    m.scale.set(1, Math.max(L, 1e-3), 1);
    if (L > 1e-6) m.quaternion.setFromUnitVectors(UP, dir.multiplyScalar(1 / L));
  };
  return m;
}
const ball = (r, material, parent) => mesh(new THREE.SphereGeometry(r, 14, 12), material, parent);
const cylW = (r, h, seg = 24, open = false) => new THREE.CylinderGeometry(r, r, h, seg, 1, open).rotateX(Math.PI / 2); // along local z

function plate(shape, depth, holes = [], bevel = 0.4) {
  for (const [hx, hy, r] of holes) {
    const p = new THREE.Path();
    p.absarc(hx, hy, r, 0, Math.PI * 2, true);
    shape.holes.push(p);
  }
  const g = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: bevel > 0, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 2, curveSegments: 24 });
  g.translate(0, 0, -depth / 2);
  return g;
}

// Convex outline wrapping circles [x, y, r]
function hullAroundCircles(circles, samples = 28) {
  const pts = [];
  for (const [x, y, r] of circles) for (let i = 0; i < samples; i++) {
    const a = (i / samples) * Math.PI * 2;
    pts.push([x + r * Math.cos(a), y + r * Math.sin(a)]);
  }
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const crossZ = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [], upper = [];
  for (const p of pts) { while (lower.length > 1 && crossZ(lower.at(-2), lower.at(-1), p) <= 0) lower.pop(); lower.push(p); }
  for (const p of pts.reverse()) { while (upper.length > 1 && crossZ(upper.at(-2), upper.at(-1), p) <= 0) upper.pop(); upper.push(p); }
  const hull = lower.slice(0, -1).concat(upper.slice(0, -1));
  const s = new THREE.Shape();
  hull.forEach(([x, y], i) => (i ? s.lineTo(x, y) : s.moveTo(x, y)));
  s.closePath();
  return s;
}

// Helical spring rebuilt to its actual length so the wire keeps a round section
function springGeometry(length, turns = 7, radius = 6.4, wire = 0.9) {
  const pts = [];
  for (let i = 0; i <= 200; i++) {
    const t = i / 200;
    pts.push(new THREE.Vector3(radius * Math.cos(t * Math.PI * 2 * turns), t * length, radius * Math.sin(t * Math.PI * 2 * turns)));
  }
  return new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 240, wire, 6, false);
}

// ---------------------------------------------------------------------------
// Setup state (URL hash > local storage > defaults)
// ---------------------------------------------------------------------------
function mergeSetup(base, over) {
  for (const k in over) {
    if (!(k in base)) continue;
    const b = base[k], o = over[k];
    if (b && typeof b === 'object' && o && typeof o === 'object') mergeSetup(b, o);
    else if (typeof b === typeof o) base[k] = o;
  }
  return base;
}
function loadInitialSetup() {
  const s = S.cloneSetup(S.DEFAULT_SETUP);
  try {
    const m = location.hash.match(/s=([^&]+)/);
    if (m) return mergeSetup(s, JSON.parse(decodeURIComponent(escape(atob(decodeURIComponent(m[1]))))));
  } catch (e) { /* ignore a broken link */ }
  try {
    const raw = localStorage.getItem('as3d-setup');
    if (raw) return mergeSetup(s, JSON.parse(raw));
  } catch (e) { /* storage unavailable */ }
  return s;
}
let setup = loadInitialSetup();
let car = S.buildCar(setup);
if (!carIsValid(car)) { setup = S.cloneSetup(S.DEFAULT_SETUP); car = S.buildCar(setup); }

function carIsValid(c) {
  for (const k of S.CORNERS) {
    const r = c[k].thetaRange;
    if (Number.isNaN(r.min) || Number.isNaN(r.max) || !S.solveLinks(c[k], 0)) return false;
  }
  return true;
}

let saveTimer = 0;
function persist() {
  clearTimeout(saveTimer);
  saveTimer = setTimeout(() => {
    try { localStorage.setItem('as3d-setup', JSON.stringify(setup)); } catch (e) { /* ignore */ }
  }, 300);
}
const shareLink = () => `${location.origin}${location.pathname}#s=${encodeURIComponent(btoa(unescape(encodeURIComponent(JSON.stringify(setup)))))}`;

// ---------------------------------------------------------------------------
// Bodywork
// ---------------------------------------------------------------------------
const bodyParts = buildBody(M, setup.vehicle.wheelbase);
chassis.add(bodyParts.body, bodyParts.inner);
let bodyMode = 'ghost';
function applyBodyMode() {
  bodyParts.body.visible = bodyMode !== 'hidden';
  for (const m of bodyMaterials) {
    const ghost = bodyMode === 'ghost';
    m.transparent = ghost; m.opacity = ghost ? 0.2 : 1; m.depthWrite = !ghost; m.needsUpdate = true;
  }
  bodyParts.body.traverse((o) => { if (o.isMesh) o.castShadow = bodyMode === 'solid'; });
  document.getElementById('btnBody').textContent = `Body: ${bodyMode}`;
}

// ---------------------------------------------------------------------------
// Steering: rack housing, rack bar, column and steering wheel
// ---------------------------------------------------------------------------
const steering = { group: new THREE.Group() };
chassis.add(steering.group);
steering.housing = rod(5, M.alu, steering.group, 20);
steering.bar = rod(2.4, M.steel, steering.group);
steering.column = rod(2.2, M.steel, steering.group);
steering.wheel = new THREE.Group();
steering.group.add(steering.wheel);
{
  const rim = new THREE.Shape();
  rim.moveTo(-24, -8); rim.lineTo(24, -8); rim.quadraticCurveTo(32, 0, 24, 11); rim.lineTo(-24, 11); rim.quadraticCurveTo(-32, 0, -24, -8);
  const g = plate(rim, 4, [[-15, 1.5, 6], [15, 1.5, 6], [0, 1, 6.5]], 0.6);
  g.applyMatrix4(new THREE.Matrix4().set(0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1)); // shape x -> y, shape y -> z, depth -> x
  mesh(g, M.carbon, steering.wheel);
  mesh(new THREE.BoxGeometry(2, 2, 6), M.accent, steering.wheel, 2.4, 0, 9);
}

// ---------------------------------------------------------------------------
// Corner visuals
// ---------------------------------------------------------------------------
const RIM_RADIUS = 57;

function buildWheel(radius, width) {
  const w = new THREE.Group();
  const hw = width / 2, rc = 9;
  const prof = [new THREE.Vector2(RIM_RADIUS, -hw + 3)];
  for (let i = 0; i <= 8; i++) { const a = -Math.PI / 2 + (i / 8) * (Math.PI / 2); prof.push(new THREE.Vector2(radius - rc + rc * Math.cos(a), -hw + rc + rc * Math.sin(a))); }
  for (let i = 0; i <= 8; i++) { const a = (i / 8) * (Math.PI / 2); prof.push(new THREE.Vector2(radius - rc + rc * Math.cos(a), hw - rc + rc * Math.sin(a))); }
  prof.push(new THREE.Vector2(RIM_RADIUS, hw - 3));
  mesh(new THREE.LatheGeometry(prof, 64), M.tyre, w);
  const stripe = mesh(new THREE.TorusGeometry(radius - 14, 0.9, 6, 64).rotateX(Math.PI / 2), M.stripe, w, 0, hw + 0.2, 0);
  stripe.castShadow = false;
  mesh(new THREE.CylinderGeometry(RIM_RADIUS, RIM_RADIUS, width - 8, 48, 1, true), M.rim, w);
  for (let i = 0; i < 6; i++) {
    const spoke = new THREE.Group();
    spoke.rotation.y = (i / 6) * Math.PI * 2;
    w.add(spoke);
    mesh(new THREE.BoxGeometry(5, 4, RIM_RADIUS - 8), M.rim, spoke, 0, hw - 12, (RIM_RADIUS - 8) / 2 + 4);
  }
  mesh(new THREE.CylinderGeometry(11, 11, 8, 24), M.rim, w, 0, hw - 12, 0);
  // brake disc and caliper (inboard)
  mesh(new THREE.CylinderGeometry(44, 44, 4, 40), M.brake, w, 0, -hw + 20, 0);
  mesh(new THREE.BoxGeometry(16, 14, 22), M.caliper, w, -40, -hw + 20, 12);
  return w;
}

function buildCornerVisual(key) {
  const front = key[0] === 'f', g = new THREE.Group();
  chassis.add(g);
  const c = { key, front, group: g };

  c.brackets = [0, 1, 2, 3].map(() => mesh(new THREE.BoxGeometry(9, 9, 9), M.alu, g));
  c.legs = [0, 1, 2, 3].map(() => rod(3.2, M.wishbone, g));
  c.lbj = ball(4.2, M.steel, g);
  c.ubj = ball(4.2, M.steel, g);
  c.uprightArms = [0, 1, 2].map(() => rod(5, M.upright, g));
  c.hub = mesh(new THREE.CylinderGeometry(13, 13, 22, 24), M.upright, g);
  c.tieRod = rod(2.5, M.tieRod, g);
  c.tieEnds = [ball(3.4, M.tieRod, g), ball(3.4, M.tieRod, g)];
  c.tieBracket = mesh(new THREE.BoxGeometry(8, 8, 8), M.alu, g);
  c.rod = rod(2.8, M.rod, g);
  c.rodEnds = [ball(3.6, M.rod, g), ball(3.6, M.rod, g)];
  c.wheel = buildWheel(setup[front ? 'front' : 'rear'].tyreRadius, setup[front ? 'front' : 'rear'].tyreWidth);
  g.add(c.wheel);
  if (!front) {
    c.driveshaft = rod(4, M.steel, g);
    c.boots = [ball(7, M.carbon, g), ball(7, M.carbon, g)];
  }

  // Rocker base frame (u, v in the rocker plane, w = rocker axis): servo, drive disc, holster
  c.base = new THREE.Group();
  g.add(c.base);
  c.rocker = new THREE.Group();
  c.base.add(c.rocker);
  c.disc = new THREE.Group();
  c.base.add(c.disc);
  c.servoParts = new THREE.Group();
  c.base.add(c.servoParts);
  mesh(new THREE.BoxGeometry(24, 13, 20), M.servo, c.servoParts, 0, 0, -19);
  mesh(new THREE.BoxGeometry(33, 13, 2), M.servo, c.servoParts, 0, 0, -13);
  mesh(cylW(2, 16, 16), M.steel, c.servoParts, 0, 0, -3);
  mesh(cylW(3.2, 2, 6), M.steel, c.servoParts, 0, 0, 3.4);

  c.holster = new THREE.Group();
  c.base.add(c.holster);
  mesh(cylW(4.2, 12, 20), M.holster, c.holster, 0, 0, -4);
  mesh(new THREE.CircleGeometry(2.4, 20), M.dark, c.holster, 0, 0, 2.05);
  mesh(new THREE.BoxGeometry(9, 7, 6), M.holster, c.holster, 0, 0, -13);
  c.led = mesh(new THREE.SphereGeometry(1.5, 10, 10), M.ledOn, c.holster, 0, 5, -13);

  c.pin = new THREE.Group();
  c.base.add(c.pin);
  mesh(cylW(1.8, 18, 20), M.pin, c.pin);
  mesh(cylW(3.8, 2.4, 24), M.pin, c.pin, 0, 0, 9.6);
  const ring = mesh(new THREE.TorusGeometry(3.4, 0.6, 8, 24), M.pin, c.pin, 0, 0, 14.5);
  ring.rotation.y = Math.PI / 2;

  // Damper (coilover) along local +y from the chassis mount to the rocker
  c.coilover = new THREE.Group();
  g.add(c.coilover);
  const eye = new THREE.TorusGeometry(3.4, 1.2, 8, 20);
  c.eyeA = mesh(eye, M.coilover, c.coilover);
  c.eyeB = mesh(eye, M.coilover, c.coilover);
  mesh(new THREE.CylinderGeometry(4.6, 4.6, 30, 24), M.coilover, c.coilover, 0, 20, 0);
  c.shaft = mesh(new THREE.CylinderGeometry(1.4, 1.4, 1, 12), M.steel, c.coilover);
  mesh(new THREE.CylinderGeometry(7.5, 7.5, 1.4, 24), M.coilover, c.coilover, 0, 10, 0);
  c.perchB = mesh(new THREE.CylinderGeometry(7.5, 7.5, 1.4, 24), M.coilover, c.coilover);
  c.springLength = 0;
  c.spring = mesh(springGeometry(40), M.spring, c.coilover, 0, 10.7, 0);
  return c;
}

const visuals = Object.fromEntries(S.CORNERS.map((k) => [k, buildCornerVisual(k)]));

// Static (setup-dependent) parts: rocker frame and plate, pin hole, holster position
function rebuildStaticVisuals() {
  for (const k of S.CORNERS) {
    const c = car[k], v = visuals[k], P = c.P;
    const w = c.rockerAxis;
    const toPlane = (p) => { const d = S.sub(p, P.rockerPivot); return S.sub(d, S.scale(w, S.dot(d, w))); };
    const u = S.norm(toPlane(P.rockerRod)), vv = S.cross(w, u);
    v.frame = { u, v: vv, w };
    const m = new THREE.Matrix4().makeBasis(V(u), V(vv), V(w));
    v.base.position.copy(V(P.rockerPivot));
    v.base.quaternion.setFromRotationMatrix(m);
    const loc = (p) => { const d = S.sub(p, P.rockerPivot); return [S.dot(d, u), S.dot(d, vv), S.dot(d, w)]; };
    const a = loc(P.rockerRod), b = loc(P.rockerDamper);
    // pin hole opposite the two arms, holster away from both arms
    const ang = (q) => Math.atan2(q[1], q[0]);
    const mean = Math.atan2(Math.sin(ang(a)) + Math.sin(ang(b)), Math.cos(ang(a)) + Math.cos(ang(b)));
    v.holeAngle = mean + Math.PI;
    v.holeR = 11;
    const cand = [v.holeAngle + 1.45, v.holeAngle - 1.45];
    const gap = (t) => Math.min(Math.abs(Math.atan2(Math.sin(t - ang(a)), Math.cos(t - ang(a)))), Math.abs(Math.atan2(Math.sin(t - ang(b)), Math.cos(t - ang(b)))));
    v.holsterAngle = gap(cand[0]) > gap(cand[1]) ? cand[0] : cand[1];
    v.holsterPos = new THREE.Vector3(34 * Math.cos(v.holsterAngle), 34 * Math.sin(v.holsterAngle), 0);
    v.holster.position.copy(v.holsterPos);

    // rocker plate
    v.rocker.clear();
    const hole = [v.holeR * Math.cos(v.holeAngle), v.holeR * Math.sin(v.holeAngle)];
    const outline = hullAroundCircles([[0, 0, 15], [a[0], a[1], 6], [b[0], b[1], 6]]);
    mesh(plate(outline, 4, [[0, 0, 2.6], [a[0], a[1], 2], [b[0], b[1], 2], [hole[0], hole[1], 2.1]]), M.rocker, v.rocker);
    mesh(new THREE.TorusGeometry(3.4, 0.6, 8, 20), M.pin, v.rocker, hole[0], hole[1], 2.3);
    for (const q of [a, b]) if (Math.abs(q[2]) > 1) { const r = rod(1.8, M.rocker, v.rocker); r.update({ x: q[0], y: q[1], z: 0 }, { x: q[0], y: q[1], z: q[2] }); }
    // drive disc
    v.disc.clear();
    const ds = new THREE.Shape(); ds.absarc(0, 0, 16, 0, Math.PI * 2, false);
    mesh(plate(ds, 2.4, [[hole[0], hole[1], 2.1]], 0.2), M.disc, v.disc, 0, 0, -5);
    mesh(new THREE.BoxGeometry(2, 5, 1), M.steel, v.disc, 14 * Math.cos(v.holeAngle + Math.PI), 14 * Math.sin(v.holeAngle + Math.PI), -3.6);
  }
}

// ---------------------------------------------------------------------------
// Pickup point markers (left side) and the move gizmo
// ---------------------------------------------------------------------------
const GROUP_COLORS = { wishbone: 0x7f77dd, upright: 0xe24b4a, steering: 0x5dcaa5, rod: 0xdcdcd6, rocker: 0xef9f27, damper: 0x1d9e75 };
const markers = new THREE.Group();
markers.visible = false;
chassis.add(markers);
const markerGeo = new THREE.SphereGeometry(4.6, 16, 12);
const markerMats = Object.fromEntries(Object.entries(GROUP_COLORS).map(([k, c]) => [k, new THREE.MeshBasicMaterial({ color: c, depthTest: false, transparent: true, opacity: 0.95 })]));
const markerSel = new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false });
const markerList = [];
function rebuildMarkers() {
  markers.clear();
  markerList.length = 0;
  for (const axle of ['front', 'rear']) {
    const pts = S.cornerPoints(setup, axle);
    for (const key of Object.keys(S.POINT_INFO)) {
      const info = S.POINT_INFO[key];
      const m = new THREE.Mesh(markerGeo, markerMats[info.group]);
      m.renderOrder = 10;
      m.position.copy(V(pts[key]));
      m.userData = { axle, key };
      markers.add(m);
      markerList.push(m);
    }
  }
  highlightMarker();
}
function highlightMarker() {
  for (const m of markerList) {
    const sel = state.selected && m.userData.axle === state.selected.axle && m.userData.key === state.selected.key;
    m.material = sel ? markerSel : markerMats[S.POINT_INFO[m.userData.key].group];
    m.scale.setScalar(sel ? 1.35 : 1);
  }
}

const gizmoTarget = new THREE.Object3D();
chassis.add(gizmoTarget);
const gizmo = new TransformControls(camera, renderer.domElement);
gizmo.setSize(0.75);
gizmo.setSpace('world');
gizmo.addEventListener('dragging-changed', (e) => { controls.enabled = !e.value; if (!e.value) { persist(); refreshAnalysis(); } });
gizmo.addEventListener('objectChange', () => {
  if (!state.selected) return;
  const p = gizmoTarget.position;
  applyPoint(state.selected.axle, state.selected.key, S.vec(p.x, p.y, p.z), true);
});
gizmo.addEventListener('change', () => requestRender());
scene.add(gizmo);

// ---------------------------------------------------------------------------
// Geometry overlay: instant centres, roll centres, roll axis, steering axes
// ---------------------------------------------------------------------------
const overlay = new THREE.Group();
overlay.visible = false;
chassis.add(overlay);
const OVERLAY_SEGMENTS = 40;
const overlayGeo = new THREE.BufferGeometry();
overlayGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(OVERLAY_SEGMENTS * 6), 3));
const overlayLines = new THREE.LineSegments(overlayGeo, new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.55, depthTest: false }));
overlayLines.renderOrder = 9;
overlay.add(overlayLines);
const kpGeo = new THREE.BufferGeometry();
kpGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(8 * 6), 3));
const kpLines = new THREE.LineSegments(kpGeo, new THREE.LineBasicMaterial({ color: 0xe24b4a, depthTest: false }));
kpLines.renderOrder = 9;
overlay.add(kpLines);
const rcMarkers = [0, 1].map((i) => { const m = new THREE.Mesh(new THREE.SphereGeometry(5.5, 16, 12), new THREE.MeshBasicMaterial({ color: i ? 0xef9f27 : 0xafa9ec, depthTest: false })); m.renderOrder = 10; overlay.add(m); return m; });
const groundMarkers = S.CORNERS.flatMap(() => [0, 1]).map((_, i) => {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(3.2, 3.2, 0.6, 16).rotateX(Math.PI / 2), new THREE.MeshBasicMaterial({ color: i % 2 ? 0xe24b4a : 0xffffff, depthTest: false }));
  m.renderOrder = 10; overlay.add(m); return m;
});

// ---------------------------------------------------------------------------
// Simulation state
// ---------------------------------------------------------------------------
const INPUTS = [
  { key: 'heave', label: 'Heave', unit: 'mm', min: -12, max: 12, step: 0.5, digits: 1, help: 'Body moving down (+) or up (−). All four wheels compress or extend together.' },
  { key: 'roll', label: 'Roll', unit: '°', min: -3, max: 3, step: 0.05, digits: 2, help: 'Body leaning to the right (+) as in a left-hand corner, or to the left (−).' },
  { key: 'dive', label: 'Pitch (dive)', unit: '°', min: -2, max: 2, step: 0.05, digits: 2, help: 'Nose down (+) as under braking, nose up (−) as under acceleration.' },
  { key: 'steer', label: 'Steering wheel', unit: '°', min: -180, max: 180, step: 1, digits: 0, help: 'Steering wheel angle: + turns left. The rack slides and the tie rods steer the front uprights.' },
];

const state = {
  tab: 'drive', axle: 'front', selected: null,
  mode: 'active', phase: 'LINKED', pinT: 0,
  input: { heave: 0, roll: 0, dive: 0, steer: 0 },
  target: { heave: 0, roll: 0, dive: 0, steer: 0 },
  road: { fl: 0, fr: 0, rl: 0, rr: 0 },
  scenario: null, clock: 0,
  psi: { fl: 0, fr: 0, rl: 0, rr: 0 },
  solution: null,
  labels: true, overlay: false,
};

function currentPose() {
  const L = setup.vehicle.wheelbase;
  const hF = setup.front.rideHeight || 0, hR = setup.rear.rideHeight || 0;
  return {
    heave: -state.input.heave + (hF + hR) / 2,
    roll: state.input.roll,
    pitch: -state.input.dive + Math.atan((hF - hR) / L) / DEG,
    steer: state.input.steer,
    road: state.road,
  };
}

function solve() {
  const pose = currentPose();
  // per-corner road height (kerbs): shift the road plane locally
  const P = S.chassisPose(setup, pose);
  const rack = S.rackFromSteer(setup, pose.steer);
  const out = { pose: P };
  for (const k of S.CORNERS) {
    const g = { n: P.ground.n, d: P.ground.d - (state.road[k] || 0) };
    const prev = state.solution?.[k];
    out[k] = S.solveOnGround(car[k], g, k[0] === 'f' ? rack : 0, prev) || prev;
    if (out[k]) out[k].motionRatio = S.motionRatio(car[k], out[k].theta, k[0] === 'f' ? rack : 0);
  }
  out.rcFront = S.rollCentre(car.fl, car.fr, out.fl, out.fr);
  out.rcRear = S.rollCentre(car.rl, car.rr, out.rl, out.rr);
  out.rack = rack;
  state.solution = out;
  return out;
}

// ---------------------------------------------------------------------------
// Pose update of all visuals
// ---------------------------------------------------------------------------
const smooth = (t) => t * t * (3 - 2 * t);
const PIN_HOVER = 0.7;
const pinLocal = (v, t, angle) => {
  const lift = smooth(THREE.MathUtils.clamp(t / 0.3, 0, 1));
  const move = smooth(THREE.MathUtils.clamp((t - 0.3) / 0.4, 0, 1));
  const drop = smooth(THREE.MathUtils.clamp((t - 0.7) / 0.3, 0, 1));
  const hole = new THREE.Vector3(v.holeR * Math.cos(angle), v.holeR * Math.sin(angle), 0);
  const p = hole.lerp(v.holsterPos, move);
  p.z = THREE.MathUtils.lerp(-1.5, 15, lift - drop);
  return p;
};
const servoPowered = () => !(state.phase === 'PARKED' || state.phase === 'RELEASE' || ((state.phase === 'UNLINK' || state.phase === 'UNPARK') && state.pinT > 0.97));

function updateVisuals(sol) {
  // chassis pose
  const { R, ref, h } = sol.pose;
  const t = S.sub(ref, S.vec(
    R[0][0] * ref.x + R[0][1] * ref.y + R[0][2] * ref.z,
    R[1][0] * ref.x + R[1][1] * ref.y + R[1][2] * ref.z,
    R[2][0] * ref.x + R[2][1] * ref.y + R[2][2] * ref.z,
  ));
  chassis.matrix.set(R[0][0], R[0][1], R[0][2], t.x, R[1][0], R[1][1], R[1][2], t.y, R[2][0], R[2][1], R[2][2], t.z + h, 0, 0, 0, 1);
  chassis.matrixWorldNeedsUpdate = true;

  for (const k of S.CORNERS) {
    const c = car[k], v = visuals[k], s = sol[k], P = c.P;
    if (!s) continue;
    // chassis brackets and wishbones
    [P.lcaFront, P.lcaRear, P.ucaFront, P.ucaRear].forEach((p, i) => v.brackets[i].position.copy(V(p)));
    v.legs[0].update(P.lcaFront, s.lbj); v.legs[1].update(P.lcaRear, s.lbj);
    v.legs[2].update(P.ucaFront, s.ubj); v.legs[3].update(P.ucaRear, s.ubj);
    v.lbj.position.copy(V(s.lbj)); v.ubj.position.copy(V(s.ubj));
    // upright
    const a = S.norm(s.axis), hubC = S.sub(s.wc, S.scale(a, 16));
    v.uprightArms[0].update(s.lbj, hubC); v.uprightArms[1].update(s.ubj, hubC); v.uprightArms[2].update(s.tro, hubC);
    v.hub.position.copy(V(hubC));
    v.hub.quaternion.setFromUnitVectors(UP, V(a));
    // wheel: local y = spin axis (outboard), local z along the kingpin
    const kp = S.norm(S.sub(s.ubj, s.lbj));
    const zb = S.norm(S.sub(kp, S.scale(a, S.dot(kp, a)))), xb = S.cross(a, zb);
    v.wheel.position.copy(V(s.wc));
    v.wheel.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(V(xb), V(a), V(zb)));
    // tie rod / toe link
    v.tieRod.update(s.tri, s.tro);
    v.tieEnds[0].position.copy(V(s.tri)); v.tieEnds[1].position.copy(V(s.tro));
    v.tieBracket.position.copy(V(P.tieRodInner));
    v.tieBracket.visible = !v.front;
    // push / pullrod
    v.rod.update(s.rodOuter, s.rockerRod);
    v.rodEnds[0].position.copy(V(s.rodOuter)); v.rodEnds[1].position.copy(V(s.rockerRod));
    // rocker, drive disc, pin
    v.rocker.rotation.z = s.phi;
    if (state.phase === 'LINKED' || state.phase === 'NEUTRAL') state.psi[k] = s.phi;
    v.disc.rotation.z = state.psi[k];
    v.pin.position.copy(pinLocal(v, state.pinT, v.holeAngle + s.phi));
    v.led.material = servoPowered() ? M.ledOn : M.ledOff;
    // damper
    const dA = P.damperChassis, dB = s.rockerDamper, L = s.damperLength;
    v.coilover.position.copy(V(dA));
    v.coilover.quaternion.setFromUnitVectors(UP, V(S.norm(S.sub(dB, dA))));
    v.eyeB.position.y = L;
    v.shaft.scale.y = Math.max(L - 4 - 25, 0.5);
    v.shaft.position.y = (25 + L - 4) / 2;
    v.perchB.position.y = L - 7.5;
    const springL = Math.max(L - 7.5 - 10.7 - 0.7, 2);
    if (Math.abs(springL - v.springLength) > 0.05) {
      v.spring.geometry.dispose();
      v.spring.geometry = springGeometry(springL);
      v.springLength = springL;
    }
    // driveshaft (rear)
    if (v.driveshaft) {
      const inner = S.vec(s.wc.x, Math.sign(s.wc.y) * 30, 76), outer = S.sub(s.wc, S.scale(a, 30));
      v.driveshaft.update(inner, outer);
      v.boots[0].position.copy(V(inner)); v.boots[1].position.copy(V(outer));
    }
  }

  // steering rack, column and wheel
  const fl = sol.fl, fr = sol.fr;
  if (fl && fr) {
    const PL = car.fl.P.tieRodInner, PR = car.fr.P.tieRodInner;
    steering.housing.update(S.vec(PL.x, PL.y - 9, PL.z), S.vec(PR.x, PR.y + 9, PR.z));
    steering.bar.update(fl.tri, fr.tri);
    const pinion = S.vec(PL.x, 0, PL.z + 4), sw = S.vec(-215, 0, 150);
    steering.column.update(pinion, sw);
    steering.wheel.position.copy(V(sw));
    const colDir = S.norm(S.sub(pinion, sw));
    steering.wheel.quaternion.setFromUnitVectors(new THREE.Vector3(1, 0, 0), V(colDir));
    steering.wheel.rotateX(state.input.steer * DEG);
  }

  if (overlay.visible) updateOverlay(sol);
}

function updateOverlay(sol) {
  const pos = overlayGeo.attributes.position.array;
  let n = 0;
  const seg = (a, b) => { if (n >= OVERLAY_SEGMENTS) return; pos.set([a.x, a.y, a.z, b.x, b.y, b.z], n * 6); n++; };
  const clampFar = (from, to, maxLen = 1400) => { const d = S.sub(to, from), L = S.len(d); return L > maxLen ? S.add(from, S.scale(d, maxLen / L)) : to; };
  for (const [kl, kr, rcKey, i] of [['fl', 'fr', 'rcFront', 0], ['rl', 'rr', 'rcRear', 1]]) {
    const rc = sol[rcKey];
    for (const k of [kl, kr]) {
      const s = sol[k];
      if (!s) continue;
      const ic = rc ? (k === kl ? rc.icl : rc.icr) : S.frontViewIC(car[k], s);
      if (!ic) continue;
      seg(s.lbj, clampFar(s.lbj, ic)); seg(s.ubj, clampFar(s.ubj, ic)); seg(s.cp, clampFar(s.cp, ic));
    }
    rcMarkers[i].visible = !!rc;
    if (rc) rcMarkers[i].position.copy(V(rc.point));
  }
  if (sol.rcFront && sol.rcRear) seg(S.add(sol.rcFront.point, S.vec(160, 0, 0)), S.add(sol.rcRear.point, S.vec(-160, 0, 0)));
  pos.fill(0, n * 6);
  overlayGeo.attributes.position.needsUpdate = true;
  overlayGeo.setDrawRange(0, n * 2);

  // steering axes extended to the road, with kingpin ground point (red) and contact patch (white)
  const kp = kpGeo.attributes.position.array;
  S.CORNERS.forEach((k, i) => {
    const s = sol[k];
    if (!s) return;
    const top = S.add(s.ubj, S.scale(S.norm(S.sub(s.ubj, s.lbj)), 25));
    kp.set([top.x, top.y, top.z, s.kGround.x, s.kGround.y, s.kGround.z], i * 6);
    groundMarkers[i * 2].position.copy(V(s.cp)).setZ(s.cp.z + 0.5);
    groundMarkers[i * 2 + 1].position.copy(V(s.kGround)).setZ(s.kGround.z + 0.8);
    groundMarkers[i * 2].quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), V(sol.pose.ground.n));
    groundMarkers[i * 2 + 1].quaternion.copy(groundMarkers[i * 2].quaternion);
  });
  kpGeo.attributes.position.needsUpdate = true;
  kpGeo.setDrawRange(0, 8);
}

// ---------------------------------------------------------------------------
// Scenarios and the active / manual sequencer
// ---------------------------------------------------------------------------
const SCENARIOS = {
  brake: (t) => { const p = (t % 3.2) / 3.2; const k = p < 0.15 ? smooth(p / 0.15) : p < 0.55 ? 1 : p < 0.7 ? 1 - smooth((p - 0.55) / 0.15) : 0; return { heave: 3 * k, roll: 0, dive: 1.3 * k, steer: 0 }; },
  accel: (t) => { const p = (t % 3.2) / 3.2; const k = p < 0.2 ? smooth(p / 0.2) : p < 0.6 ? 1 : p < 0.75 ? 1 - smooth((p - 0.6) / 0.15) : 0; return { heave: 0.5 * k, roll: 0, dive: -0.9 * k, steer: 0 }; },
  corner: (t) => { const s = Math.sin(t * 1.1); return { heave: 1.5 * Math.abs(s), roll: 1.8 * s, dive: 0, steer: 120 * s }; },
  slalom: (t) => ({ heave: 0.8, roll: 1.6 * Math.sin(t * 2.4 - 0.35), dive: 0, steer: 140 * Math.sin(t * 2.4) }),
  kerb: () => ({ heave: 0, roll: 0, dive: 0, steer: 0 }),
};
const kerbProfile = (t) => (t > 0 && t < 0.28 ? 13 * Math.sin((t / 0.28) * Math.PI) : 0);
const approach = (cur, target, rate, dt) => cur + (target - cur) * (1 - Math.exp(-rate * dt));
const zeroTargets = () => { for (const k in state.target) state.target[k] = 0; };

function stepSequence(dt) {
  let busy = false;
  if (state.scenario && state.phase === 'LINKED') {
    state.clock += dt;
    Object.assign(state.target, SCENARIOS[state.scenario](state.clock));
    if (state.scenario === 'kerb') {
      const p = state.clock % 1.8;
      state.road.fl = kerbProfile(p); state.road.rl = kerbProfile(p - 0.35);
    }
    busy = true;
  } else { state.road.fl = state.road.rl = 0; }
  const rate = state.scenario ? 18 : 9;
  let settled = true;
  for (const k in state.input) {
    state.input[k] = approach(state.input[k], state.target[k], rate, dt);
    if (Math.abs(state.input[k] - state.target[k]) > (k === 'steer' ? 0.05 : 0.005)) settled = false; else state.input[k] = state.target[k];
  }
  if (!settled) busy = true;

  switch (state.phase) {
    case 'NEUTRAL': zeroTargets(); if (settled) state.phase = 'UNLINK'; busy = true; break;
    case 'UNLINK': state.pinT = Math.min(state.pinT + dt * 0.9, 1); if (state.pinT >= 1) state.phase = 'PARKED'; busy = true; break;
    case 'RELEASE': zeroTargets(); if (settled) state.phase = 'UNPARK'; busy = true; break;
    case 'UNPARK': state.pinT = Math.max(state.pinT - dt * 0.9, PIN_HOVER); if (state.pinT <= PIN_HOVER) state.phase = 'ALIGN'; busy = true; break;
    case 'ALIGN': {
      let aligned = true;
      for (const k of S.CORNERS) {
        const target = state.solution?.[k]?.phi ?? 0;
        state.psi[k] = approach(state.psi[k], target, 6, dt);
        if (Math.abs(state.psi[k] - target) > 0.002) aligned = false;
      }
      if (aligned) state.phase = 'LINK';
      busy = true;
      break;
    }
    case 'LINK': state.pinT = Math.max(state.pinT - dt * 0.9, 0); if (state.pinT <= 0) state.phase = 'LINKED'; busy = true; break;
    default: break;
  }
  return busy;
}

const MODE_HINTS = {
  LINKED: 'Active: on each corner the ball lock pin locks the rocker to the servo drive disc. The servos drive the rockers, and the rockers drive the whole suspension.',
  NEUTRAL: 'Returning to static ride height so the pins are unloaded…',
  UNLINK: 'Pulling the pins out of the rockers and parking them in their holsters…',
  PARKED: 'Manual: the pins sit in their holsters and press the micro-switches, so servo power is OFF. The rockers turn freely on their bearings. The sliders act as your hands: the suspension moves exactly as in active mode.',
  RELEASE: 'Hands off: the springs bring the suspension back to static ride height…',
  UNPARK: 'Lifting the pins out of the holsters: micro-switches released, servo power ON…',
  ALIGN: 'Servos powered: aligning each drive disc hole with its rocker hole…',
  LINK: 'Inserting the pins through rockers and drive discs…',
};

function setMode(mode) {
  if (mode === state.mode) return;
  if (mode === 'manual' && state.phase === 'LINKED') { state.mode = 'manual'; state.scenario = null; state.phase = 'NEUTRAL'; }
  else if (mode === 'active' && state.phase === 'PARKED') { state.mode = 'active'; state.phase = 'RELEASE'; }
  else return;
  zeroTargets();
  syncDriveSliders();
  refreshDriveUI();
  requestRender();
}

// ---------------------------------------------------------------------------
// Setup changes
// ---------------------------------------------------------------------------
let warnTimer = 0;
function toast(msg) {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.classList.add('show');
  clearTimeout(warnTimer);
  warnTimer = setTimeout(() => el.classList.remove('show'), 2400);
}

// Apply a change to the setup; revert if the linkage cannot be assembled
function changeSetup(mutator, { live = false } = {}) {
  const backup = S.cloneSetup(setup);
  mutator(setup);
  const next = S.buildCar(setup);
  if (!carIsValid(next)) {
    setup = mergeSetup(setup, backup);
    toast('This geometry cannot be assembled: change reverted.');
    refreshSetupUI(); refreshPointsUI();
    return false;
  }
  car = next;
  state.solution = null;
  refreshLabelTexts();
  rebuildStaticVisuals();
  rebuildMarkers();
  if (!live) { persist(); refreshAnalysis(); }
  refreshSetupUI(live);
  refreshPointsUI(live);
  requestRender();
  return true;
}

function applyPoint(axle, key, p, live = false) {
  const ok = changeSetup((s) => S.setCornerPoint(s, axle, key, p), { live });
  if (!ok && state.selected) placeGizmo();
  return ok;
}

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------
const $ = (id) => document.getElementById(id);
const fmt = (x, d = 1) => (Number.isFinite(x) ? x.toFixed(d) : '–');

function makeSlider(parent, cfg, get, set, onFocusHelp) {
  const row = document.createElement('div');
  row.className = 'slider';
  const id = `sl-${cfg.key}-${Math.random().toString(36).slice(2, 7)}`;
  row.innerHTML = `<label for="${id}" title="${cfg.help || ''}">${cfg.label}</label><output></output><input id="${id}" type="range" min="${cfg.min}" max="${cfg.max}" step="${cfg.step}">${cfg.sub ? '<span class="sub"></span>' : ''}`;
  parent.appendChild(row);
  const input = row.querySelector('input'), out = row.querySelector('output'), sub = row.querySelector('.sub');
  const show = () => { const val = get(); input.value = val; out.textContent = cfg.format ? cfg.format(val) : `${fmt(val, cfg.digits ?? 1)} ${cfg.unit}`; if (sub && cfg.sub) sub.textContent = cfg.sub(); };
  input.addEventListener('input', () => { set(parseFloat(input.value)); show(); onFocusHelp?.(cfg); });
  input.addEventListener('focus', () => onFocusHelp?.(cfg));
  row.querySelector('label').addEventListener('click', () => onFocusHelp?.(cfg));
  show();
  return { show, input };
}

// Tabs
function setTab(tab) {
  state.tab = tab;
  document.querySelectorAll('.tabs button').forEach((b) => b.classList.toggle('on', b.dataset.tab === tab));
  document.querySelectorAll('.tab').forEach((s) => { s.hidden = s.dataset.tab !== tab; });
  markers.visible = tab === 'points';
  if (tab === 'points') {
    // edit the design position: chassis level, no steering
    state.scenario = null; zeroTargets(); syncDriveSliders();
  } else { state.selected = null; gizmo.detach(); highlightMarker(); }
  if (tab === 'analysis') refreshAnalysis();
  refreshDriveUI();
  requestRender();
}
document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => {
  if (panel.classList.contains('collapsed')) togglePanel();
  setTab(b.dataset.tab);
}));

// Drive tab
const driveSliders = INPUTS.map((cfg) => makeSlider($('driveSliders'), cfg,
  () => state.target[cfg.key],
  (v) => { state.target[cfg.key] = v; state.scenario = null; refreshDriveUI(); requestRender(); }));
function syncDriveSliders() { driveSliders.forEach((s) => s.show()); }
$('btnActive').addEventListener('click', () => setMode('active'));
$('btnManual').addEventListener('click', () => setMode('manual'));
$('scenarios').querySelectorAll('button').forEach((b) => b.addEventListener('click', () => {
  const sc = b.dataset.scenario;
  state.scenario = state.scenario === sc || sc === 'stop' ? null : sc;
  state.clock = 0;
  if (!state.scenario) { zeroTargets(); syncDriveSliders(); }
  refreshDriveUI();
  requestRender();
}));

function refreshDriveUI() {
  const active = state.mode === 'active', stable = state.phase === 'LINKED' || state.phase === 'PARKED';
  $('btnActive').classList.toggle('on', active);
  $('btnManual').classList.toggle('on', !active);
  driveSliders.forEach((s, i) => {
    s.input.disabled = !stable || state.tab === 'points';
    s.input.previousElementSibling.previousElementSibling.textContent = INPUTS[i].label + (INPUTS[i].key === 'steer' ? '' : active ? ' – servo' : ' – by hand');
  });
  $('scenarios').querySelectorAll('button').forEach((b) => {
    b.disabled = b.dataset.scenario !== 'stop' && !(active && state.phase === 'LINKED');
    b.classList.toggle('on', state.scenario === b.dataset.scenario);
  });
  $('modeHint').textContent = MODE_HINTS[state.phase];
}

const NAMES = { fl: 'FL', fr: 'FR', rl: 'RL', rr: 'RR' };
$('wheelTable').innerHTML = S.CORNERS.map((k) => `<tr><td>${NAMES[k]}</td><td id="t-${k}"></td><td id="c-${k}"></td><td id="s-${k}"></td><td id="d-${k}"></td></tr>`).join('');
function refreshWheelTable(sol) {
  const flags = [];
  for (const k of S.CORNERS) {
    const s = sol[k];
    if (!s) continue;
    $(`t-${k}`).textContent = `${fmt(s.travel)} mm`;
    $(`c-${k}`).textContent = `${fmt(s.camber, 2)}°`;
    $(`s-${k}`).textContent = k[0] === 'f' && Math.abs(state.input.steer) > 0.5 ? `${fmt(s.yaw, 1)}°` : `${fmt(s.toe, 2)}°`;
    $(`d-${k}`).textContent = `${fmt(s.damperLength)} mm`;
    $(`t-${k}`).className = s.status === 'ok' ? '' : 'flag';
    if (s.status === 'bump') flags.push(`${NAMES[k]} on bump stop`);
    if (s.status === 'droop') flags.push(`${NAMES[k]} wheel in the air`);
  }
  $('warn').hidden = !flags.length;
  $('warn').textContent = flags.join(' · ');
}

// Setup tab
const HELP = {
  rod: 'Pushrod: the rod works in compression, from the lower wishbone up to a rocker on top of the chassis. Pullrod: the rod works in tension, from the upper wishbone down to a rocker low in the chassis (lower centre of gravity, harder to reach).',
  camber: 'Camber: tilt of the wheel seen from the front. Negative = top of the tyre leaning in. It keeps the outer tyre flatter on the road when the car rolls in a corner.',
  toe: 'Toe: wheels pointing in (toe-in, +) or out (toe-out, −) seen from above. It is set by the length of the tie rod (front) or toe link (rear). Front toe-out sharpens turn-in; rear toe-in adds stability.',
  caster: 'Caster: backward tilt of the steering axis seen from the side. More caster gives stronger self-centring, heavier steering and more negative camber on the outer wheel when steering.',
  kpi: 'Kingpin inclination (KPI): inward tilt of the steering axis seen from the front. Steering lifts the car slightly, which helps the wheel return to the centre.',
  scrub: 'Scrub radius: distance on the road between the tyre centre and the point where the steering axis meets the road (front view). A small value limits steering kickback from bumps and braking.',
  trail: 'Mechanical trail: distance on the road between where the steering axis meets the road and the tyre centre (side view). It creates the self-aligning torque the driver feels.',
  lbjHeight: 'Lower ball joint height: where the lower wishbone meets the upright.',
  ubjHeight: 'Upper ball joint height: a taller upright (more spread between the ball joints) reduces the loads in the wishbones.',
  rideHeight: 'Ride height: set by turning the push/pullrod turnbuckle. A longer pushrod (or shorter pullrod) lifts the car at this axle.',
  track: 'Track width: distance between the centres of the left and right tyres.',
  wheelbase: 'Wheelbase: distance between the front and rear axles (bodywork not rescaled).',
  cgHeight: 'Height of the centre of gravity. Used for the anti-dive and anti-squat percentages.',
  brakeBiasFront: 'Brake balance: share of the braking force on the front axle. Used for anti-dive and anti-lift.',
  rackPerRev: 'Rack travel for one turn of the steering wheel. More travel = quicker steering.',
};
const AXLE_SLIDERS = [
  { key: 'camber', label: 'Camber', unit: '°', min: -5, max: 1, step: 0.05, digits: 2, path: ['alignment', 'camber'] },
  { key: 'toe', label: 'Toe (+ in)', unit: '°', min: -1.5, max: 1.5, step: 0.05, digits: 2, path: ['alignment', 'toe'], sub: () => `${state.axle === 'front' ? 'Tie rod' : 'Toe link'} length ${fmt(car[state.axle === 'front' ? 'fl' : 'rl'].tieRodLength, 2)} mm` },
  { key: 'caster', label: 'Caster', unit: '°', min: -2, max: 14, step: 0.1, digits: 1, path: ['alignment', 'caster'] },
  { key: 'kpi', label: 'Kingpin inclination (KPI)', unit: '°', min: -2, max: 16, step: 0.1, digits: 1, path: ['alignment', 'kpi'] },
  { key: 'scrub', label: 'Scrub radius', unit: 'mm', min: -10, max: 40, step: 0.5, digits: 1, path: ['alignment', 'scrub'] },
  { key: 'trail', label: 'Mechanical trail', unit: 'mm', min: -10, max: 30, step: 0.5, digits: 1, path: ['alignment', 'trail'] },
  { key: 'lbjHeight', label: 'Lower ball joint height', unit: 'mm', min: 28, max: 60, step: 0.5, digits: 1, path: ['alignment', 'lbjHeight'] },
  { key: 'ubjHeight', label: 'Upper ball joint height', unit: 'mm', min: 100, max: 135, step: 0.5, digits: 1, path: ['alignment', 'ubjHeight'] },
  { key: 'rideHeight', label: 'Ride height', unit: 'mm', min: -8, max: 8, step: 0.5, digits: 1, path: ['rideHeight'], sub: () => `${state.axle === 'front' ? (setup.front.rod === 'push' ? 'Pushrod' : 'Pullrod') : (setup.rear.rod === 'push' ? 'Pushrod' : 'Pullrod')} length ${fmt(car[state.axle === 'front' ? 'fl' : 'rl'].rodLength, 1)} mm` },
  { key: 'track', label: 'Track width', unit: 'mm', min: 320, max: 440, step: 1, digits: 0, path: ['track'] },
];
const VEHICLE_SLIDERS = [
  { key: 'wheelbase', label: 'Wheelbase', unit: 'mm', min: 700, max: 860, step: 1, digits: 0 },
  { key: 'cgHeight', label: 'Centre of gravity height', unit: 'mm', min: 40, max: 110, step: 1, digits: 0 },
  { key: 'brakeBiasFront', label: 'Brake balance (front)', unit: '%', min: 0.4, max: 0.75, step: 0.01, format: (v) => `${Math.round(v * 100)} %` },
  { key: 'rackPerRev', label: 'Rack travel per turn', unit: 'mm', min: 15, max: 60, step: 1, digits: 0 },
];
const getPath = (obj, path) => path.reduce((o, k) => o[k], obj);
const setPath = (obj, path, v) => { const last = path.at(-1); path.slice(0, -1).reduce((o, k) => o[k], obj)[last] = v; };
const showHelp = (cfg) => { $('setupHelp').textContent = HELP[cfg.key] || ''; };

const setupSliders = AXLE_SLIDERS.map((cfg) => makeSlider($('setupSliders'), { ...cfg, help: HELP[cfg.key] },
  () => getPath(setup[state.axle], cfg.path),
  (v) => changeSetup((s) => setPath(s[state.axle], cfg.path, v)),
  showHelp));
const vehicleSliders = VEHICLE_SLIDERS.map((cfg) => {
  return makeSlider($('vehicleSliders'), { ...cfg, help: HELP[cfg.key] },
    () => setup.vehicle[cfg.key],
    (v) => changeSetup((s) => {
      if (cfg.key === 'wheelbase') {
        // move the whole rear axle with the wheelbase
        const dx = s.vehicle.wheelbase - v;
        const shift = (p) => { p.x += dx; };
        Object.values(s.rear.points).forEach(shift); Object.values(s.rear.push).forEach(shift); Object.values(s.rear.pull).forEach(shift);
      }
      s.vehicle[cfg.key] = v;
    }),
    showHelp);
});

document.querySelectorAll('[data-axle]').forEach((b) => b.addEventListener('click', () => {
  state.axle = b.dataset.axle;
  state.selected = null; gizmo.detach(); highlightMarker();
  refreshSetupUI(); refreshPointsUI();
  requestRender();
}));
document.querySelectorAll('[data-rod]').forEach((b) => b.addEventListener('click', () => {
  changeSetup((s) => { s[state.axle].rod = b.dataset.rod; });
  $('setupHelp').textContent = HELP.rod;
}));
$('btnResetAxle').addEventListener('click', () => changeSetup((s) => { s[state.axle] = S.cloneSetup(S.DEFAULT_SETUP)[state.axle]; if (state.axle === 'rear') { const dx = S.DEFAULT_SETUP.vehicle.wheelbase - s.vehicle.wheelbase; const sh = (p) => { p.x += dx; }; Object.values(s.rear.points).forEach(sh); Object.values(s.rear.push).forEach(sh); Object.values(s.rear.pull).forEach(sh); } }));
$('btnResetAll').addEventListener('click', () => changeSetup((s) => { const d = S.cloneSetup(S.DEFAULT_SETUP); for (const k in d) s[k] = d[k]; }));

function refreshSetupUI() {
  document.querySelectorAll('[data-axle]').forEach((b) => b.classList.toggle('on', b.dataset.axle === state.axle));
  document.querySelectorAll('[data-rod]').forEach((b) => b.classList.toggle('on', b.dataset.rod === setup[state.axle].rod));
  setupSliders.forEach((s) => s.show());
  vehicleSliders.forEach((s) => s.show());
}

// Points tab
const POINT_ORDER = ['lcaFront', 'lcaRear', 'ucaFront', 'ucaRear', 'lbj', 'ubj', 'tieRodInner', 'tieRodOuter', 'rodOuter', 'rockerPivot', 'rockerAxis', 'rockerRod', 'rockerDamper', 'damperChassis'];
function pointName(axle, key) {
  const info = S.POINT_INFO[key];
  let n = axle === 'rear' && info.nameRear ? info.nameRear : info.name;
  if (key === 'rodOuter') n = setup[axle].rod === 'push' ? 'Pushrod – lower wishbone end' : 'Pullrod – upper wishbone end';
  return n;
}
function buildPointList() {
  const list = $('pointList');
  list.innerHTML = '';
  const pts = S.cornerPoints(setup, state.axle);
  for (const key of POINT_ORDER) {
    const info = S.POINT_INFO[key];
    const row = document.createElement('div');
    row.className = 'pt';
    row.dataset.key = key;
    const col = '#' + GROUP_COLORS[info.group].toString(16).padStart(6, '0');
    row.innerHTML = `<span class="dot" style="background:${col}"></span><span class="name">${pointName(state.axle, key)}${info.derived ? ' <small>· sets KPI, caster, scrub, trail</small>' : ''}</span>
      <div class="xyz">${['x', 'y', 'z'].map((a) => `<input type="number" step="0.5" data-a="${a}" value="${pts[key][a].toFixed(1)}" aria-label="${a}">`).join('')}</div>`;
    row.addEventListener('click', (e) => { if (e.target.tagName !== 'INPUT') selectPoint(state.axle, key); });
    row.querySelectorAll('input').forEach((inp) => {
      inp.addEventListener('focus', () => selectPoint(state.axle, key, false));
      inp.addEventListener('change', () => {
        const cur = S.cornerPoints(setup, state.axle)[key];
        const v = parseFloat(inp.value);
        if (!Number.isFinite(v)) { refreshPointsUI(); return; }
        applyPoint(state.axle, key, { ...cur, [inp.dataset.a]: v });
        placeGizmo();
      });
    });
    list.appendChild(row);
  }
  highlightRow();
}
function refreshPointsUI(live = false) {
  const pts = S.cornerPoints(setup, state.axle);
  const rows = $('pointList').querySelectorAll('.pt');
  if (!rows.length || live === 'rebuild') { buildPointList(); return; }
  rows.forEach((row) => {
    const key = row.dataset.key;
    row.querySelector('.name').firstChild.textContent = pointName(state.axle, key);
    row.querySelectorAll('input').forEach((inp) => { if (document.activeElement !== inp) inp.value = pts[key][inp.dataset.a].toFixed(1); });
  });
  if (state.selected && !live) placeGizmo();
}
function highlightRow() {
  $('pointList').querySelectorAll('.pt').forEach((r) => r.classList.toggle('sel', !!state.selected && state.selected.axle === state.axle && state.selected.key === r.dataset.key));
}
function placeGizmo() {
  if (!state.selected) return;
  const p = S.cornerPoints(setup, state.selected.axle)[state.selected.key];
  gizmoTarget.position.copy(V(p));
}
function selectPoint(axle, key, scroll = true) {
  if (axle !== state.axle) { state.axle = axle; refreshSetupUI(); buildPointList(); }
  state.selected = { axle, key };
  placeGizmo();
  gizmo.attach(gizmoTarget);
  highlightMarker();
  highlightRow();
  if (scroll) $('pointList').querySelector(`[data-key="${key}"]`)?.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
  requestRender();
}

$('btnShare').addEventListener('click', async () => {
  const link = shareLink();
  history.replaceState(null, '', link);
  try { await navigator.clipboard.writeText(link); toast('Link copied: anyone can open this exact setup.'); }
  catch (e) { toast('Link ready in the address bar.'); }
});
$('btnExport').addEventListener('click', () => {
  const blob = new Blob([JSON.stringify(setup, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'suspension-setup.json';
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
});
$('btnImport').addEventListener('click', () => $('fileImport').click());
$('fileImport').addEventListener('change', async (e) => {
  const f = e.target.files[0];
  if (!f) return;
  try {
    const data = JSON.parse(await f.text());
    if (changeSetup((s) => { const d = mergeSetup(S.cloneSetup(S.DEFAULT_SETUP), data); for (const k in d) s[k] = d[k]; })) toast('Setup imported.');
  } catch (err) { toast('Could not read this file.'); }
  e.target.value = '';
});

// Analysis tab
const METRICS = [
  { key: 'rc', label: 'Roll centre height', unit: 'mm', help: 'Height of the roll centre above the road (static). The body rolls about the line joining the front and rear roll centres.' },
  { key: 'camberGain', label: 'Camber gain', unit: '°/10 mm', d: 2, help: 'Camber change for 10 mm of bump. Negative = the wheel leans in more as it compresses.' },
  { key: 'bumpSteer', label: 'Bump steer', unit: '°/10 mm', d: 2, help: 'Toe change for 10 mm of bump. Ideally close to zero: set by the height and length of the tie rod / toe link.' },
  { key: 'mr', label: 'Motion ratio', unit: '', d: 3, help: 'Damper travel divided by wheel travel. The wheel rate is the spring rate multiplied by the square of this ratio.' },
  { key: 'caster', label: 'Caster', unit: '°', d: 1, help: HELP.caster },
  { key: 'kpi', label: 'KPI', unit: '°', d: 1, help: HELP.kpi },
  { key: 'scrub', label: 'Scrub radius', unit: 'mm', help: HELP.scrub },
  { key: 'trail', label: 'Mechanical trail', unit: 'mm', help: HELP.trail },
  { key: 'anti', label: 'Anti-dive / anti-squat', unit: '%', d: 0, help: 'Front: share of the braking pitch resisted by the wishbone geometry (anti-dive). Rear: share of the acceleration squat resisted (anti-squat).' },
  { key: 'ack', label: 'Ackermann (90° steer)', unit: '%', d: 0, help: '100 % = ideal Ackermann (inner wheel steers more), 0 % = parallel steer, negative = anti-Ackermann.' },
];
$('metricTable').innerHTML = METRICS.map((m) => `<tr><td title="${m.help}">${m.label}${m.unit ? ` <small>(${m.unit})</small>` : ''}</td><td id="mf-${m.key}"></td><td id="mr-${m.key}"></td></tr>`).join('');
let curves = { front: [], rear: [] };
function gainPer10(c, prop) {
  const a = S.cornerAtTravel(c, -5), b = S.cornerAtTravel(c, 5);
  return a && b ? b[prop] - a[prop] : NaN;
}
function refreshAnalysis() {
  curves = { front: S.travelCurves(car.fl, 30), rear: S.travelCurves(car.rl, 30) };
  const st = S.solveCar(car, {});
  const anti = S.antiFeatures(car);
  const steer = S.steeringCurve(car, 4, 90).at(-1);
  const vals = {
    front: { rc: st.rcFront?.point.z, camberGain: gainPer10(car.fl, 'camber'), bumpSteer: gainPer10(car.fl, 'toe'), mr: st.fl.motionRatio, caster: st.fl.caster, kpi: st.fl.kpi, scrub: st.fl.scrub, trail: st.fl.trail, anti: anti.antiDive, ack: steer?.ackermann },
    rear: { rc: st.rcRear?.point.z, camberGain: gainPer10(car.rl, 'camber'), bumpSteer: gainPer10(car.rl, 'toe'), mr: st.rl.motionRatio, caster: st.rl.caster, kpi: st.rl.kpi, scrub: st.rl.scrub, trail: st.rl.trail, anti: anti.antiSquat, ack: NaN },
  };
  for (const m of METRICS) {
    $(`mf-${m.key}`).textContent = fmt(vals.front[m.key], m.d ?? 1);
    $(`mr-${m.key}`).textContent = fmt(vals.rear[m.key], m.d ?? 1);
  }
  drawCharts();
}

function drawChart(canvas, title, unit, prop, digits) {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  if (canvas.width !== Math.round(w * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  const pad = { l: 40, r: 8, t: 18, b: 18 };
  const all = [...curves.front, ...curves.rear].map((p) => p[prop]).filter(Number.isFinite);
  if (!all.length) return;
  let lo = Math.min(...all), hi = Math.max(...all);
  const span = Math.max(hi - lo, prop === 'mr' ? 0.05 : 0.2);
  const mid = (hi + lo) / 2; lo = mid - span * 0.6; hi = mid + span * 0.6;
  const v = setup.vehicle, x0 = -v.travelDroop, x1 = v.travelBump;
  const X = (t) => pad.l + ((t - x0) / (x1 - x0)) * (w - pad.l - pad.r);
  const Y = (y) => pad.t + (1 - (y - lo) / (hi - lo)) * (h - pad.t - pad.b);
  ctx.font = '11px system-ui, sans-serif';
  ctx.fillStyle = '#ecebe6';
  ctx.fillText(`${title} (${unit})`, pad.l, 12);
  ctx.strokeStyle = 'rgba(255,255,255,0.12)'; ctx.lineWidth = 1;
  ctx.fillStyle = '#a3a29b';
  for (let i = 0; i <= 3; i++) {
    const y = lo + ((hi - lo) * i) / 3, py = Y(y);
    ctx.beginPath(); ctx.moveTo(pad.l, py); ctx.lineTo(w - pad.r, py); ctx.stroke();
    ctx.fillText(y.toFixed(digits), 2, py + 4);
  }
  ctx.beginPath(); ctx.moveTo(X(0), pad.t); ctx.lineTo(X(0), h - pad.b); ctx.stroke();
  ctx.fillText(`droop ${x0} mm`, pad.l, h - 4);
  ctx.textAlign = 'right'; ctx.fillText(`bump +${x1} mm`, w - pad.r, h - 4); ctx.textAlign = 'left';
  const sol = state.solution;
  for (const [axle, color, key] of [['front', '#afa9ec', 'fl'], ['rear', '#ef9f27', 'rl']]) {
    const pts = curves[axle].filter((p) => Number.isFinite(p[prop]));
    ctx.strokeStyle = color; ctx.lineWidth = 2;
    ctx.beginPath();
    pts.forEach((p, i) => (i ? ctx.lineTo(X(p.travel), Y(p[prop])) : ctx.moveTo(X(p.travel), Y(p[prop]))));
    ctx.stroke();
    const s = sol?.[key];
    if (s && pts.length > 1) {
      // interpolate the curve at the current travel
      const t = THREE.MathUtils.clamp(s.travel, x0, x1);
      let y = pts[0][prop];
      for (let i = 1; i < pts.length; i++) if (pts[i].travel >= t) { const a = pts[i - 1], b = pts[i]; y = a[prop] + ((b[prop] - a[prop]) * (t - a.travel)) / (b.travel - a.travel || 1); break; }
      ctx.fillStyle = color;
      ctx.beginPath(); ctx.arc(X(t), Y(y), 4, 0, Math.PI * 2); ctx.fill();
    }
  }
}
function drawCharts() {
  if (state.tab !== 'analysis') return;
  drawChart($('chartCamber'), 'Camber', '°', 'camber', 2);
  drawChart($('chartToe'), 'Toe / bump steer', '°', 'toe', 2);
  drawChart($('chartMR'), 'Motion ratio', '–', 'mr', 3);
}

// Display buttons
const BODY_MODES = ['ghost', 'solid', 'hidden'];
$('btnBody').addEventListener('click', () => { bodyMode = BODY_MODES[(BODY_MODES.indexOf(bodyMode) + 1) % 3]; applyBodyMode(); requestRender(); });
$('btnOverlay').addEventListener('click', (e) => { state.overlay = !state.overlay; overlay.visible = state.overlay; e.currentTarget.classList.toggle('on', state.overlay); labelsDirty = true; requestRender(); });
$('btnLabels').addEventListener('click', (e) => { state.labels = !state.labels; e.currentTarget.classList.toggle('on', state.labels); requestRender(); });

// Collapsible panel (bottom sheet on phones)
const panel = $('panel');
function togglePanel() {
  const collapsed = panel.classList.toggle('collapsed');
  $('panelToggle').setAttribute('aria-expanded', String(!collapsed));
  $('panelToggleText').textContent = collapsed ? 'Show controls' : 'Hide controls';
  updateViewOffset();
  requestRender();
}
$('panelToggle').addEventListener('click', togglePanel);
if (isSmallScreen()) { panel.classList.add('collapsed'); $('panelToggleText').textContent = 'Show controls'; }

// Picking pickup points in 3D
const raycaster = new THREE.Raycaster();
let downAt = null;
renderer.domElement.addEventListener('pointerdown', (e) => { downAt = { x: e.clientX, y: e.clientY, t: performance.now() }; });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downAt || !markers.visible || gizmo.dragging) return;
  const moved = Math.hypot(e.clientX - downAt.x, e.clientY - downAt.y);
  if (moved > 6 || performance.now() - downAt.t > 600) return;
  const r = renderer.domElement.getBoundingClientRect();
  const ndc = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  raycaster.setFromCamera(ndc, camera);
  const hit = raycaster.intersectObjects(markerList, false)[0];
  if (hit) selectPoint(hit.object.userData.axle, hit.object.userData.key);
});

// ---------------------------------------------------------------------------
// Labels (HTML elements projected from 3D anchors, ISO coordinates)
// ---------------------------------------------------------------------------
const labels = [];
let labelsDirty = true;
function addLabel(text, anchor, cls = '') {
  const el = document.createElement('div');
  el.className = 'label ' + cls;
  el.textContent = text;
  document.body.appendChild(el);
  labels.push({ el, anchor });
}
const sol = () => state.solution;
const mid = (a, b, t = 0.5) => S.lerp(a, b, t);
addLabel('Upper wishbone', () => sol()?.fl && mid(car.fl.P.ucaRear, sol().fl.ubj, 0.45), 'minor');
addLabel('Lower wishbone', () => sol()?.fl && mid(car.fl.P.lcaFront, sol().fl.lbj, 0.45), 'minor');
addLabel('Tie rod', () => sol()?.fl && S.add(mid(sol().fl.tri, sol().fl.tro, 0.55), S.vec(14, 0, 6)));
addLabel('Steering rack', () => S.add(car.fl.P.tieRodInner, S.vec(16, -48, -10)), 'minor');
addLabel(setup.front.rod === 'push' ? 'Pushrod' : 'Pullrod', () => sol()?.fl && mid(sol().fl.rodOuter, sol().fl.rockerRod, 0.4));
addLabel('Rocker + servo', () => S.add(car.fl.P.rockerPivot, S.vec(0, 0, 26)));
addLabel('Damper', () => sol()?.fl && S.add(mid(car.fl.P.damperChassis, sol().fl.rockerDamper), S.vec(0, 0, 12)), 'minor');
addLabel('Upright', () => sol()?.rl && S.add(sol().rl.ubj, S.vec(0, 6, 22)), 'minor');
addLabel('Toe link', () => sol()?.rl && S.add(mid(sol().rl.tri, sol().rl.tro, 0.6), S.vec(-12, 0, -8)), 'minor');
addLabel('Driveshaft', () => sol()?.rl && S.vec(sol().rl.wc.x + 10, 90, 76), 'minor');
addLabel('Roll centre F', () => (state.overlay && sol()?.rcFront ? S.add(sol().rcFront.point, S.vec(0, 0, -14)) : null), 'rc');
addLabel('Roll centre R', () => (state.overlay && sol()?.rcRear ? S.add(sol().rcRear.point, S.vec(0, 0, -14)) : null), 'rc');
// keep rod labels in sync with the rod type
function refreshLabelTexts() { labels[4].el.textContent = setup.front.rod === 'push' ? 'Front pushrod' : 'Front pullrod'; }

const tmpL = new THREE.Vector3();
function updateLabels() {
  const w = container.clientWidth, h = container.clientHeight;
  for (const { el, anchor } of labels) {
    const p = state.labels && state.tab !== 'points' ? anchor() : null;
    if (!p) { el.style.display = 'none'; continue; }
    tmpL.set(p.x, p.y, p.z);
    chassis.localToWorld(tmpL).project(camera);
    const visible = tmpL.z < 1 && Math.abs(tmpL.x) < 1.02 && Math.abs(tmpL.y) < 1.02;
    el.style.display = visible ? '' : 'none';
    if (visible) el.style.transform = `translate(${((tmpL.x + 1) / 2) * w}px, ${((1 - tmpL.y) / 2) * h}px) translate(-50%, -50%)`;
  }
  labelsDirty = false;
}

// ---------------------------------------------------------------------------
// Camera views (auto-framed for the area left visible by the control panel)
// ---------------------------------------------------------------------------
const FRAME = { center: new THREE.Vector3(-39, 9, 0), radius: 72 };
const VIEWS = {
  iso: { dir: [0.95, 0.62, 0.9] },
  front: { dir: [1, 0.14, 0] },
  side: { dir: [0, 0.12, 1] },
  top: { dir: [0.001, 1, 0] },
  corner: { pos: [26, 20, -46], target: [-1, 7, -13] },
  rocker: { pos: [16, 34, -26], target: [-1.5, 14, -3] },
};
const panelOffset = () => (!isSmallScreen() && !panel.classList.contains('collapsed') ? panel.offsetWidth + 14 : 0);
function updateViewOffset() {
  const w = container.clientWidth, h = container.clientHeight, off = panelOffset();
  if (off) camera.setViewOffset(w, h, off / 2, 0, w, h); else camera.clearViewOffset();
  camera.updateProjectionMatrix();
}
function fitDistance() {
  const w = Math.max(container.clientWidth - panelOffset(), 1), h = Math.max(container.clientHeight * (isSmallScreen() ? 0.6 : 1), 1);
  const vFov = THREE.MathUtils.degToRad(camera.fov);
  const hFov = 2 * Math.atan(Math.tan(vFov / 2) * (w / h));
  return (FRAME.radius / Math.sin(Math.min(vFov, hFov) / 2)) * 0.82;
}
let camTween = null;
function goToView(name, instant = false) {
  const v = VIEWS[name];
  const to = v.dir
    ? { target: FRAME.center.clone(), pos: FRAME.center.clone().addScaledVector(new THREE.Vector3(...v.dir).normalize(), fitDistance()) }
    : { pos: new THREE.Vector3(...v.pos), target: new THREE.Vector3(...v.target) };
  if (!v.dir && isSmallScreen()) to.pos.sub(to.target).multiplyScalar(1.5).add(to.target);
  if (isSmallScreen()) to.target.y -= 6; // the bottom sheet hides the lower part of the screen
  if (instant) { camera.position.copy(to.pos); controls.target.copy(to.target); controls.update(); return; }
  camTween = { from: { pos: camera.position.clone(), target: controls.target.clone() }, to, t: 0 };
  requestRender();
}
document.querySelectorAll('[data-view]').forEach((b) => b.addEventListener('click', () => goToView(b.dataset.view)));

// ---------------------------------------------------------------------------
// Render loop (on demand, to save battery on mobile devices)
// ---------------------------------------------------------------------------
let running = false, last = performance.now(), lastPhase = state.phase, chartTimer = 0;
function requestRender() { if (!running) { running = true; last = performance.now(); requestAnimationFrame(tick); } }

function stepCamera(dt) {
  if (!camTween) return false;
  camTween.t = Math.min(camTween.t + dt / 0.8, 1);
  const k = 1 - Math.pow(1 - camTween.t, 3);
  camera.position.lerpVectors(camTween.from.pos, camTween.to.pos, k);
  controls.target.lerpVectors(camTween.from.target, camTween.to.target, k);
  if (camTween.t >= 1) camTween = null;
  return true;
}

function tick(now) {
  const dt = Math.min((now - last) / 1000, 0.05);
  last = now;
  const busy = stepSequence(dt);
  const s = solve();
  updateVisuals(s);
  const moving = stepCamera(dt);
  const orbiting = controls.update();
  renderer.render(scene, camera);
  updateLabels();
  refreshWheelTable(s);
  chartTimer += dt;
  if (state.tab === 'analysis' && (chartTimer > 0.1 || !busy)) { chartTimer = 0; drawCharts(); }
  if (state.phase !== lastPhase) { lastPhase = state.phase; refreshDriveUI(); syncDriveSliders(); }
  if (state.scenario) syncDriveSliders();
  if (busy || moving || orbiting) requestAnimationFrame(tick); else running = false;
}

controls.addEventListener('change', requestRender);
controls.addEventListener('start', requestRender);

function resize() {
  const w = container.clientWidth, h = container.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  updateViewOffset();
  drawCharts();
  requestRender();
}
new ResizeObserver(resize).observe(container);

// ---------------------------------------------------------------------------
// Start
// ---------------------------------------------------------------------------
applyBodyMode();
rebuildStaticVisuals();
rebuildMarkers();
buildPointList();
refreshSetupUI();
refreshDriveUI();
refreshLabelTexts();
refreshAnalysis();
resize();
goToView('iso', true);
document.getElementById('loading').classList.add('done');
window.addEventListener('hashchange', () => { setup = loadInitialSetup(); changeSetup(() => {}); });
