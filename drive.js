// Track test: drive the car with the lab setup and read live telemetry.
//
// Physics: dynamics.js (full-size planar car + live suspension kinematics from suspension.js).
// Rendering: Three.js, 1 unit = 1 m. World plane: X east, Y north (three.z = -Y).

import * as THREE from 'three';
import * as S from './suspension.js';
import { buildBody } from './body.js';
import { VehicleSim, DEFAULT_DRIVE, WHEELS, SCALE, SURFACES, PEAK_SLIP, ENGINE } from './dynamics.js';
import { CarAudio } from './audio.js';
import { TelemetryLog, download } from './motec.js';
import { TRACK, locate } from './track.js';

const $ = (id) => document.getElementById(id);
const DEG = Math.PI / 180;
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const fmt = (x, d = 1) => (Number.isFinite(x) ? x.toFixed(d) : '–');
const KERB_K = 1 / 220; // curvature above which a corner gets kerbs

// ---------------------------------------------------------------------------
// Setup (shared with the lab) and drive settings
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
function loadSetup() {
  const s = S.cloneSetup(S.DEFAULT_SETUP);
  try {
    const m = location.hash.match(/s=([^&]+)/);
    if (m) return mergeSetup(s, JSON.parse(decodeURIComponent(escape(atob(decodeURIComponent(m[1]))))));
  } catch (e) { /* broken link */ }
  try { const raw = localStorage.getItem('as3d-setup-v2'); if (raw) return mergeSetup(s, JSON.parse(raw)); } catch (e) { /* no storage */ }
  return s;
}
function loadDrive() {
  try { const raw = localStorage.getItem('as3d-drive-v2'); if (raw) return { ...DEFAULT_DRIVE, ...JSON.parse(raw) }; } catch (e) { /* no storage */ }
  return { ...DEFAULT_DRIVE };
}
let setup = loadSetup();
let drive = loadDrive();
try { S.buildCar(setup); } catch (e) { setup = S.cloneSetup(S.DEFAULT_SETUP); }
const encodeSetup = () => encodeURIComponent(btoa(unescape(encodeURIComponent(JSON.stringify(setup)))));
function saveAll() {
  try {
    localStorage.setItem('as3d-drive-v2', JSON.stringify(drive));
    localStorage.setItem('as3d-setup-v2', JSON.stringify(setup));
  } catch (e) { /* ignore */ }
  const href = `index.html#s=${encodeSetup()}`;
  $('backLink').href = href;
  $('editGeometry').href = href;
}

const sim = new VehicleSim(setup, drive);
// start a little before the line so the first lap is timed
const gridPose = () => { const p = TRACK.pts[TRACK.pts.length - 30]; return { x: p.x, y: p.y, psi: Math.atan2(p.ty, p.tx) }; };
sim.reset(gridPose());

// ---------------------------------------------------------------------------
// Renderer, scene, lights
// ---------------------------------------------------------------------------
const isTouch = matchMedia('(pointer: coarse)').matches || 'ontouchstart' in window;
const small = () => matchMedia('(max-width: 720px), (max-height: 520px)').matches;
const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(devicePixelRatio, isTouch ? 1.5 : 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
$('viewport').appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x8fa6c2);
scene.fog = new THREE.Fog(0x8fa6c2, 250, 1100);
const camera = new THREE.PerspectiveCamera(62, 1, 0.05, 2500);

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x3a4a2a, 0.9));
const sun = new THREE.DirectionalLight(0xffffff, 2.2);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -14, right: 14, top: 14, bottom: -14, near: 1, far: 120 });
sun.shadow.bias = -0.0004;
scene.add(sun, sun.target);

// ---------------------------------------------------------------------------
// Circuit
// ---------------------------------------------------------------------------
const W = (x, y, h = 0) => new THREE.Vector3(x, h, -y);
{
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(4000, 4000), new THREE.MeshStandardMaterial({ color: 0x4d6b3a, roughness: 1 }));
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);

  const { pts, width } = TRACK, n = pts.length, hw = width / 2;
  // ribbon helper between two lateral offsets, optional per-sample colour
  const ribbon = (o1, o2, h, colorAt, mat, filter = () => true) => {
    const pos = [], col = [], idx = [];
    let v = 0;
    for (let i = 0; i < n; i++) {
      const a = pts[i], b = pts[(i + 1) % n];
      if (!filter(a, i)) continue;
      const c = colorAt ? colorAt(i) : null;
      for (const [p, o] of [[a, o1], [a, o2], [b, o1], [b, o2]]) {
        pos.push(p.x + p.nx * o, h, -(p.y + p.ny * o));
        if (c) col.push(c.r, c.g, c.b);
      }
      idx.push(v, v + 1, v + 2, v + 1, v + 3, v + 2);
      v += 4;
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    if (colorAt) g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
    g.setIndex(idx);
    g.computeVertexNormals();
    const m = new THREE.Mesh(g, mat);
    m.receiveShadow = true;
    scene.add(m);
    return m;
  };
  ribbon(hw, -hw, 0.02, null, new THREE.MeshStandardMaterial({ color: 0x3b3c42, roughness: 0.92 }));
  ribbon(hw, hw - 0.25, 0.03, null, new THREE.MeshStandardMaterial({ color: 0xf2f2ee, roughness: 0.8 }));
  ribbon(-hw + 0.25, -hw, 0.03, null, new THREE.MeshStandardMaterial({ color: 0xf2f2ee, roughness: 0.8 }));
  // kerbs on both sides through the corners
  const red = new THREE.Color(0xd23b3b), white = new THREE.Color(0xf0f0ea);
  const kerbMat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.7 });
  const inCorner = (p) => Math.abs(p.k) > KERB_K;
  ribbon(hw + 1.4, hw, 0.04, (i) => (Math.floor(i / 2) % 2 ? red : white), kerbMat, inCorner);
  ribbon(-hw, -hw - 1.4, 0.04, (i) => (Math.floor(i / 2) % 2 ? red : white), kerbMat, inCorner);
  // start / finish line (chequered)
  const cv = document.createElement('canvas'); cv.width = 64; cv.height = 8;
  const cx = cv.getContext('2d');
  for (let i = 0; i < 16; i++) for (let j = 0; j < 2; j++) { cx.fillStyle = (i + j) % 2 ? '#111' : '#fff'; cx.fillRect(i * 4, j * 4, 4, 4); }
  const tex = new THREE.CanvasTexture(cv); tex.magFilter = THREE.NearestFilter;
  const line = new THREE.Mesh(new THREE.PlaneGeometry(1.6, width), new THREE.MeshStandardMaterial({ map: tex, roughness: 0.8 }));
  line.rotation.x = -Math.PI / 2;
  line.rotation.z = Math.atan2(pts[0].ty, pts[0].tx);
  line.position.copy(W(pts[0].x, pts[0].y, 0.035));
  scene.add(line);
  // gantry over the line
  const gm = new THREE.MeshStandardMaterial({ color: 0x2b2a3a, roughness: 0.5, metalness: 0.4 });
  for (const s of [1, -1]) {
    const post = new THREE.Mesh(new THREE.BoxGeometry(0.5, 7, 0.5), gm);
    post.position.copy(W(pts[0].x + pts[0].nx * s * (hw + 3), pts[0].y + pts[0].ny * s * (hw + 3), 3.5));
    post.castShadow = true; scene.add(post);
  }
  const beam = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.9, width + 6.5), new THREE.MeshStandardMaterial({ color: 0x7f77dd, roughness: 0.5 }));
  beam.position.copy(W(pts[0].x, pts[0].y, 7));
  beam.rotation.y = Math.atan2(pts[0].ty, pts[0].tx);
  scene.add(beam);
  // grandstand along the main straight
  const stand = new THREE.Mesh(new THREE.BoxGeometry(160, 6, 14), new THREE.MeshStandardMaterial({ color: 0x5f5e66, roughness: 0.8 }));
  stand.position.copy(W(60, 34, 3)); stand.castShadow = stand.receiveShadow = true; scene.add(stand);
  const roof = new THREE.Mesh(new THREE.BoxGeometry(164, 0.6, 16), new THREE.MeshStandardMaterial({ color: 0x7f77dd, roughness: 0.6 }));
  roof.position.copy(W(60, 36, 11)); scene.add(roof);
  // trees, away from the track
  let seed = 7;
  const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  const spots = [];
  for (let i = 0; i < 2600 && spots.length < 340; i++) {
    const x = -450 + rnd() * 1100, y = -200 + rnd() * 860;
    let d = Infinity;
    for (let j = 0; j < n; j += 4) d = Math.min(d, (pts[j].x - x) ** 2 + (pts[j].y - y) ** 2);
    if (d > 30 * 30 && !(x > -30 && x < 150 && y > 20 && y < 50)) spots.push([x, y, 0.7 + rnd() * 0.8]);
  }
  const crown = new THREE.InstancedMesh(new THREE.ConeGeometry(3, 9, 7), new THREE.MeshStandardMaterial({ color: 0x2f5a2e, roughness: 1 }), spots.length);
  const trunk = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.35, 0.45, 3, 6), new THREE.MeshStandardMaterial({ color: 0x5a4632, roughness: 1 }), spots.length);
  const mtx = new THREE.Matrix4();
  spots.forEach(([x, y, k], i) => {
    mtx.makeScale(k, k, k).setPosition(x, 7.5 * k, -y); crown.setMatrixAt(i, mtx);
    mtx.makeScale(k, k, k).setPosition(x, 1.5 * k, -y); trunk.setMatrixAt(i, mtx);
  });
  crown.castShadow = true;
  scene.add(crown, trunk);
}

// ---------------------------------------------------------------------------
// Car (lab geometry in mm, scaled x4 to metres)
// ---------------------------------------------------------------------------
const mat = (color, o = {}) => new THREE.MeshStandardMaterial({ color, metalness: 0.2, roughness: 0.5, ...o });
const M = {
  body: mat(0x2b2a3a, { roughness: 0.32, metalness: 0.35 }), accent: mat(0x7f77dd, { roughness: 0.35, metalness: 0.3 }),
  carbon: mat(0x1f1f24), dark: mat(0x09090b, { roughness: 0.9 }), cockpit: mat(0x0e0e11, { roughness: 0.8 }),
  halo: mat(0x34343a, { metalness: 0.7, roughness: 0.35 }), helmet: mat(0xef9f27, { roughness: 0.25 }),
  engine: mat(0x55545c, { metalness: 0.7, roughness: 0.4 }), alu: mat(0xbdbcb5, { metalness: 0.85, roughness: 0.3 }),
  tyre: mat(0x18181a, { roughness: 0.92 }), rim: mat(0x9a9a96, { metalness: 0.9, roughness: 0.25, side: THREE.DoubleSide }),
  link: mat(0x3d3d46, { metalness: 0.6, roughness: 0.35 }), rod: mat(0xd8d6ce, { metalness: 0.9, roughness: 0.22 }),
};
const carRoot = new THREE.Group();
scene.add(carRoot);
const isoCar = new THREE.Group();
isoCar.matrixAutoUpdate = false;
carRoot.add(isoCar);
const chassis = new THREE.Group();
chassis.matrixAutoUpdate = false;
isoCar.add(chassis);
function placeIsoCar() {
  const s = SCALE / 1000, a = sim.a;
  isoCar.matrix.set(s, 0, 0, a, 0, 0, s, 0, 0, -s, 0, 0, 0, 0, 0, 1);
  isoCar.matrixWorldNeedsUpdate = true;
}
placeIsoCar();
{
  const { body, inner } = buildBody(M, setup.vehicle.wheelbase);
  body.traverse((o) => { if (o.isMesh) o.castShadow = true; });
  chassis.add(body, inner);
}
const UP = new THREE.Vector3(0, 1, 0), tA = new THREE.Vector3(), tB = new THREE.Vector3();
function rod(r, m, parent) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, 1, 8), m);
  mesh.castShadow = true;
  parent.add(mesh);
  mesh.update = (a, b) => {
    tA.set(a.x, a.y, a.z); tB.set(b.x, b.y, b.z);
    const d = tB.sub(tA), L = d.length();
    mesh.position.copy(tA).addScaledVector(d, 0.5);
    mesh.scale.set(1, Math.max(L, 1e-3), 1);
    if (L > 1e-6) mesh.quaternion.setFromUnitVectors(UP, d.multiplyScalar(1 / L));
  };
  return mesh;
}
function buildWheel(radius, width) {
  const w = new THREE.Group(), hw = width / 2, rc = 9, rimR = 57;
  const prof = [new THREE.Vector2(rimR, -hw + 3)];
  for (let i = 0; i <= 8; i++) { const a = -Math.PI / 2 + (i / 8) * (Math.PI / 2); prof.push(new THREE.Vector2(radius - rc + rc * Math.cos(a), -hw + rc + rc * Math.sin(a))); }
  for (let i = 0; i <= 8; i++) { const a = (i / 8) * (Math.PI / 2); prof.push(new THREE.Vector2(radius - rc + rc * Math.cos(a), hw - rc + rc * Math.sin(a))); }
  prof.push(new THREE.Vector2(rimR, hw - 3));
  const tyre = new THREE.Mesh(new THREE.LatheGeometry(prof, 40), M.tyre);
  tyre.castShadow = true;
  w.add(tyre);
  const stripeMat = new THREE.MeshBasicMaterial({ color: 0x97c459 });
  const stripe = new THREE.Mesh(new THREE.TorusGeometry(radius - 14, 1.6, 6, 48).rotateX(Math.PI / 2), stripeMat);
  stripe.position.y = hw + 0.4;
  w.add(stripe);
  const spin = new THREE.Group();
  w.add(spin);
  spin.add(new THREE.Mesh(new THREE.CylinderGeometry(rimR, rimR, width - 8, 32, 1, true), M.rim));
  for (let i = 0; i < 6; i++) {
    const s = new THREE.Mesh(new THREE.BoxGeometry(6, 4, rimR - 8), M.rim);
    s.position.set(0, hw - 12, 0);
    const g = new THREE.Group(); g.rotation.y = (i / 6) * Math.PI * 2; s.position.z = (rimR - 8) / 2 + 4; g.add(s); spin.add(g);
  }
  w.userData = { stripeMat, spin };
  return w;
}
const corners = Object.fromEntries(WHEELS.map((k) => {
  const ax = setup[k[0] === 'f' ? 'front' : 'rear'];
  const c = { wheel: buildWheel(ax.tyreRadius, ax.tyreWidth), legs: [0, 1, 2, 3].map(() => rod(3.4, M.link, chassis)), tie: rod(2.6, M.link, chassis), rod: rod(3, M.rod, chassis), spinAngle: 0 };
  chassis.add(c.wheel);
  return [k, c];
}));
const helmetLocal = new THREE.Vector3(-262, 0, 205); // driver's eyes, lab mm (ISO)

function updateCar() {
  const sol = sim.kin;
  // kerbs and grass shake the car a little
  const kerbN = sim.wheels.filter((w) => w.surface === SURFACES.kerb).length, grassN = sim.wheels.filter((w) => w.surface === SURFACES.grass).length;
  const shake = paused ? 0 : Math.min(1, sim.speed / 25) * (kerbN * 0.006 + grassN * 0.01);
  carRoot.position.set(sim.x, shake ? (Math.random() - 0.5) * shake : 0, -sim.y);
  carRoot.rotation.y = sim.psi;
  if (!sol) return;
  const { R, ref, h } = sol.pose;
  const Rr = (v) => S.vec(R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z, R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z, R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z);
  const t = S.sub(ref, Rr(ref));
  chassis.matrix.set(R[0][0], R[0][1], R[0][2], t.x, R[1][0], R[1][1], R[1][2], t.y, R[2][0], R[2][1], R[2][2], t.z + h, 0, 0, 0, 1);
  chassis.matrixWorldNeedsUpdate = true;
  WHEELS.forEach((k, i) => {
    const s = sol[k], c = corners[k], P = sim.g.car[k].P, w = sim.wheels[i];
    if (!s) return;
    const a = S.norm(s.axis), kp = S.norm(S.sub(s.ubj, s.lbj));
    const zb = S.norm(S.sub(kp, S.scale(a, S.dot(kp, a)))), xb = S.cross(a, zb);
    c.wheel.position.set(s.wc.x, s.wc.y, s.wc.z);
    c.wheel.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(new THREE.Vector3(xb.x, xb.y, xb.z), new THREE.Vector3(a.x, a.y, a.z), new THREE.Vector3(zb.x, zb.y, zb.z)));
    c.legs[0].update(P.lcaFront, s.lbj); c.legs[1].update(P.lcaRear, s.lbj);
    c.legs[2].update(P.ucaFront, s.ubj); c.legs[3].update(P.ucaRear, s.ubj);
    c.tie.update(s.tri, s.tro);
    c.rod.update(s.rodOuter, s.rockerRod);
    // wheel rotation and grip colour on the sidewall stripe
    c.spinAngle += (sim.vx / sim.g.tyreRadius) * frameDt * (w.side > 0 ? -1 : 1);
    c.wheel.userData.spin.rotation.y = c.spinAngle;
    c.wheel.userData.stripeMat.color.copy(gripColor(w));
  });
}
const cGreen = new THREE.Color(0x97c459), cYellow = new THREE.Color(0xfac775), cRed = new THREE.Color(0xe24b4a), cGrey = new THREE.Color(0x666666), tmpC = new THREE.Color();
function gripColor(w) {
  if (w.fz < 50) return cGrey;
  const u = clamp(w.util, 0, 1);
  return u < 0.75 ? tmpC.copy(cGreen).lerp(cYellow, u / 0.75) : tmpC.copy(cYellow).lerp(cRed, (u - 0.75) / 0.25);
}

// ---------------------------------------------------------------------------
// Cameras
// ---------------------------------------------------------------------------
const CAMS = ['chase', 'cockpit', 'tv', 'top'];
let camMode = 0;
const camPos = new THREE.Vector3(), camLook = new THREE.Vector3();
function updateCamera(dt, snap = false) {
  const fwd = new THREE.Vector3(Math.cos(sim.psi), 0, -Math.sin(sim.psi));
  const car = carRoot.position;
  let pos, look;
  switch (CAMS[camMode]) {
    case 'cockpit': {
      carRoot.updateMatrixWorld(true);
      pos = chassis.localToWorld(helmetLocal.clone());
      look = pos.clone().addScaledVector(fwd, 10).add(new THREE.Vector3(0, -0.6, 0));
      camera.position.copy(pos); camera.lookAt(look);
      return;
    }
    case 'tv': pos = car.clone().add(new THREE.Vector3(-fwd.z * 14 - fwd.x * 6, 7, fwd.x * 14 - fwd.z * 6)); look = car.clone(); break;
    case 'top': pos = car.clone().add(new THREE.Vector3(0.01, 70, 0)); look = car.clone(); break;
    default: pos = car.clone().addScaledVector(fwd, -7.2).add(new THREE.Vector3(0, 2.3, 0)); look = car.clone().addScaledVector(fwd, 3).add(new THREE.Vector3(0, 0.8, 0));
  }
  const k = snap ? 1 : 1 - Math.exp(-dt * (CAMS[camMode] === 'chase' ? 7 : 4));
  camPos.lerp(pos, k); camLook.lerp(look, k);
  if (snap) { camPos.copy(pos); camLook.copy(look); }
  camera.position.copy(camPos);
  camera.lookAt(camLook);
}

// ---------------------------------------------------------------------------
// Inputs: keyboard, gamepad, touch
// ---------------------------------------------------------------------------
const keys = new Set();
const input = { steer: 0, throttle: 0, brake: 0 };   // driver's pedals and steering
const simIn = { steer: 0, throttle: 0, brake: 0 };   // what the car receives (pedals swap in reverse)
let reverse = false, reverseHold = 0, reverseArmed = false;
const audio = new CarAudio();
try { audio.enabled = localStorage.getItem('as3d-drive-sound') !== 'off'; } catch (e) { /* ignore */ }
// browsers only start audio after a click, a tap or a key press
const wakeAudio = () => { if (audio.enabled) audio.start(); updateSoundBtn(); };
addEventListener('pointerdown', wakeAudio);
let padIndex = null, padPrev = [];
const touch = { steer: 0, throttle: 0, brake: 0, active: false };

addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  keys.add(e.code);
  wakeAudio();
  if (!$('help').hidden && ['Enter', 'Space', 'ArrowUp', 'KeyW'].includes(e.code)) { startDriving(); e.preventDefault(); return; }
  switch (e.code) {
    case 'KeyR': resetCar(); break;
    case 'KeyC': cycleCam(); break;
    case 'KeyT': toggleTelemetry(); break;
    case 'KeyB': toggleSetup(); break;
    case 'KeyH': showHelp(); break;
    case 'KeyP': case 'Escape': togglePause(); break;
    case 'KeyE': shiftGear(1); break;
    case 'KeyQ': shiftGear(-1); break;
    case 'KeyM': toggleSound(); break;
    case 'KeyL': exportMotec(); break;
    default: break;
  }
  if (e.code.startsWith('Arrow') || e.code === 'Space') e.preventDefault();
});
addEventListener('keyup', (e) => keys.delete(e.code));
addEventListener('blur', () => keys.clear());

addEventListener('gamepadconnected', (e) => { padIndex = e.gamepad.index; toast(`Controller connected: ${e.gamepad.id.split('(')[0].trim()}`); updatePadStatus(); });
addEventListener('gamepaddisconnected', (e) => { if (padIndex === e.gamepad.index) padIndex = null; toast('Controller disconnected'); updatePadStatus(); });
function updatePadStatus() {
  const el = $('padStatus');
  const pad = padIndex !== null && navigator.getGamepads?.()[padIndex];
  el.textContent = pad ? `Connected: ${pad.id.split('(')[0].trim()}` : 'No controller detected: plug it in and press a button.';
  el.classList.toggle('ok', !!pad);
}
function readPad() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  let pad = padIndex !== null ? pads[padIndex] : null;
  if (!pad) for (const p of pads) if (p && p.connected) { pad = p; padIndex = p.index; updatePadStatus(); break; }
  if (!pad) return null;
  const b = (i) => pad.buttons[i] ? pad.buttons[i].value : 0;
  const pressed = (i) => !!pad.buttons[i]?.pressed;
  const edge = (i) => pressed(i) && !padPrev[i];
  if (edge(0)) { if (!$('help').hidden) startDriving(); else resetCar(); }
  if (pad.buttons.some((x) => x.pressed)) wakeAudio();
  if (edge(3)) cycleCam();
  if (edge(2)) toggleTelemetry();
  if (edge(1)) toggleSetup();
  if (edge(9)) togglePause();
  if (edge(8)) showHelp();
  if (edge(5)) shiftGear(1);   // RB: upshift
  if (edge(4)) shiftGear(-1);  // LB: downshift
  padPrev = pad.buttons.map((x) => x.pressed);
  let x = pad.axes[0] || 0;
  x = Math.abs(x) < 0.07 ? 0 : Math.sign(x) * ((Math.abs(x) - 0.07) / 0.93) ** 1.5;
  return { steer: -x, throttle: b(7), brake: b(6) };
}
// touch
if (isTouch) {
  $('touch').hidden = false;
  const pad = $('touchSteer'), knob = pad.querySelector('i');
  const setSteer = (e) => {
    const r = pad.getBoundingClientRect();
    const x = clamp(((e.clientX - r.left) / r.width) * 2 - 1, -1, 1);
    touch.steer = -x; touch.active = true;
    knob.style.left = `${((x + 1) / 2) * 100}%`;
  };
  pad.addEventListener('pointerdown', (e) => { pad.setPointerCapture(e.pointerId); setSteer(e); });
  pad.addEventListener('pointermove', (e) => { if (e.buttons || e.pointerType === 'touch') setSteer(e); });
  const release = () => { touch.steer = 0; knob.style.left = '50%'; };
  pad.addEventListener('pointerup', release); pad.addEventListener('pointercancel', release);
  for (const [id, key] of [['touchThrottle', 'throttle'], ['touchBrake', 'brake']]) {
    const btn = $(id);
    btn.addEventListener('pointerdown', (e) => { btn.setPointerCapture(e.pointerId); touch[key] = 1; touch.active = true; btn.classList.add('pressed'); if (!$('help').hidden) startDriving(); });
    const up = () => { touch[key] = 0; btn.classList.remove('pressed'); };
    btn.addEventListener('pointerup', up); btn.addEventListener('pointercancel', up);
  }
}

function readInputs(dt) {
  const v = Math.max(sim.speed, 1);
  const pad = readPad();
  const kSteer = (keys.has('ArrowLeft') || keys.has('KeyA') ? 1 : 0) - (keys.has('ArrowRight') || keys.has('KeyD') ? 1 : 0);
  const kThr = keys.has('ArrowUp') || keys.has('KeyW') ? 1 : 0;
  const kBrk = keys.has('ArrowDown') || keys.has('KeyS') || keys.has('Space') ? 1 : 0;
  // Steering assist: full lock on the keyboard or the stick gives the steering angle that uses the
  // tyres' peak grip at this speed (Ackermann angle L/R for the tightest possible corner + a little
  // slip), instead of turning the front tyres far past their peak slip angle.
  const assist = drive.steerAssist !== false;
  const roadMax = sim.g.L * sim.maxLateral(v) / (v * v) + 0.3 * PEAK_SLIP; // rad
  const usefulLock = clamp((roadMax / DEG) * sim.g.steerRatio / drive.steerLock, 0, 1);
  let steerT, thr, brk, analog = false;
  if (pad && (Math.abs(pad.steer) > 0 || pad.throttle > 0.02 || pad.brake > 0.02)) {
    steerT = pad.steer * (assist ? Math.min(1, usefulLock * 1.15) : 1); thr = pad.throttle; brk = pad.brake; analog = true;
  } else if (touch.active && (touch.steer || touch.throttle || touch.brake)) {
    steerT = touch.steer * (assist ? Math.min(1, usefulLock * 1.1) : Math.min(1, 12 / Math.max(v, 12))); thr = touch.throttle; brk = touch.brake; analog = true;
  } else {
    steerT = kSteer * (assist ? usefulLock : Math.min(1, 7 / Math.max(v, 7))); thr = kThr; brk = kBrk;
  }
  const rate = analog ? 12 : (steerT === 0 ? 3.5 : 2.2);
  input.steer += clamp(steerT - input.steer, -rate * dt, rate * dt);
  const pedalRate = analog ? 20 : 6;
  input.throttle += clamp(thr - input.throttle, -pedalRate * dt, pedalRate * dt);
  input.brake += clamp(brk - input.brake, -pedalRate * 2 * dt, pedalRate * dt);
  // Reverse: once the car is stopped, release the brake and press it again (hold ~0.3 s) to engage
  // it; the brake pedal then drives backwards and the throttle pedal brakes. Braking to a stop and
  // keeping the pedal down never selects reverse. Press the throttle once stopped for first gear.
  if (!paused) {
    if (!reverse) {
      const stopped = sim.speed < 0.5;
      if (!stopped) reverseArmed = false;
      else if (brk < 0.1) reverseArmed = true;
      reverseHold = reverseArmed && brk > 0.5 && thr < 0.05 ? reverseHold + dt : 0;
      if (reverseHold > 0.3) { reverse = true; reverseHold = 0; reverseArmed = false; sim.setReverse(true); toast('Reverse gear: brake to go back, throttle to stop'); }
    } else if (thr > 0.1 && sim.vx > -0.6) { reverse = false; sim.setReverse(false); }
  }
  simIn.steer = input.steer;
  simIn.throttle = reverse ? input.brake * 0.6 : input.throttle;
  simIn.brake = reverse ? input.throttle : input.brake;
}
function shiftGear(dir) {
  if (reverse || paused) return;
  if (drive.autoGearbox) { drive.autoGearbox = false; applySettings(); toast('Manual gearbox (switch back in Setup)'); }
  sim.shift(dir);
}

// ---------------------------------------------------------------------------
// Lap timing and telemetry recording
// ---------------------------------------------------------------------------
const BIN = 10, NBINS = Math.ceil(TRACK.length / BIN);
const lap = { t: 0, valid: true, started: false, idx: 0, s: 0, last: null, best: null, sectors: [null, null, null], bestSectors: [null, null, null], sectorStart: 0, trace: new Float32Array(NBINS), bestTrace: null, offTrack: false, progress: 0 };
function fmtTime(t) {
  if (t == null || !Number.isFinite(t)) return '–';
  const m = Math.floor(t / 60), s = t - m * 60;
  return `${m}:${s.toFixed(3).padStart(6, '0')}`;
}
function wheelLaterals(loc) {
  return sim.pos.map(([px, py]) => {
    const wx = sim.x + px * Math.cos(sim.psi) - py * Math.sin(sim.psi), wy = sim.y + px * Math.sin(sim.psi) + py * Math.cos(sim.psi);
    return loc.lateral + (wx - sim.x) * loc.point.nx + (wy - sim.y) * loc.point.ny;
  });
}
let surfaces = [SURFACES.asphalt, SURFACES.asphalt, SURFACES.asphalt, SURFACES.asphalt];
function updateTrack(dt) {
  const loc = locate(sim.x, sim.y, lap.idx);
  lap.idx = loc.index;
  lap.lateral = loc.lateral;
  const hw = TRACK.width / 2;
  const lats = wheelLaterals(loc);
  // kerbs only exist through the corners
  const kerbs = Math.abs(loc.point.k) > KERB_K;
  surfaces = lats.map((l) => (Math.abs(l) <= hw ? SURFACES.asphalt : kerbs && Math.abs(l) <= hw + 1.4 ? SURFACES.kerb : SURFACES.grass));
  if (lats.every((l) => Math.abs(l) > hw) && lap.started) { lap.valid = false; }
  // far away: put the car back
  if (Math.abs(loc.lateral) > 60) resetCar();
  const prev = lap.s;
  lap.s = loc.s;
  const L = TRACK.length;
  // distance really covered forwards this lap (reversing over the line does not count)
  let ds = lap.s - prev;
  if (ds < -L / 2) ds += L; else if (ds > L / 2) ds -= L;
  lap.progress += ds;
  if (lap.started) {
    lap.t += dt;
    lap.trace[Math.min(NBINS - 1, Math.floor(lap.s / BIN))] = sim.speed * 3.6;
    // sectors
    for (let k = 1; k <= 2; k++) {
      const b = (L * k) / 3;
      if (prev < b && lap.s >= b && lap.s - prev < 50) closeSector(k - 1);
    }
  }
  if (prev > L - 60 && lap.s < 60 && (!lap.started || lap.progress > L * 0.9)) {
    if (lap.started) finishLap(); else { lap.started = true; }
    tlog.beacon();
    lap.progress = 0;
    lap.t = 0; lap.valid = true; lap.sectorStart = 0; lap.trace = new Float32Array(NBINS); lap.sectors = [null, null, null];
  }
}
function closeSector(i) {
  lap.sectors[i] = lap.t - lap.sectorStart;
  lap.sectorStart = lap.t;
}
function finishLap() {
  closeSector(2);
  const t = lap.t;
  lap.last = { t, valid: lap.valid };
  tlog.lap(t, lap.valid);
  if (lap.valid && (!lap.best || t < lap.best)) {
    const delta = lap.best ? t - lap.best : null;
    lap.best = t; lap.bestTrace = lap.trace; lap.bestSectors = [...lap.sectors];
    toast(delta == null ? `First lap: ${fmtTime(t)}` : `New best lap: ${fmtTime(t)} (${delta.toFixed(3)} s)`);
  } else if (!lap.valid) toast('Lap not counted: track limits');
  else toast(`Lap ${fmtTime(t)} (+${(t - lap.best).toFixed(3)} s)`);
}

// ---------------------------------------------------------------------------
// HUD and telemetry widgets
// ---------------------------------------------------------------------------
const ggTrail = [];
function drawTyres() {
  const cv = $('cvTyres'), ctx = cv.getContext('2d'), Wd = cv.width, H = cv.height;
  ctx.clearRect(0, 0, Wd, H);
  // car outline
  ctx.strokeStyle = 'rgba(255,255,255,0.18)'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(Wd / 2, 14); ctx.lineTo(Wd / 2 + 16, 60); ctx.lineTo(Wd / 2 + 16, 130); ctx.lineTo(Wd / 2 - 16, 130); ctx.lineTo(Wd / 2 - 16, 60); ctx.closePath(); ctx.stroke();
  const spots = { fl: [Wd / 2 - 48, 30], fr: [Wd / 2 + 48, 30], rl: [Wd / 2 - 48, 108], rr: [Wd / 2 + 48, 108] };
  ctx.font = '11px ui-monospace, monospace';
  sim.wheels.forEach((w) => {
    const [x, y] = spots[w.key], left = w.side > 0;
    // tyre rectangle, rotated by its steer angle
    ctx.save(); ctx.translate(x, y); ctx.rotate(-w.delta);
    ctx.fillStyle = `#${gripColor(w).getHexString()}`;
    ctx.fillRect(-7, -16, 14, 32);
    ctx.restore();
    // load bar
    const bx = left ? x - 22 : x + 16, frac = clamp(w.fz / 9000, 0, 1);
    ctx.fillStyle = 'rgba(255,255,255,0.12)'; ctx.fillRect(bx, y - 18, 6, 36);
    ctx.fillStyle = '#afa9ec'; ctx.fillRect(bx, y + 18 - 36 * frac, 6, 36 * frac);
    // values
    ctx.fillStyle = '#ecebe6'; ctx.textAlign = left ? 'right' : 'left';
    const tx = left ? x - 27 : x + 27;
    ctx.fillText(`${(w.fz / 1000).toFixed(1)} kN`, tx, y - 7);
    ctx.fillStyle = '#a3a29b';
    ctx.fillText(`γ ${w.camber.toFixed(1)}°`, tx, y + 6);
    ctx.fillText(`α ${(w.alpha / DEG).toFixed(1)}°`, tx, y + 19);
    if (w.slip) { ctx.fillStyle = '#ef9f27'; ctx.fillText(w.slip, tx, y + 32); }
    if (w.fz < 50) { ctx.fillStyle = '#f09595'; ctx.fillText('lifted', tx, y + 32); }
    else if (!w.slip && w.surface !== SURFACES.asphalt) { ctx.fillStyle = '#97c459'; ctx.fillText(w.surface.name, tx, y + 32); }
    else if (!w.slip && w.dirt > 0.05) { ctx.fillStyle = '#c9a26b'; ctx.fillText(`dirty ${Math.round(w.dirt * 100)}%`, tx, y + 32); }
  });
}
function drawGG() {
  const cv = $('cvGG'), ctx = cv.getContext('2d'), Wd = cv.width, H = cv.height, cx = Wd / 2, cy = H / 2, R = 26;
  ctx.clearRect(0, 0, Wd, H);
  ctx.strokeStyle = 'rgba(255,255,255,0.14)'; ctx.fillStyle = '#a3a29b'; ctx.font = '10px system-ui';
  for (let g = 1; g <= 3; g++) { ctx.beginPath(); ctx.arc(cx, cy, g * R, 0, Math.PI * 2); ctx.stroke(); ctx.fillText(`${g}g`, cx + g * R * 0.72 + 2, cy - g * R * 0.72); }
  ctx.beginPath(); ctx.moveTo(cx - 3 * R, cy); ctx.lineTo(cx + 3 * R, cy); ctx.moveTo(cx, cy - 3 * R); ctx.lineTo(cx, cy + 3 * R); ctx.stroke();
  ctx.fillText('brake', cx + 3, H - 3); ctx.fillText('accel', cx + 3, 10);
  ggTrail.forEach((p, i) => {
    ctx.fillStyle = `rgba(175,169,236,${(i / ggTrail.length) * 0.8})`;
    ctx.fillRect(cx - p[0] * R - 1.5, cy - p[1] * R - 1.5, 3, 3);
  });
  const p = ggTrail.at(-1);
  if (p) { ctx.fillStyle = '#ef9f27'; ctx.beginPath(); ctx.arc(cx - p[0] * R, cy - p[1] * R, 4, 0, Math.PI * 2); ctx.fill(); }
}
function drawTrace() {
  const cv = $('cvTrace'), ctx = cv.getContext('2d'), Wd = cv.width, H = cv.height;
  ctx.clearRect(0, 0, Wd, H);
  const vmax = 320, X = (i) => (i / (NBINS - 1)) * Wd, Y = (v) => H - 12 - (v / vmax) * (H - 18);
  ctx.strokeStyle = 'rgba(255,255,255,0.1)'; ctx.fillStyle = '#a3a29b'; ctx.font = '10px system-ui';
  for (const v of [100, 200, 300]) { ctx.beginPath(); ctx.moveTo(0, Y(v)); ctx.lineTo(Wd, Y(v)); ctx.stroke(); ctx.fillText(`${v}`, 2, Y(v) - 2); }
  for (const k of [1, 2]) { const x = (k / 3) * Wd; ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke(); }
  const line = (arr, color, upto = NBINS) => {
    ctx.strokeStyle = color; ctx.lineWidth = 1.6; ctx.beginPath();
    let pen = false;
    for (let i = 0; i < upto; i++) { const v = arr[i]; if (!v) { pen = false; continue; } pen ? ctx.lineTo(X(i), Y(v)) : ctx.moveTo(X(i), Y(v)); pen = true; }
    ctx.stroke();
  };
  if (lap.bestTrace) line(lap.bestTrace, '#ef9f27');
  line(lap.trace, '#afa9ec', Math.floor(lap.s / BIN) + 1);
  const x = X(lap.s / BIN);
  ctx.fillStyle = '#fff'; ctx.fillRect(x - 1, 0, 2, H);
  ctx.fillStyle = '#a3a29b'; ctx.fillText('distance →', Wd - 56, H - 2);
}
function updateHud() {
  $('speed').textContent = Math.round(sim.speed * 3.6);
  $('gear').textContent = sim.gear === -1 ? 'R' : sim.gear;
  $('gear').classList.toggle('rev', sim.gear === -1);
  const revs = clamp((sim.rpm - ENGINE.idle) / (ENGINE.redline - ENGINE.idle), 0, 1);
  $('revBar').style.width = `${revs * 100}%`;
  $('revBar').className = sim.rpm > ENGINE.upshift - 250 ? 'shift' : sim.rpm > 8500 ? 'high' : '';
  $('rpm').textContent = `${Math.round(sim.rpm / 100) * 100} rpm`;
  $('throttleBar').style.height = `${input.throttle * 100}%`;
  $('brakeBar').style.height = `${input.brake * 100}%`;
  $('steerBar').style.left = `${50 - input.steer * 50}%`;
  const flags = [...new Set(sim.wheels.map((w) => w.slip).filter(Boolean))];
  $('assist').textContent = flags.join(' · ');
  $('lapTime').textContent = lap.started ? fmtTime(lap.t) : 'out lap';
  $('lapFlag').textContent = lap.started && !lap.valid ? 'INVALID' : '';
  $('lapLast').textContent = lap.last ? fmtTime(lap.last.t) + (lap.last.valid ? '' : ' ✕') : '–';
  $('lapBest').textContent = fmtTime(lap.best);
  // live delta to the best lap at the same distance
  let delta = null;
  if (lap.best && lap.bestTrace && lap.started) delta = lap.t - lap.best * (lap.s / TRACK.length);
  const d = $('lapDelta');
  d.textContent = delta == null ? '–' : `${delta > 0 ? '+' : ''}${delta.toFixed(2)}`;
  d.className = delta == null ? '' : delta < 0 ? 'good' : 'bad';
  $('sectors').querySelectorAll('i').forEach((el, i) => {
    const t = lap.sectors[i], b = lap.bestSectors[i];
    el.className = t == null ? '' : b == null || t <= b ? 'best' : 'slow';
    el.title = t == null ? '' : `Sector ${i + 1}: ${t.toFixed(2)} s`;
  });
  if (!$('telemetry').hidden) {
    const bal = sim.balance || 0;
    $('balanceDot').style.left = `${50 + clamp(bal / 3, -1, 1) * 48}%`;
    $('balanceText').textContent = Math.abs(sim.ayF) < 2 ? 'go round a corner' : bal > 0.6 ? `understeer (${bal.toFixed(1)}°)` : bal < -0.6 ? `oversteer (${(-bal).toFixed(1)}°)` : 'neutral';
    const stats = [
      ['Lateral', `${(sim.ayF / 9.81).toFixed(2)} g`], ['Longit.', `${(sim.axF / 9.81).toFixed(2)} g`],
      ['Roll', `${(sim.roll / DEG).toFixed(2)}°`], ['Pitch', `${(sim.dive / DEG).toFixed(2)}°`],
      ['Downforce', `${(sim.downforce / 1000).toFixed(1)} kN`], ['Yaw rate', `${(sim.r / DEG).toFixed(0)}°/s`],
      ['Wheel angle', `${(((sim.wheels[0].delta + sim.wheels[1].delta) / 2) / DEG).toFixed(1)}°`], ['Front travel', `${(((sim.wheels[0].travel + sim.wheels[1].travel) / 2) * 1000).toFixed(1)} mm`],
      ['Engine', `${Math.round(sim.rpm / 10) * 10} rpm`], ['Drive force', `${(sim.driveForce / 1000).toFixed(1)} kN`],
    ];
    $('stats').innerHTML = stats.map(([k, v]) => `<div>${k}<b>${v}</b></div>`).join('');
    drawTyres(); drawGG(); drawTrace();
  }
}

// ---------------------------------------------------------------------------
// Setup drawer
// ---------------------------------------------------------------------------
const HELP = {
  springFront: 'Front spring rate at the damper. The wheel rate is this times the motion ratio squared (set by the rocker in the lab). Stiffer = less pitch and roll, less mechanical grip at that axle.',
  springRear: 'Rear spring rate at the damper. With a low rear motion ratio (pullrod) the rear needs a stiffer spring for the same wheel rate.',
  arbFront: 'Front anti-roll bar. Stiffer = more of the roll load transfer goes to the front: less front grip, more understeer.',
  arbRear: 'Rear anti-roll bar. Stiffer = more load transfer at the rear: the rear lets go first, more oversteer.',
  downforce: 'Downforce coefficient (ClA). More downforce = more grip at high speed, no change at low speed.',
  aeroBalance: 'Share of the downforce on the front axle. Forward = sharper in fast corners; rearward = more stable.',
  brakeBias: 'Share of the braking force on the front wheels. Too far forward: fronts lock (no steering); too far back: rears lock (spin).',
  weightFront: 'Static weight on the front axle (ballast position).',
  mass: 'Mass of the car with the driver.',
  power: 'Engine power. More power = more wheelspin out of slow corners.',
  grip: 'Peak friction coefficient of the tyres (slicks ≈ 1.6–1.8).',
};
const SLIDERS = {
  slSprings: [
    { key: 'springFront', label: 'Front spring', unit: 'N/mm', min: 60, max: 700, step: 10, d: 0 },
    { key: 'springRear', label: 'Rear spring', unit: 'N/mm', min: 60, max: 1200, step: 10, d: 0 },
    { key: 'arbFront', label: 'Front anti-roll bar', unit: 'N·m/°', min: 0, max: 3000, step: 50, d: 0 },
    { key: 'arbRear', label: 'Rear anti-roll bar', unit: 'N·m/°', min: 0, max: 3000, step: 50, d: 0 },
  ],
  slAero: [
    { key: 'downforce', label: 'Downforce (ClA)', unit: 'm²', min: 0, max: 5, step: 0.1, d: 1 },
    { key: 'aeroBalance', label: 'Aero balance (front)', unit: '%', min: 0.3, max: 0.55, step: 0.01, pct: true },
    { key: 'brakeBias', label: 'Brake balance (front)', unit: '%', min: 0.4, max: 0.8, step: 0.01, pct: true, lab: true },
  ],
  slMass: [
    { key: 'mass', label: 'Mass', unit: 'kg', min: 600, max: 950, step: 5, d: 0 },
    { key: 'weightFront', label: 'Weight on front', unit: '%', min: 0.36, max: 0.54, step: 0.01, pct: true },
    { key: 'power', label: 'Power', unit: 'kW', min: 200, max: 700, step: 10, d: 0 },
    { key: 'grip', label: 'Tyre grip (µ)', unit: '', min: 1.0, max: 2.1, step: 0.05, d: 2 },
  ],
};
const sliderViews = [];
for (const [box, list] of Object.entries(SLIDERS)) for (const cfg of list) {
  const row = document.createElement('div');
  row.className = 'slider';
  row.innerHTML = `<label title="${HELP[cfg.key]}">${cfg.label}</label><output></output><input type="range" min="${cfg.min}" max="${cfg.max}" step="${cfg.step}">`;
  $(box).appendChild(row);
  const inp = row.querySelector('input'), out = row.querySelector('output');
  const get = () => (cfg.lab ? setup.vehicle.brakeBiasFront : drive[cfg.key]);
  const show = () => { const v = get(); inp.value = v; out.textContent = cfg.pct ? `${Math.round(v * 100)} %` : `${fmt(v, cfg.d)} ${cfg.unit}`; };
  inp.addEventListener('input', () => {
    const v = parseFloat(inp.value);
    if (cfg.lab) setup.vehicle.brakeBiasFront = v; else drive[cfg.key] = v;
    applySettings(); show();
    $('explain').textContent = HELP[cfg.key];
  });
  row.querySelector('label').addEventListener('click', () => { $('explain').textContent = HELP[cfg.key]; });
  show();
  sliderViews.push(show);
}
function applySettings() {
  sim.configure(setup, drive);
  placeIsoCar();
  saveAll();
  $('btnTC').classList.toggle('on', !!drive.tc);
  $('btnABS').classList.toggle('on', !!drive.abs);
  $('btnGearbox').classList.toggle('on', !!drive.autoGearbox);
  $('btnSteerAssist').classList.toggle('on', drive.steerAssist !== false);
}
$('btnGearbox').addEventListener('click', () => { drive.autoGearbox = !drive.autoGearbox; applySettings(); toast(drive.autoGearbox ? 'Automatic gearbox' : 'Manual gearbox: E / Q or RB / LB'); });
$('btnSteerAssist').addEventListener('click', () => { drive.steerAssist = !drive.steerAssist; applySettings(); toast(`Steering assist ${drive.steerAssist ? 'on' : 'off'}`); });
$('btnTC').addEventListener('click', () => { drive.tc = !drive.tc; applySettings(); toast(`Traction control ${drive.tc ? 'on' : 'off'}`); });
$('btnABS').addEventListener('click', () => { drive.abs = !drive.abs; applySettings(); toast(`ABS ${drive.abs ? 'on' : 'off'}`); });
$('btnDriveReset').addEventListener('click', () => { drive = { ...DEFAULT_DRIVE }; setup.vehicle.brakeBiasFront = S.DEFAULT_SETUP.vehicle.brakeBiasFront; applySettings(); sliderViews.forEach((f) => f()); });

function geometrySummary() {
  const st = sim.g, f = setup.front, r = setup.rear;
  const rows = [
    ['Spring actuation', f.rod === 'push' ? 'pushrod' : 'pullrod', r.rod === 'push' ? 'pushrod' : 'pullrod'],
    ['Camber', `${f.alignment.camber.toFixed(2)}°`, `${r.alignment.camber.toFixed(2)}°`],
    ['Toe (+ in)', `${f.alignment.toe.toFixed(2)}°`, `${r.alignment.toe.toFixed(2)}°`],
    ['Caster / KPI', `${f.alignment.caster.toFixed(1)}° / ${f.alignment.kpi.toFixed(1)}°`, `${r.alignment.caster.toFixed(1)}° / ${r.alignment.kpi.toFixed(1)}°`],
    ['Roll centre*', `${(st.rcF * 1000).toFixed(0)} mm`, `${(st.rcR * 1000).toFixed(0)} mm`],
    ['Motion ratio', fmt(st.mrF, 2), fmt(st.mrR, 2)],
    ['Wheel rate*', `${(sim.kwF / 1000).toFixed(0)} N/mm`, `${(sim.kwR / 1000).toFixed(0)} N/mm`],
    ['Anti-dive / squat', `${(st.antiDive * 100).toFixed(0)} %`, `${(st.antiSquat * 100).toFixed(0)} %`],
  ];
  $('geoSummary').innerHTML = `<span></span><span class="h">Front</span><span class="h">Rear</span>${rows.map(([k, a, b]) => `<span>${k}</span><b>${a}</b><b>${b}</b>`).join('')}<span style="grid-column:1/4;font-size:11px">* full-size values (lab ×4)</span>`;
}

// ---------------------------------------------------------------------------
// UI actions
// ---------------------------------------------------------------------------
let paused = true;
function toast(msg) {
  const el = $('toast');
  el.textContent = msg; el.classList.add('show');
  clearTimeout(toast.t); toast.t = setTimeout(() => el.classList.remove('show'), 2200);
}
function resetCar() {
  const loc = locate(sim.x, sim.y, lap.idx);
  const p = loc.point;
  sim.reset({ x: p.x, y: p.y, psi: Math.atan2(p.ty, p.tx) });
  input.steer = input.throttle = input.brake = 0;
  reverse = false;
  if (lap.started) lap.valid = false;
  updateCamera(0, true);
}
function cycleCam() { camMode = (camMode + 1) % CAMS.length; $('btnCam').textContent = `Camera: ${CAMS[camMode]}`; updateCamera(0, true); }
function toggleTelemetry() { const t = $('telemetry'); t.hidden = !t.hidden; $('btnTelemetry').classList.toggle('on', !t.hidden); }
function toggleSetup() {
  const s = $('setupPanel'); s.hidden = !s.hidden; $('btnSetup').classList.toggle('on', !s.hidden);
  if (!s.hidden) { geometrySummary(); if (small()) $('telemetry').hidden = true; }
}
function showHelp() { $('help').hidden = false; paused = true; updatePadStatus(); }
function startDriving() { $('help').hidden = true; paused = false; last = performance.now(); wakeAudio(); }
function togglePause() {
  if (!$('help').hidden) { startDriving(); return; }
  paused = !paused; toast(paused ? 'Paused' : 'Go');
}
function updateSoundBtn() {
  const b = $('btnSound');
  b.textContent = audio.enabled ? 'Sound: on' : 'Sound: off';
  b.classList.toggle('on', audio.enabled);
}
function toggleSound() {
  audio.setEnabled(!audio.enabled);
  try { localStorage.setItem('as3d-drive-sound', audio.enabled ? 'on' : 'off'); } catch (e) { /* ignore */ }
  updateSoundBtn();
  if (audio.enabled && !audio.running) toast('Click or press a key to start the sound');
}
$('btnSound').addEventListener('click', toggleSound);
document.addEventListener('visibilitychange', () => { if (document.hidden) audio.suspend(); else wakeAudio(); });
$('btnCam').addEventListener('click', cycleCam);
$('btnTelemetry').addEventListener('click', toggleTelemetry);
$('btnSetup').addEventListener('click', toggleSetup);
$('btnSetupClose').addEventListener('click', toggleSetup);
$('btnReset').addEventListener('click', resetCar);
$('btnHelp').addEventListener('click', showHelp);
$('btnStart').addEventListener('click', startDriving);
if (small()) { $('telemetry').hidden = true; $('btnTelemetry').classList.remove('on'); }

// ---------------------------------------------------------------------------
// Data logging for MoTeC i2 (50 Hz). Channel names follow MoTeC conventions so the usual i2
// worksheets and maths pick them up. Positions are turned into GPS coordinates around an arbitrary
// reference point so i2 can draw the track map.
// ---------------------------------------------------------------------------
const LOG_RATE = 50, LAT0 = 46.2, LON0 = 6.1, R_EARTH = 6371000;
const W4 = [['FL', 0], ['FR', 1], ['RL', 2], ['RR', 3]];
const surfaceCode = (w) => (w.surface === SURFACES.grass ? 2 : w.surface === SURFACES.kerb ? 1 : 0);
const LOG_CHANNELS = [
  { name: 'Ground Speed', short: 'Speed', unit: 'km/h', get: () => sim.speed * 3.6 },
  { name: 'Engine RPM', short: 'RPM', unit: 'rpm', get: () => sim.rpm },
  { name: 'Gear', short: 'Gear', unit: '', get: () => sim.gear },
  { name: 'Throttle Pos', short: 'Thr', unit: '%', get: () => input.throttle * 100 },
  { name: 'Brake Pos', short: 'Brk', unit: '%', get: () => input.brake * 100 },
  { name: 'Steering Angle', short: 'Steer', unit: 'deg', get: () => sim.steerDeg },
  { name: 'Steered Angle', short: 'SteerW', unit: 'deg', get: () => ((sim.wheels[0].delta + sim.wheels[1].delta) / 2) / DEG },
  { name: 'G Force Lat', short: 'GLat', unit: 'G', get: () => sim.ayF / 9.81 },
  { name: 'G Force Long', short: 'GLong', unit: 'G', get: () => sim.axF / 9.81 },
  { name: 'Yaw Rate', short: 'Yaw', unit: 'deg/s', get: () => sim.r / DEG },
  { name: 'Body Roll', short: 'Roll', unit: 'deg', get: () => sim.roll / DEG },
  { name: 'Body Pitch', short: 'Pitch', unit: 'deg', get: () => sim.dive / DEG },
  { name: 'Understeer Angle', short: 'US', unit: 'deg', get: () => sim.balance || 0 },
  { name: 'Downforce', short: 'DF', unit: 'N', get: () => sim.downforce },
  { name: 'Drive Force', short: 'Fdrive', unit: 'N', get: () => sim.driveForce },
  { name: 'TC Active', short: 'TC', unit: '', get: () => (sim.wheels.some((w) => w.slip === 'TC') ? 1 : 0) },
  { name: 'ABS Active', short: 'ABS', unit: '', get: () => (sim.wheels.some((w) => w.slip === 'ABS') ? 1 : 0) },
  { name: 'Lap Distance', short: 'LapDist', unit: 'm', get: () => lap.s },
  { name: 'Track Offset', short: 'Offset', unit: 'm', get: () => lap.lateral || 0 },
  { name: 'GPS Latitude', short: 'Lat', unit: 'deg', get: () => LAT0 + (sim.y / R_EARTH) / DEG },
  { name: 'GPS Longitude', short: 'Long', unit: 'deg', get: () => LON0 + (sim.x / (R_EARTH * Math.cos(LAT0 * DEG))) / DEG },
  ...W4.flatMap(([c, i]) => [
    { name: `Tyre Load ${c}`, short: `Load${c}`, unit: 'N', get: () => sim.wheels[i].fz },
    { name: `Tyre Slip Angle ${c}`, short: `Slip${c}`, unit: 'deg', get: () => sim.wheels[i].alpha / DEG },
    { name: `Camber ${c}`, short: `Cam${c}`, unit: 'deg', get: () => sim.wheels[i].camber },
    { name: `Tyre Grip Used ${c}`, short: `Grip${c}`, unit: '%', get: () => sim.wheels[i].util * 100 },
    { name: `Wheel Travel ${c}`, short: `Trav${c}`, unit: 'mm', get: () => sim.wheels[i].travel * 1000 },
    { name: `Surface ${c}`, short: `Surf${c}`, unit: '', get: () => surfaceCode(sim.wheels[i]) },
  ]),
];
const tlog = new TelemetryLog(LOG_CHANNELS, LOG_RATE);
let logAcc = 0;
function logStep(dt) {
  logAcc += dt;
  while (logAcc >= 1 / LOG_RATE) {
    logAcc -= 1 / LOG_RATE;
    if (!tlog.record()) { if (!tlog.warned) { tlog.warned = true; toast('Log full (60 min): export it, then reload to start a new one'); } break; }
  }
}
function exportMotec() {
  if (tlog.n < LOG_RATE) { toast('Drive a little first: nothing to export yet'); return; }
  const d = tlog.started, p2 = (x) => String(x).padStart(2, '0');
  const base = `TrackTest_${d.getFullYear()}-${p2(d.getMonth() + 1)}-${p2(d.getDate())}_${p2(d.getHours())}-${p2(d.getMinutes())}-${p2(d.getSeconds())}`;
  const f = setup.front, r = setup.rear;
  const comment = `Setup: front ${f.rod}rod, rear ${r.rod}rod, camber ${f.alignment.camber.toFixed(2)} / ${r.alignment.camber.toFixed(2)} deg, `
    + `springs ${drive.springFront} / ${drive.springRear} N/mm, ARB ${drive.arbFront} / ${drive.arbRear} Nm/deg, ClA ${drive.downforce}, `
    + `aero balance ${Math.round(drive.aeroBalance * 100)} %, brake balance ${Math.round(setup.vehicle.brakeBiasFront * 100)} %, mass ${drive.mass} kg, power ${drive.power} kW, grip ${drive.grip}`;
  download(`${base}.ld`, tlog.buildLd({
    driver: '', vehicle: 'Active Suspension 3D', venue: 'Test circuit',
    event: 'Track test', session: 'Practice', comment, shortComment: 'Track test',
    weight: drive.mass, vehicleType: 'Single seater', vehicleComment: 'Lab setup x4',
  }), 'application/octet-stream');
  setTimeout(() => download(`${base}.ldx`, tlog.buildLdx(), 'application/xml'), 600);
  const m = Math.floor(tlog.duration / 60), sec = Math.round(tlog.duration % 60);
  toast(`MoTeC log exported: ${m} min ${sec} s, ${tlog.beacons.length + 1} laps. Keep the .ld and .ldx together.`);
}
$('btnExport').addEventListener('click', exportMotec);

// ---------------------------------------------------------------------------
// Main loop: fixed 240 Hz physics, rendering at the display rate
// ---------------------------------------------------------------------------
const PHYS_DT = 1 / 240;
let last = performance.now(), acc = 0, hudTimer = 0, frameDt = 0;
function frame(now) {
  requestAnimationFrame(frame);
  frameDt = Math.min((now - last) / 1000, 0.1);
  last = now;
  readInputs(frameDt);
  if (!paused) {
    acc += frameDt;
    let steps = 0;
    while (acc >= PHYS_DT && steps < 40) {
      sim.step(PHYS_DT, simIn, (i) => surfaces[i]);
      acc -= PHYS_DT; steps++;
    }
    updateTrack(frameDt);
    logStep(frameDt);
    ggTrail.push([sim.ayF / 9.81, sim.axF / 9.81]);
    if (ggTrail.length > 180) ggTrail.shift();
  } else frameDt = 0;
  updateCar();
  audio.update(sim, simIn, { paused });
  updateCamera(Math.max(frameDt, 1 / 60));
  sun.position.copy(carRoot.position).add(new THREE.Vector3(30, 60, 20));
  sun.target.position.copy(carRoot.position);
  renderer.render(scene, camera);
  hudTimer += frameDt || 1 / 60;
  if (hudTimer > 1 / 20) { hudTimer = 0; updateHud(); }
}

function resize() {
  const w = innerWidth, h = innerHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.fov = w < h ? 75 : 62;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);
resize();
applySettings();
geometrySummary();
updatePadStatus();
updateCar();
updateCamera(0, true);
requestAnimationFrame(frame);
