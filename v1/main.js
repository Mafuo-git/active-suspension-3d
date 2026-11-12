// Active Suspension 3D – front axle of a 1/4-scale F2-style car.
//
// Each rocker can be locked to a servo-driven drive disc by a ball lock pin (dog-clutch principle):
//   - Active mode: pin through rocker hub + drive disc, the servo turns the rocker and drives the whole corner.
//   - Manual mode: pin parked in a holster on the chassis (micro-switch cuts servo power), the rocker turns
//     freely on its bearing and the corner is moved by hand.
// The suspension kinematics are the same in both modes; only the input changes (servo or hand).
//
// Scene frame: x = lateral (right positive), y = up, z = longitudinal (front positive). 1 unit = 10 mm.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { HP, UNIT_MM, TRAVEL_LIMITS_MM, solveCorner, motionRatio, rockerPoint } from './kinematics.js';

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
const TRACK = HP.wheelCentre.x * 2;
const PIN_Z = { engaged: 0.25, lifted: 1.9 };      // pin centre along z (shaft spans ±1.1)
const DRIVE_DISC_Z = -0.55;
const SERVO_Z = -2.1;
const WISHBONE_LEG_Z = { lower: 3.5, upper: 3.0 }; // front / rear leg spacing at the chassis
const INPUT_RANGE_MM = { heave: [-10, 12], roll: [-10, 10] };
const SEQUENCE_SPEED = 0.9;                         // pin move cycles per second

// ---------------------------------------------------------------------------
// Renderer, scene, camera, lights
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

const camera = new THREE.PerspectiveCamera(36, 1, 0.5, 600);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 6;
controls.maxDistance = 160;
controls.maxPolarAngle = Math.PI * 0.495; // stay above the ground

const sun = new THREE.DirectionalLight(0xffffff, 1.5);
sun.position.set(18, 40, 26);
sun.castShadow = true;
sun.shadow.mapSize.set(1024, 1024);
Object.assign(sun.shadow.camera, { left: -32, right: 32, top: 32, bottom: -32, near: 5, far: 100 });
sun.shadow.bias = -0.0006;
scene.add(sun, new THREE.HemisphereLight(0xffffff, 0x3a3a40, 0.35));

// Ground: shadow catcher and a faint grid
const ground = new THREE.Mesh(new THREE.PlaneGeometry(400, 400), new THREE.ShadowMaterial({ opacity: 0.28 }));
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);
const grid = new THREE.GridHelper(160, 32, 0x55555f, 0x3a3a42);
grid.material.transparent = true;
grid.material.opacity = 0.35;
grid.position.y = 0.01;
scene.add(grid);

// ---------------------------------------------------------------------------
// Materials
// ---------------------------------------------------------------------------
const mat = (color, opts = {}) => new THREE.MeshStandardMaterial({ color, metalness: 0.2, roughness: 0.5, ...opts });
const M = {
  carbon: mat(0x26262b, { roughness: 0.55, metalness: 0.15 }),
  nose: mat(0x2f2f36, { roughness: 0.45 }),
  alu: mat(0xbdbcb5, { metalness: 0.85, roughness: 0.3 }),
  steel: mat(0xdcdcd6, { metalness: 0.95, roughness: 0.2 }),
  wishbone: mat(0x3d3d44, { metalness: 0.6, roughness: 0.35 }),
  upright: mat(0x8e8d87, { metalness: 0.8, roughness: 0.35 }),
  servo: mat(0x534ab7, { roughness: 0.45 }),
  disc: mat(0x7f77dd, { metalness: 0.6, roughness: 0.3 }),
  rocker: mat(0xc9c8c1, { metalness: 0.85, roughness: 0.28 }),
  coilover: mat(0x1d9e75, { metalness: 0.5, roughness: 0.3 }),
  spring: mat(0x9fe1cb, { metalness: 0.4, roughness: 0.35 }),
  pin: mat(0xef9f27, { metalness: 0.75, roughness: 0.25 }),
  holster: mat(0x5f5e5a, { metalness: 0.6, roughness: 0.4 }),
  bore: mat(0x0c0c0e, { roughness: 1 }),
  tyre: mat(0x18181a, { roughness: 0.92 }),
  stripe: mat(0xef9f27, { roughness: 0.6 }),
  rim: mat(0x9a9a96, { metalness: 0.9, roughness: 0.25, side: THREE.DoubleSide }),
  ledOn: mat(0x639922, { emissive: 0x3b6d11, emissiveIntensity: 1.2 }),
  ledOff: mat(0xe24b4a, { emissive: 0xa32d2d, emissiveIntensity: 1.2 }),
};
const xrayMaterials = [M.carbon, M.nose, M.servo];

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------
function mesh(geometry, material, parent, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(geometry, material);
  m.position.set(x, y, z);
  m.castShadow = true;
  m.receiveShadow = true;
  parent.add(m);
  return m;
}

// Extruded plate (outline in the x-y plane) centred on its thickness along z
function plate(shape, depth, holes = [], bevel = 0.03) {
  for (const [hx, hy, r] of holes) {
    const p = new THREE.Path();
    p.absarc(hx, hy, r, 0, Math.PI * 2, true);
    shape.holes.push(p);
  }
  const g = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: bevel > 0, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 2, curveSegments: 32 });
  g.translate(0, 0, -depth / 2);
  return g;
}

// Convex outline wrapping circles [x, y, r] (rocker plates)
function hullAroundCircles(circles, samples = 28) {
  const pts = [];
  for (const [x, y, r] of circles) for (let i = 0; i < samples; i++) {
    const a = (i / samples) * Math.PI * 2;
    pts.push([x + r * Math.cos(a), y + r * Math.sin(a)]);
  }
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [], upper = [];
  for (const p of pts) { while (lower.length > 1 && cross(lower.at(-2), lower.at(-1), p) <= 0) lower.pop(); lower.push(p); }
  for (const p of pts.reverse()) { while (upper.length > 1 && cross(upper.at(-2), upper.at(-1), p) <= 0) upper.pop(); upper.push(p); }
  const hull = lower.slice(0, -1).concat(upper.slice(0, -1));
  const s = new THREE.Shape();
  hull.forEach(([x, y], i) => (i ? s.lineTo(x, y) : s.moveTo(x, y)));
  s.closePath();
  return s;
}

const cylZ = (r, len, seg = 24) => new THREE.CylinderGeometry(r, r, len, seg).rotateX(Math.PI / 2);
const cylX = (r, len, seg = 24, open = false) => new THREE.CylinderGeometry(r, r, len, seg, 1, open).rotateZ(Math.PI / 2);

// Unit-height Y-aligned mesh stretched between two points (rods, links)
function rod(radius, material, parent) {
  const m = mesh(new THREE.CylinderGeometry(radius, radius, 1, 14), material, parent);
  m.update = (a, b) => {
    const dir = new THREE.Vector3().subVectors(b, a);
    m.position.copy(a).addScaledVector(dir, 0.5);
    m.scale.set(1, dir.length(), 1);
    m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
  };
  return m;
}

// Box whose back (z0) and front (z1) cross-sections differ: tapered nose
function taperedBox(back, front, z0, z1) {
  const g = new THREE.BoxGeometry(1, 1, 1, 1, 1, 1);
  const pos = g.attributes.position;
  for (let i = 0; i < pos.count; i++) {
    const sx = Math.sign(pos.getX(i)), sy = Math.sign(pos.getY(i)), sz = Math.sign(pos.getZ(i));
    const s = sz > 0 ? front : back;
    pos.setXYZ(i, sx * s.halfWidth, sy > 0 ? s.top : s.bottom, sz > 0 ? z1 : z0);
  }
  g.computeVertexNormals();
  return g;
}

// Helical spring rebuilt to its actual length so the wire keeps a round section
function springGeometry(length, turns = 6, radius = 0.55, wire = 0.07) {
  const pts = [];
  for (let i = 0; i <= 240; i++) {
    const t = i / 240;
    pts.push(new THREE.Vector3(radius * Math.cos(t * Math.PI * 2 * turns), t * length, radius * Math.sin(t * Math.PI * 2 * turns)));
  }
  return new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 300, wire, 8, false);
}

const v3 = (p, z = 0) => new THREE.Vector3(p.x, p.y, z);

// ---------------------------------------------------------------------------
// Car body (moves with heave and roll; wheels stay on the ground)
// ---------------------------------------------------------------------------
const car = new THREE.Group();
scene.add(car);

// Monocoque section around the front suspension
{
  const s = new THREE.Shape();
  s.moveTo(-6.2, 2.0); s.lineTo(6.2, 2.0); s.lineTo(7.3, 15.3);
  s.quadraticCurveTo(7.4, 16.5, 6.2, 16.5); s.lineTo(-6.2, 16.5);
  s.quadraticCurveTo(-7.4, 16.5, -7.3, 15.3); s.closePath();
  const g = new THREE.ExtrudeGeometry(s, { depth: 17, bevelEnabled: true, bevelThickness: 0.3, bevelSize: 0.3, bevelSegments: 3, curveSegments: 16 });
  g.translate(0, 0, -11);
  mesh(g, M.carbon, car);
}
// Nose cone
mesh(taperedBox({ halfWidth: 6.0, bottom: 3.0, top: 14.5 }, { halfWidth: 2.2, bottom: 3.2, top: 6.2 }, 6.3, 22), M.nose, car);
// Central coilover mount
mesh(new THREE.BoxGeometry(0.9, HP.coiloverChassis.y - 16.5, 0.9), M.alu, car, 0, (HP.coiloverChassis.y + 16.5) / 2, 0);
mesh(new THREE.BoxGeometry(2.4, 1.2, 1.2), M.alu, car, 0, HP.coiloverChassis.y, 0);

// ---------------------------------------------------------------------------
// Front corner (built for the right side, mirrored for the left)
// ---------------------------------------------------------------------------
function buildCorner(side) {
  const g = new THREE.Group();
  g.scale.x = side;
  car.add(g);
  const c = { group: g, side };

  // Wishbones: A-arms with front and rear legs meeting at the ball joints
  c.legs = [];
  for (const [inboard, zLeg] of [[HP.lowerInboard, WISHBONE_LEG_Z.lower], [HP.upperInboard, WISHBONE_LEG_Z.upper]]) {
    for (const z of [zLeg, -zLeg]) {
      mesh(new THREE.BoxGeometry(0.6, 0.8, 0.6), M.alu, g, inboard.x - 0.2, inboard.y, z);
      c.legs.push({ rod: rod(0.17, M.wishbone, g), from: v3(inboard, z) });
    }
  }
  c.lowerBall = mesh(new THREE.SphereGeometry(0.33, 16, 16), M.steel, g);
  c.upperBall = mesh(new THREE.SphereGeometry(0.33, 16, 16), M.steel, g);

  // Upright (rotates about the lower ball joint)
  c.upright = new THREE.Group();
  g.add(c.upright);
  {
    const lbj = HP.lowerBallJoint, rel = (p) => new THREE.Vector3(p.x - lbj.x, p.y - lbj.y, 0);
    const kingpin = rod(0.45, M.upright, c.upright); kingpin.update(rel(lbj), rel(HP.upperBallJoint));
    const hubArm = rod(0.55, M.upright, c.upright); hubArm.update(rel({ x: lbj.x - 0.4, y: HP.wheelCentre.y }), rel(HP.wheelCentre));
    const pickup = rod(0.22, M.upright, c.upright); pickup.update(rel(lbj).setY(1.0), rel(HP.pushrodLower));
    mesh(cylX(0.5, 3.4, 24), M.steel, c.upright, HP.wheelCentre.x - lbj.x - 0.8, HP.wheelCentre.y - lbj.y, 0);
  }

  // Wheel: tyre (lathe profile), rim barrel, spokes
  c.wheel = new THREE.Group();
  g.add(c.wheel);
  {
    const profile = [];
    const ri = 6.1, ro = 9.0, hw = 3.0, rc = 1.1;
    profile.push(new THREE.Vector2(ri, -hw));
    for (let i = 0; i <= 8; i++) { const a = -Math.PI / 2 + (i / 8) * (Math.PI / 2); profile.push(new THREE.Vector2(ro - rc + rc * Math.cos(a), -hw + rc + rc * Math.sin(a))); }
    for (let i = 0; i <= 8; i++) { const a = (i / 8) * (Math.PI / 2); profile.push(new THREE.Vector2(ro - rc + rc * Math.cos(a), hw - rc + rc * Math.sin(a))); }
    profile.push(new THREE.Vector2(ri, hw));
    mesh(new THREE.LatheGeometry(profile, 64).rotateZ(Math.PI / 2), M.tyre, c.wheel);
    const stripe = mesh(new THREE.TorusGeometry(7.4, 0.08, 8, 64).rotateY(Math.PI / 2), M.stripe, c.wheel, 3.02, 0, 0);
    stripe.castShadow = false;
    mesh(cylX(6.1, 5.6, 48, true), M.rim, c.wheel);
    for (let i = 0; i < 6; i++) {
      const spoke = new THREE.Group();
      spoke.rotation.x = (i / 6) * Math.PI * 2;
      c.wheel.add(spoke);
      mesh(new THREE.BoxGeometry(0.35, 5.2, 0.7), M.rim, spoke, 2.4, 3.0, 0);
    }
    mesh(cylX(1.1, 0.9, 32), M.rim, c.wheel, 2.4, 0, 0);
  }

  // Pushrod
  c.pushrod = rod(0.16, M.steel, g);
  c.pushrodEnds = [mesh(new THREE.SphereGeometry(0.26, 14, 14), M.steel, g), mesh(new THREE.SphereGeometry(0.26, 14, 14), M.steel, g)];

  // Servo on its pedestal, coaxial with the rocker pivot, and its axle
  const rp = HP.rockerPivot;
  mesh(new THREE.BoxGeometry(2.6, rp.y - 1.25 - 16.5, 2.6), M.alu, g, rp.x, (rp.y - 1.25 + 16.5) / 2, SERVO_Z);
  mesh(new THREE.BoxGeometry(2.3, 2.5, 2.2), M.servo, g, rp.x, rp.y, SERVO_Z);
  mesh(new THREE.BoxGeometry(3.4, 0.25, 2.2), M.servo, g, rp.x, rp.y - 0.2, SERVO_Z);
  mesh(cylZ(0.26, 1.9, 20), M.steel, g, rp.x, rp.y, -0.55);
  mesh(cylZ(0.34, 0.25, 6), M.steel, g, rp.x, rp.y, 0.3); // axle nut

  // Drive disc on the servo output (rotates with the servo)
  c.disc = new THREE.Group();
  c.disc.position.set(rp.x, rp.y, DRIVE_DISC_Z);
  g.add(c.disc);
  {
    const s = new THREE.Shape(); s.absarc(0, 0, 1.55, 0, Math.PI * 2, false);
    mesh(plate(s, 0.22, [[HP.pinHole.x, HP.pinHole.y, 0.28]], 0.02), M.disc, c.disc);
    mesh(new THREE.BoxGeometry(0.18, 0.5, 0.1), M.steel, c.disc, 0, 1.25, 0.13); // index mark
  }

  // Rocker (free on the axle; locked to the drive disc only when the pin is engaged)
  c.rocker = new THREE.Group();
  c.rocker.position.set(rp.x, rp.y, 0);
  g.add(c.rocker);
  {
    const a = HP.rockerPushrod, b = HP.rockerCoilover, h = HP.pinHole;
    const outline = hullAroundCircles([[0, 0, 1.55], [a.x, a.y, 0.5], [b.x, b.y, 0.5]]);
    mesh(plate(outline, 0.3, [[0, 0, 0.27], [a.x, a.y, 0.16], [b.x, b.y, 0.16], [h.x, h.y, 0.28]]), M.rocker, c.rocker);
    mesh(new THREE.TorusGeometry(0.42, 0.06, 8, 24), M.pin, c.rocker, h.x, h.y, 0.17); // pin hole chamfer ring
  }

  // Coilover (chassis mount -> rocker), damper body near the chassis
  c.coilover = new THREE.Group();
  g.add(c.coilover);
  {
    const eye = new THREE.TorusGeometry(0.34, 0.12, 10, 24);
    c.eyeA = mesh(eye, M.coilover, c.coilover);
    c.eyeB = mesh(eye, M.coilover, c.coilover);
    c.body = mesh(new THREE.CylinderGeometry(0.36, 0.36, 2.0, 28), M.coilover, c.coilover, 0, 1.5, 0);
    c.shaft = mesh(new THREE.CylinderGeometry(0.12, 0.12, 1, 14), M.steel, c.coilover);
    c.perchA = mesh(new THREE.CylinderGeometry(0.68, 0.68, 0.12, 28), M.coilover, c.coilover, 0, 1.0, 0);
    c.perchB = mesh(new THREE.CylinderGeometry(0.68, 0.68, 0.12, 28), M.coilover, c.coilover);
    c.springLength = 3;
    c.spring = mesh(springGeometry(c.springLength), M.spring, c.coilover, 0, 1.06, 0);
  }

  // Pin holster on the chassis, with its micro-switch and status LED
  const hs = HP.holster;
  mesh(new THREE.BoxGeometry(1.4, hs.y - 16.5 + 0.9, 1.2), M.holster, g, hs.x, (hs.y + 0.9 + 16.5) / 2, -0.4);
  mesh(new THREE.CircleGeometry(0.3, 24), M.bore, g, hs.x, hs.y, 0.21);
  mesh(new THREE.TorusGeometry(0.36, 0.05, 8, 24), M.steel, g, hs.x, hs.y, 0.22);
  mesh(new THREE.BoxGeometry(1.0, 0.8, 0.5), M.holster, g, hs.x, hs.y, -1.25);
  c.led = mesh(new THREE.SphereGeometry(0.16, 12, 12), M.ledOn, g, hs.x, hs.y + 0.55, -1.25);

  // Ball lock pin: shaft, locking balls, head, push button and pull ring
  c.pin = new THREE.Group();
  g.add(c.pin);
  mesh(cylZ(0.25, 2.2, 28), M.pin, c.pin);
  for (const a of [0, Math.PI]) mesh(new THREE.SphereGeometry(0.09, 10, 10), M.steel, c.pin, 0.22 * Math.cos(a), 0.22 * Math.sin(a), -0.95);
  mesh(cylZ(0.5, 0.3, 28), M.pin, c.pin, 0, 0, 1.25);
  mesh(cylZ(0.2, 0.15, 20), M.steel, c.pin, 0, 0, 1.47);
  const ring = mesh(new THREE.TorusGeometry(0.45, 0.07, 10, 32), M.pin, c.pin, 0, 0, 1.95);
  ring.rotation.y = Math.PI / 2;

  return c;
}

const corners = { right: buildCorner(1), left: buildCorner(-1) };

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
// Sequence: LINKED -> NEUTRAL -> UNLINK -> PARKED (manual) -> RELEASE -> UNPARK -> ALIGN -> LINK -> LINKED (active)
const state = {
  mode: 'active',
  phase: 'LINKED',
  pinT: 0,                 // 0 = pin in rocker, 1 = pin in holster
  heave: 0, roll: 0,       // current inputs (mm)
  heaveTarget: 0, rollTarget: 0,
  scenario: null, scenarioClock: 0,
  psi: { right: 0, left: 0 }, // drive disc (servo) angles
  solution: { right: solveCorner(0), left: solveCorner(0) },
  labels: true, xray: false,
};

const clampTravel = (mm) => THREE.MathUtils.clamp(mm, TRAVEL_LIMITS_MM.rebound, TRAVEL_LIMITS_MM.bump);
const travelsMm = () => ({ right: clampTravel(state.heave + state.roll), left: clampTravel(state.heave - state.roll) });
// The holster micro-switch cuts servo power whenever the pin is seated in the holster
const PIN_HOVER = 0.7; // pin lifted out of the holster, waiting above it
const servoPowered = () => !(state.phase === 'PARKED' || state.phase === 'RELEASE' || ((state.phase === 'UNLINK' || state.phase === 'UNPARK') && state.pinT > 0.97));
const smooth = (t) => t * t * (3 - 2 * t);

// Pin pose along its path: rocker hole -> lift -> transfer -> insert into holster
function pinPose(t, holeXY) {
  const lift = smooth(THREE.MathUtils.clamp(t / 0.3, 0, 1));
  const move = smooth(THREE.MathUtils.clamp((t - 0.3) / 0.4, 0, 1));
  const drop = smooth(THREE.MathUtils.clamp((t - 0.7) / 0.3, 0, 1));
  const x = THREE.MathUtils.lerp(holeXY.x, HP.holster.x, move);
  const y = THREE.MathUtils.lerp(holeXY.y, HP.holster.y, move);
  const z = THREE.MathUtils.lerp(PIN_Z.engaged, PIN_Z.lifted, lift - drop);
  return new THREE.Vector3(x, y, z);
}

// ---------------------------------------------------------------------------
// Pose update
// ---------------------------------------------------------------------------
function updateCorner(c, key, travelMm) {
  const s = solveCorner(travelMm / UNIT_MM);
  state.solution[key] = s;

  // Wishbones and ball joints
  const lbj = v3(s.lbj), ubj = v3(s.ubj);
  c.lowerBall.position.copy(lbj);
  c.upperBall.position.copy(ubj);
  c.legs.forEach((leg, i) => leg.rod.update(leg.from, i < 2 ? lbj : ubj));

  // Upright and wheel
  c.upright.position.copy(lbj);
  c.upright.rotation.z = s.uprightRotation;
  c.wheel.position.copy(v3(s.wheelCentre));
  c.wheel.rotation.z = s.uprightRotation;

  // Pushrod and rocker
  const pa = v3(s.pushrodLower), pb = v3(s.rockerPushrod);
  c.pushrod.update(pa, pb);
  c.pushrodEnds[0].position.copy(pa);
  c.pushrodEnds[1].position.copy(pb);
  c.rocker.rotation.z = s.phi;

  // Drive disc follows the rocker whenever the pin is engaged
  if (state.phase === 'LINKED' || state.phase === 'NEUTRAL') state.psi[key] = s.phi;
  c.disc.rotation.z = state.psi[key];

  // Coilover
  const a = HP.coiloverChassis, b = s.rockerCoilover, L = s.coiloverLength;
  c.coilover.position.set(a.x, a.y, 0);
  c.coilover.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(b.x - a.x, b.y - a.y, 0).normalize());
  c.eyeB.position.y = L;
  const shaftStart = 2.5, shaftEnd = L - 0.4;
  c.shaft.scale.y = Math.max(shaftEnd - shaftStart, 0.05);
  c.shaft.position.y = (shaftStart + shaftEnd) / 2;
  c.perchB.position.y = L - 0.75;
  const newSpring = Math.max(L - 0.75 - 1.06 - 0.06, 0.2);
  if (Math.abs(newSpring - c.springLength) > 0.004) {
    c.spring.geometry.dispose();
    c.spring.geometry = springGeometry(newSpring);
    c.springLength = newSpring;
  }

  // Ball lock pin
  c.pin.position.copy(pinPose(state.pinT, s.pinHole));

  // Micro-switch LED: green = servo powered, red = pin in holster (power cut)
  c.led.material = servoPowered() ? M.ledOn : M.ledOff;
}

function updateAssembly() {
  const t = travelsMm();
  updateCorner(corners.right, 'right', t.right);
  updateCorner(corners.left, 'left', t.left);
  // Body heave and roll so that both contact patches stay on the ground
  const sR = t.right / UNIT_MM, sL = t.left / UNIT_MM;
  car.position.y = -(sR + sL) / 2;
  car.rotation.z = -(sR - sL) / TRACK;
  car.updateMatrixWorld(true);
}

// ---------------------------------------------------------------------------
// Sequencer and scenarios
// ---------------------------------------------------------------------------
const SCENARIOS = {
  brake: (t) => { const p = (t % 3.2) / 3.2; const k = p < 0.15 ? smooth(p / 0.15) : p < 0.55 ? 1 : p < 0.7 ? 1 - smooth((p - 0.55) / 0.15) : 0; return { heave: 11 * k, roll: 0 }; },
  corner: (t) => ({ heave: 2 * Math.abs(Math.sin(t * 1.4)), roll: 9 * Math.sin(t * 1.4) }),
  kerb: (t) => { const p = (t % 1.6) / 1.6; const k = p < 0.25 ? Math.sin((p / 0.25) * Math.PI) : 0; return { heave: 6 * k, roll: 6 * k }; },
};

const approach = (cur, target, rate, dt) => cur + (target - cur) * (1 - Math.exp(-rate * dt));

function stepSequence(dt) {
  let busy = true;
  if (state.scenario && state.phase === 'LINKED') {
    state.scenarioClock += dt;
    const s = SCENARIOS[state.scenario](state.scenarioClock);
    state.heaveTarget = s.heave; state.rollTarget = s.roll;
  }
  const rate = state.scenario ? 16 : 8;
  state.heave = approach(state.heave, state.heaveTarget, rate, dt);
  state.roll = approach(state.roll, state.rollTarget, rate, dt);
  const settled = Math.abs(state.heave - state.heaveTarget) < 0.02 && Math.abs(state.roll - state.rollTarget) < 0.02;

  switch (state.phase) {
    case 'NEUTRAL': // servo brings the corner back to static ride height to unload the pin
      state.heaveTarget = state.rollTarget = 0;
      if (settled) state.phase = 'UNLINK';
      break;
    case 'UNLINK':
      state.pinT = Math.min(state.pinT + dt * SEQUENCE_SPEED, 1);
      if (state.pinT >= 1) state.phase = 'PARKED';
      break;
    case 'RELEASE': // hands off: the springs return the corner to static ride height
      state.heaveTarget = state.rollTarget = 0;
      if (settled) state.phase = 'UNPARK';
      break;
    case 'UNPARK': // pin lifted out of the holster: micro-switch released, servo power ON
      state.pinT = Math.max(state.pinT - dt * SEQUENCE_SPEED, PIN_HOVER);
      if (state.pinT <= PIN_HOVER) state.phase = 'ALIGN';
      break;
    case 'ALIGN': { // servo powers up and aligns the drive disc hole with the rocker hole
      let aligned = true;
      for (const k of ['right', 'left']) {
        state.psi[k] = approach(state.psi[k], state.solution[k].phi, 6, dt);
        if (Math.abs(state.psi[k] - state.solution[k].phi) > 0.002) aligned = false;
      }
      if (aligned) state.phase = 'LINK';
      break;
    }
    case 'LINK':
      state.pinT = Math.max(state.pinT - dt * SEQUENCE_SPEED, 0);
      if (state.pinT <= 0) state.phase = 'LINKED';
      break;
    default:
      busy = !settled || !!state.scenario;
  }
  return busy;
}

// ---------------------------------------------------------------------------
// Labels (HTML elements projected from 3D anchors)
// ---------------------------------------------------------------------------
const labels = [];
const R = corners.right;
const worldOf = (obj, x, y, z) => obj.localToWorld(new THREE.Vector3(x, y, z));
function addLabel(text, anchor, minor = false) {
  const el = document.createElement('div');
  el.className = 'label' + (minor ? ' minor' : '');
  el.textContent = text;
  document.body.appendChild(el);
  labels.push({ el, anchor });
}
addLabel('Monocoque', () => worldOf(car, -4, 10, 6.3), true);
addLabel('Rocker', () => worldOf(R.group, HP.rockerPivot.x + 1.2, HP.rockerPivot.y + 2.9, 0));
addLabel('Servo', () => worldOf(R.group, HP.rockerPivot.x + 1.6, HP.rockerPivot.y + 1.6, SERVO_Z));
addLabel('Drive disc', () => worldOf(R.group, HP.rockerPivot.x + 2.0, HP.rockerPivot.y - 0.9, DRIVE_DISC_Z), true);
addLabel('Ball lock pin', () => worldOf(R.pin, 0, -0.9, 1.4));
addLabel('Pin holster', () => worldOf(R.group, HP.holster.x - 0.4, 16.3, 0.4));
addLabel('Micro-switch', () => worldOf(R.group, HP.holster.x, HP.holster.y + 1.0, -1.25), true);
addLabel('Coilover', () => { const s = state.solution.right; return worldOf(R.group, (s.rockerCoilover.x + HP.coiloverChassis.x) / 2, HP.coiloverChassis.y + 1.0, 0); });
addLabel('Pushrod', () => { const s = state.solution.right; return worldOf(R.group, (s.pushrodLower.x + s.rockerPushrod.x) / 2 + 1.2, (s.pushrodLower.y + s.rockerPushrod.y) / 2, 0); });
addLabel('Upper wishbone', () => worldOf(R.group, 11.5, HP.upperInboard.y + 1.0, WISHBONE_LEG_Z.upper * 0.55), true);
addLabel('Lower wishbone', () => worldOf(R.group, 11.5, HP.lowerInboard.y - 1.1, WISHBONE_LEG_Z.lower * 0.55), true);
addLabel('Upright', () => worldOf(R.upright, -1.4, 7.5, 0), true);
addLabel('Front-right wheel', () => worldOf(R.wheel, 3.4, -9.6, 0));

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------
const $ = (id) => document.getElementById(id);
const ui = {
  heave: $('heave'), roll: $('roll'), heaveLabel: $('heaveLabel'), rollLabel: $('rollLabel'),
  heaveValue: $('heaveValue'), rollValue: $('rollValue'), hint: $('hint'),
  btnActive: $('btnActive'), btnManual: $('btnManual'), scenarios: $('scenarios'),
};
Object.assign(ui.heave, { min: INPUT_RANGE_MM.heave[0], max: INPUT_RANGE_MM.heave[1], step: 0.5, value: 0 });
Object.assign(ui.roll, { min: INPUT_RANGE_MM.roll[0], max: INPUT_RANGE_MM.roll[1], step: 0.5, value: 0 });

const HINTS = {
  LINKED: 'Active mode: the ball lock pin locks the rocker to the servo drive disc. The servo turns the rocker, which drives the whole corner: pushrod, upright, wishbones, wheel and coilover.',
  NEUTRAL: 'Returning to static ride height so the pin is unloaded…',
  UNLINK: 'Pulling the pin out of the rocker and parking it in the holster…',
  PARKED: 'Manual mode: the pin sits in the holster and presses the micro-switch, so servo power is OFF. The rocker turns freely on its bearing and never drives the servo. The sliders act as your hands: the suspension moves exactly as in active mode.',
  RELEASE: 'Hands off: the springs bring the suspension back to static ride height…',
  UNPARK: 'Lifting the pin out of the holster: micro-switch released, servo power ON…',
  ALIGN: 'Servo powered: aligning the drive disc hole with the rocker hole…',
  LINK: 'Inserting the pin through rocker and drive disc…',
};

function setReadout(id, text, cls = '') { const el = $(id); el.textContent = text; el.className = cls; }

function refreshUI() {
  const active = state.mode === 'active';
  const stable = state.phase === 'LINKED' || state.phase === 'PARKED';
  ui.btnActive.classList.toggle('on', active);
  ui.btnManual.classList.toggle('on', !active);
  ui.heave.disabled = ui.roll.disabled = !stable;
  ui.heaveLabel.textContent = active ? 'Heave – servo command' : 'Heave – push by hand';
  ui.rollLabel.textContent = active ? 'Roll – servo command' : 'Roll – push by hand';
  ui.scenarios.querySelectorAll('button').forEach((b) => {
    b.disabled = !(active && state.phase === 'LINKED');
    b.classList.toggle('on', state.scenario === b.dataset.scenario);
  });
  ui.hint.textContent = HINTS[state.phase];
}

function refreshReadouts() {
  const t = travelsMm(), s = state.solution.right;
  const locked = state.phase === 'LINKED' || state.phase === 'NEUTRAL' || (state.phase === 'UNLINK' && state.pinT < 0.08) || (state.phase === 'LINK' && state.pinT < 0.08);
  const parked = state.phase === 'PARKED' || state.phase === 'RELEASE';
  setReadout('rCoupling', locked ? 'Locked to servo' : 'Free (bearing)', locked ? 'accent' : 'ok');
  setReadout('rPin', locked ? 'In rocker' : parked ? 'In holster' : 'Moving', '');
  setReadout('rPower', servoPowered() ? 'ON' : 'OFF', servoPowered() ? 'accent' : 'ok');
  setReadout('rTravel', `${t.left.toFixed(1)} / ${t.right.toFixed(1)} mm`);
  setReadout('rRocker', `${THREE.MathUtils.radToDeg(s.phi).toFixed(1)}°`);
  setReadout('rSpring', `${(s.coiloverLength * UNIT_MM).toFixed(1)} mm`);
  setReadout('rMotion', motionRatio(t.right / UNIT_MM).toFixed(2));
  setReadout('rCamber', `${THREE.MathUtils.radToDeg(s.camber).toFixed(2)}°`);
  ui.heaveValue.textContent = `${state.heaveTarget.toFixed(1)} mm`;
  ui.rollValue.textContent = `${state.rollTarget.toFixed(1)} mm`;
  if (state.scenario) { ui.heave.value = state.heaveTarget; ui.roll.value = state.rollTarget; }
}

function setMode(mode) {
  if (mode === state.mode) return;
  if (mode === 'manual' && state.phase === 'LINKED') {
    state.mode = 'manual'; state.scenario = null; state.phase = 'NEUTRAL';
  } else if (mode === 'active' && state.phase === 'PARKED') {
    state.mode = 'active'; state.phase = 'RELEASE';
  } else return; // ignore while a transition is running
  state.heaveTarget = state.rollTarget = 0;
  ui.heave.value = ui.roll.value = 0;
  refreshUI();
  requestRender();
}

ui.btnActive.addEventListener('click', () => setMode('active'));
ui.btnManual.addEventListener('click', () => setMode('manual'));
for (const [el, key] of [[ui.heave, 'heaveTarget'], [ui.roll, 'rollTarget']]) {
  el.addEventListener('input', () => { state[key] = parseFloat(el.value); state.scenario = null; refreshUI(); requestRender(); });
}
ui.scenarios.querySelectorAll('button').forEach((b) => b.addEventListener('click', () => {
  const sc = b.dataset.scenario;
  state.scenario = state.scenario === sc || sc === 'stop' ? null : sc;
  state.scenarioClock = 0;
  if (!state.scenario) { state.heaveTarget = state.rollTarget = 0; ui.heave.value = ui.roll.value = 0; }
  refreshUI();
  requestRender();
}));
$('btnLabels').addEventListener('click', (e) => { state.labels = !state.labels; e.currentTarget.classList.toggle('on', state.labels); requestRender(); });
$('btnXray').addEventListener('click', (e) => {
  state.xray = !state.xray;
  e.currentTarget.classList.toggle('on', state.xray);
  for (const m of xrayMaterials) { m.transparent = state.xray; m.opacity = state.xray ? 0.25 : 1; m.depthWrite = !state.xray; m.needsUpdate = true; }
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
if (isSmallScreen()) { panel.classList.add('collapsed'); $('panelToggleText').textContent = 'Show controls'; }

// ---------------------------------------------------------------------------
// Camera views (auto-framed for the area left visible by the control panel)
// ---------------------------------------------------------------------------
const FRAME = { center: new THREE.Vector3(0, 9, 2), radius: 26 };
const VIEWS = {
  iso: { dir: [0.85, 0.5, 1.0] },
  front: { dir: [0, 0.18, 1] },
  top: { dir: [0, 1, 0.08] },
  rocker: { pos: [15, 27, 14], target: [5, 18.2, -0.3] },
};
const panelOffset = () => (!isSmallScreen() && !panel.classList.contains('collapsed') ? panel.offsetWidth + 14 : 0);
function updateViewOffset() {
  const w = container.clientWidth, h = container.clientHeight, off = panelOffset();
  if (off) camera.setViewOffset(w, h, off / 2, 0, w, h); else camera.clearViewOffset();
}
function fitDistance() {
  const w = Math.max(container.clientWidth - panelOffset(), 1), h = Math.max(container.clientHeight, 1);
  const vFov = THREE.MathUtils.degToRad(camera.fov);
  const hFov = 2 * Math.atan(Math.tan(vFov / 2) * (w / h));
  return (FRAME.radius / Math.sin(Math.min(vFov, hFov) / 2)) * 0.85;
}
let camTween = null;
function goToView(name, instant = false) {
  const v = VIEWS[name];
  const to = v.dir
    ? { target: FRAME.center.clone(), pos: FRAME.center.clone().addScaledVector(new THREE.Vector3(...v.dir).normalize(), fitDistance()) }
    : { pos: new THREE.Vector3(...v.pos), target: new THREE.Vector3(...v.target) };
  if (!v.dir && isSmallScreen()) to.pos.sub(to.target).multiplyScalar(1.4).add(to.target);
  if (instant) { camera.position.copy(to.pos); controls.target.copy(to.target); controls.update(); return; }
  camTween = { from: { pos: camera.position.clone(), target: controls.target.clone() }, to, t: 0 };
  requestRender();
}
document.querySelectorAll('[data-view]').forEach((b) => b.addEventListener('click', () => goToView(b.dataset.view)));

// ---------------------------------------------------------------------------
// Render loop (on demand, to save battery on mobile devices)
// ---------------------------------------------------------------------------
let running = false, last = performance.now(), lastPhase = state.phase;
const tmp = new THREE.Vector3();
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
  const busy = stepSequence(dt);
  updateAssembly();
  const moving = stepCamera(dt);
  const orbiting = controls.update();
  renderer.render(scene, camera);
  updateLabels();
  refreshReadouts();
  if (state.phase !== lastPhase) { lastPhase = state.phase; refreshUI(); }
  if (busy || moving || orbiting) requestAnimationFrame(tick); else running = false;
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
