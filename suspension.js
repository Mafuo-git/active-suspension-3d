// Suspension kinematics for a 1/4-scale F2-style car (pure math, no rendering).
//
// Frame (ISO 8855, chassis fixed): x forward, y left, z up. Units: mm and radians.
// Origin: ground level, car centre line, front axle.
//
// Each corner is a double wishbone with:
//   - lower and upper wishbones (two inboard pivots each, one ball joint),
//   - an upright carrying the wheel, steered by a tie rod (front) or held by a toe link (rear),
//   - a pushrod (on the lower wishbone) or a pullrod (on the upper wishbone) driving a rocker,
//   - a damper between the rocker and the chassis.
//
// The setup only stores the LEFT side; the right side is its mirror image (y -> -y).
// Ball joint positions are not stored: they follow from the alignment settings
// (KPI, caster, scrub radius, mechanical trail, ball joint heights), so every alignment
// knob changes the real geometry. Moving a ball joint by hand updates those settings back.

export const DEG = Math.PI / 180;

// ---------------------------------------------------------------------------
// Small vector library (plain objects, no allocation-heavy classes)
// ---------------------------------------------------------------------------
export const vec = (x = 0, y = 0, z = 0) => ({ x, y, z });
export const add = (a, b) => vec(a.x + b.x, a.y + b.y, a.z + b.z);
export const sub = (a, b) => vec(a.x - b.x, a.y - b.y, a.z - b.z);
export const scale = (a, k) => vec(a.x * k, a.y * k, a.z * k);
export const dot = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
export const cross = (a, b) => vec(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
export const len = (a) => Math.hypot(a.x, a.y, a.z);
export const dist = (a, b) => len(sub(a, b));
export const norm = (a) => scale(a, 1 / (len(a) || 1));
export const lerp = (a, b, t) => add(a, scale(sub(b, a), t));
export const mirror = (a) => vec(a.x, -a.y, a.z);
export const clone = (a) => vec(a.x, a.y, a.z);

// Rotate point p about the axis (origin o, unit direction u) by angle a (Rodrigues)
export function rotateAbout(p, o, u, a) {
  const v = sub(p, o), c = Math.cos(a), s = Math.sin(a);
  const r = add(add(scale(v, c), scale(cross(u, v), s)), scale(u, dot(u, v) * (1 - c)));
  return add(o, r);
}

// Point p0 turns about axis (o, u). Find the angle that puts it at distance L from point c.
// Returns the solution closest to `ref`, or NaN when the links cannot be assembled.
export function circleSphere(p0, o, u, c, L, ref = 0) {
  const centre = add(o, scale(u, dot(sub(p0, o), u)));
  const e1 = sub(p0, centre), e2 = cross(u, e1);
  const d = sub(centre, c);
  const A = 2 * dot(e1, d), B = 2 * dot(e2, d);
  const K = L * L - dot(d, d) - dot(e1, e1);
  const R = Math.hypot(A, B);
  if (R < 1e-9 || Math.abs(K) > R) return NaN;
  const base = Math.atan2(B, A), w = Math.acos(K / R);
  const wrap = (x) => Math.atan2(Math.sin(x), Math.cos(x));
  const a1 = wrap(base + w), a2 = wrap(base - w);
  return Math.abs(wrap(a1 - ref)) <= Math.abs(wrap(a2 - ref)) ? a1 : a2;
}

// 2D line intersection (lines through p1-p2 and p3-p4 in the plane of components i, j)
function intersect2(p1, p2, p3, p4) {
  const d1x = p2[0] - p1[0], d1y = p2[1] - p1[1], d2x = p4[0] - p3[0], d2y = p4[1] - p3[1];
  const den = d1x * d2y - d1y * d2x;
  if (Math.abs(den) < 1e-9) return null;
  const t = ((p3[0] - p1[0]) * d2y - (p3[1] - p1[1]) * d2x) / den;
  return [p1[0] + d1x * t, p1[1] + d1y * t];
}

// ---------------------------------------------------------------------------
// Default setup (1/4 scale of a 2018-onwards F2-style car, values in mm / degrees)
// ---------------------------------------------------------------------------
export const DEFAULT_SETUP = {
  vehicle: {
    wheelbase: 784,      // F2: ~3135 mm
    cgHeight: 70,        // centre of gravity above ground
    brakeBiasFront: 0.6, // share of braking force on the front axle
    rackPerRev: 34,      // rack travel per steering wheel turn (mm/rev)
    travelBump: 18,      // wheel travel limits (bump stops), mm
    travelDroop: 15,
  },
  front: {
    track: 380, tyreRadius: 84, tyreWidth: 76,
    rod: 'push',
    alignment: { camber: -2.5, toe: 0, caster: 6, kpi: 8, scrub: 16, trail: 6, lbjHeight: 40, ubjHeight: 120 },
    rideHeight: 0,       // ride height change from design, set with the pushrod length (mm)
    points: {
      lcaFront: vec(95, 45, 42), lcaRear: vec(-85, 45, 44),
      ucaFront: vec(80, 52, 119), ucaRear: vec(-80, 52, 113),
      tieRodInner: vec(40, 48, 84.5), tieRodOuter: vec(40, 165, 86),
    },
    push: {
      rodOuter: vec(2, 158, 50),
      rockerPivot: vec(-12, 40, 150), rockerAxis: vec(-12, 27.1, 134.7),
      rockerRod: vec(10, 40.8, 149.3), rockerDamper: vec(-12, 26.3, 161.6), damperChassis: vec(-84, 26.3, 161.6),
    },
    pull: {
      rodOuter: vec(-5, 150, 118),
      rockerPivot: vec(-12, 42, 40), rockerAxis: vec(-12, 53.7, 23.8),
      rockerRod: vec(9.9, 44, 41.4), rockerDamper: vec(-12, 27.4, 29.5), damperChassis: vec(60, 27.4, 29.5),
    },
  },
  rear: {
    track: 365, tyreRadius: 84, tyreWidth: 101,
    rod: 'pull',
    alignment: { camber: -1.5, toe: 0.25, caster: 2, kpi: 4, scrub: 12, trail: 0, lbjHeight: 40, ubjHeight: 125 },
    rideHeight: 0,
    points: {
      lcaFront: vec(-689, 38, 46), lcaRear: vec(-844, 32, 38),
      ucaFront: vec(-714, 40, 119), ucaRear: vec(-854, 36, 117),
      tieRodInner: vec(-829, 38, 68.5), tieRodOuter: vec(-824, 160, 70),
    },
    push: {
      rodOuter: vec(-750, 128, 52),
      rockerPivot: vec(-744, 36, 140), rockerAxis: vec(-744, 22.2, 125.5),
      rockerRod: vec(-765.8, 38, 138.1), rockerDamper: vec(-744, 23, 152.4), damperChassis: vec(-672, 23, 152.4),
    },
    pull: {
      rodOuter: vec(-750, 108, 122),
      rockerPivot: vec(-734, 50, 52), rockerAxis: vec(-734, 34.6, 64.8),
      rockerRod: vec(-756, 50.8, 53), rockerDamper: vec(-734, 61.5, 65.9), damperChassis: vec(-662, 61.5, 65.9),
    },
  },
};

export const cloneSetup = (s) => JSON.parse(JSON.stringify(s));

// Editable points of an axle, with their role (for the UI and the 3D gizmo)
export const POINT_INFO = {
  lcaFront: { name: 'Lower wishbone – front pivot', group: 'wishbone' },
  lcaRear: { name: 'Lower wishbone – rear pivot', group: 'wishbone' },
  ucaFront: { name: 'Upper wishbone – front pivot', group: 'wishbone' },
  ucaRear: { name: 'Upper wishbone – rear pivot', group: 'wishbone' },
  lbj: { name: 'Lower ball joint', group: 'upright', derived: true },
  ubj: { name: 'Upper ball joint', group: 'upright', derived: true },
  tieRodInner: { name: 'Tie rod – inner (rack)', nameRear: 'Toe link – inner', group: 'steering' },
  tieRodOuter: { name: 'Tie rod – outer (steering arm)', nameRear: 'Toe link – outer', group: 'steering' },
  rodOuter: { name: 'Push/pullrod – wishbone end', group: 'rod' },
  rockerPivot: { name: 'Rocker pivot', group: 'rocker' },
  rockerAxis: { name: 'Rocker axis direction', group: 'rocker' },
  rockerRod: { name: 'Rocker – rod pickup', group: 'rocker' },
  rockerDamper: { name: 'Rocker – damper pickup', group: 'rocker' },
  damperChassis: { name: 'Damper – chassis mount', group: 'damper' },
};
export const ROD_KEYS = ['rodOuter', 'rockerPivot', 'rockerAxis', 'rockerRod', 'rockerDamper', 'damperChassis'];

// ---------------------------------------------------------------------------
// Static geometry of one corner, built from the setup
// ---------------------------------------------------------------------------
const axleX = (setup, axle) => (axle === 'front' ? 0 : -setup.vehicle.wheelbase);

// Wheel centre, spin axis (pointing outboard) and contact patch of the LEFT wheel at design ride height
export function staticWheel(setup, axle) {
  const ax = setup[axle], al = ax.alignment;
  const g = al.camber * DEG, t = al.toe * DEG, r = ax.tyreRadius;
  // camber: top of the wheel inboard is negative; toe-in is positive
  let a = vec(0, Math.cos(g), -Math.sin(g));
  a = vec(a.y * Math.sin(t), a.y * Math.cos(t), a.z);
  const wc = vec(axleX(setup, axle), ax.track / 2, r * Math.sqrt(1 - a.z * a.z));
  const down = add(vec(0, 0, -1), scale(a, a.z));
  const cp = add(wc, scale(norm(down), r));
  return { wc, axis: a, cp };
}

function kingpinDir(al) {
  return norm(vec(-Math.tan(al.caster * DEG), -Math.tan(al.kpi * DEG), 1));
}

// Ball joints from the alignment settings
export function ballJoints(setup, axle) {
  const al = setup[axle].alignment, { cp } = staticWheel(setup, axle);
  const k = kingpinDir(al);
  const G = vec(cp.x + al.trail, cp.y - al.scrub, 0);
  return {
    lbj: add(G, scale(k, al.lbjHeight / k.z)),
    ubj: add(G, scale(k, al.ubjHeight / k.z)),
    ground: G,
  };
}

// Inverse: alignment settings from ball joints moved by hand (camber and toe are kept)
export function alignmentFromBallJoints(setup, axle, lbj, ubj) {
  const al = setup[axle].alignment, { cp } = staticWheel(setup, axle);
  if (ubj.z - lbj.z < 5) return false;
  const k = norm(sub(ubj, lbj));
  const G = sub(lbj, scale(k, lbj.z / k.z));
  Object.assign(al, {
    caster: Math.atan(-k.x / k.z) / DEG,
    kpi: Math.atan(-k.y / k.z) / DEG,
    trail: G.x - cp.x,
    scrub: cp.y - G.y,
    lbjHeight: lbj.z,
    ubjHeight: ubj.z,
  });
  return true;
}

// Tie rod outer point at the static toe setting (the stored point is at zero toe)
export function staticTieRodOuter(setup, axle) {
  const { lbj, ubj } = ballJoints(setup, axle);
  const k = norm(sub(ubj, lbj));
  return rotateAbout(setup[axle].points.tieRodOuter, lbj, k, -setup[axle].alignment.toe * DEG);
}
export function storeTieRodOuter(setup, axle, p) {
  const { lbj, ubj } = ballJoints(setup, axle);
  const k = norm(sub(ubj, lbj));
  setup[axle].points.tieRodOuter = rotateAbout(p, lbj, k, setup[axle].alignment.toe * DEG);
}

// All editable points of the LEFT corner (static positions), keyed like POINT_INFO
export function cornerPoints(setup, axle) {
  const ax = setup[axle], rod = ax[ax.rod];
  const { lbj, ubj } = ballJoints(setup, axle);
  return {
    ...ax.points,
    tieRodOuter: staticTieRodOuter(setup, axle),
    lbj, ubj,
    ...rod,
  };
}

// Write one moved point back into the setup
export function setCornerPoint(setup, axle, key, p) {
  const ax = setup[axle];
  if (key === 'lbj' || key === 'ubj') {
    const bj = ballJoints(setup, axle);
    return alignmentFromBallJoints(setup, axle, key === 'lbj' ? p : bj.lbj, key === 'ubj' ? p : bj.ubj);
  }
  if (key === 'tieRodOuter') { storeTieRodOuter(setup, axle, p); return true; }
  if (ROD_KEYS.includes(key)) { ax[ax.rod][key] = clone(p); return true; }
  ax.points[key] = clone(p);
  return true;
}

// Rigid frame of the upright from the two ball joints and the tie rod outer point
function uprightFrame(lbj, ubj, tro) {
  const ez = norm(sub(ubj, lbj));
  const w = sub(tro, lbj);
  const ex = norm(sub(w, scale(ez, dot(w, ez))));
  return { o: lbj, ex, ey: cross(ez, ex), ez };
}
const toLocal = (f, p) => { const v = sub(p, f.o); return vec(dot(v, f.ex), dot(v, f.ey), dot(v, f.ez)); };
const toWorld = (f, l) => add(f.o, add(add(scale(f.ex, l.x), scale(f.ey, l.y)), scale(f.ez, l.z)));

// Build a solvable corner. side = +1 (left) or -1 (right).
export function buildCorner(setup, axle, side) {
  const ax = setup[axle];
  const P0 = cornerPoints(setup, axle);
  const w0 = staticWheel(setup, axle);
  const m = side > 0 ? clone : mirror;
  const P = {};
  for (const k in P0) P[k] = m(P0[k]);
  const wc = m(w0.wc), axis = m(w0.axis), cp = m(w0.cp);

  const lcaAxis = norm(sub(P.lcaRear, P.lcaFront));
  const ucaAxis = norm(sub(P.ucaRear, P.ucaFront));
  const rockerAxis = norm(sub(P.rockerAxis, P.rockerPivot));
  const frame0 = uprightFrame(P.lbj, P.ubj, P.tieRodOuter);
  const c = {
    axle, side, setup, P, rod: ax.rod,
    wc0: wc, axis0: axis, cp0: cp, radius: ax.tyreRadius, width: ax.tyreWidth,
    lcaAxis, ucaAxis, rockerAxis,
    kingpinLength: dist(P.lbj, P.ubj),
    tieRodLength: dist(P.tieRodOuter, P.tieRodInner),
    rodLength: dist(P.rodOuter, P.rockerRod),
    damperStatic: dist(P.rockerDamper, P.damperChassis),
    local: {
      wc: toLocal(frame0, wc),
      axisEnd: toLocal(frame0, add(wc, axis)),
      tro: toLocal(frame0, P.tieRodOuter),
    },
    frame0,
    // the wishbone carrying the rod and its axis
    rodOnUpper: ax.rod === 'pull',
  };
  // Ride height: change the rod length so the damper sits at its static length
  // when the wheel has moved by -rideHeight relative to the chassis.
  c.rodLength = rodLengthForRideHeight(c, ax.rideHeight || 0);
  c.thetaRange = travelRange(c);
  return c;
}

// Position of all moving points for a lower wishbone angle theta and a rack displacement.
// Returns null when the linkage cannot be assembled.
export function solveLinks(c, theta, rack = 0, hint = {}) {
  const P = c.P;
  const lbj = rotateAbout(P.lbj, P.lcaFront, c.lcaAxis, theta);
  const psi = circleSphere(P.ubj, P.ucaFront, c.ucaAxis, lbj, c.kingpinLength, hint.psi ?? 0);
  if (Number.isNaN(psi)) return null;
  const ubj = rotateAbout(P.ubj, P.ucaFront, c.ucaAxis, psi);

  // Upright: the tie rod outer point turns about the kingpin axis
  const tri = add(P.tieRodInner, vec(0, rack, 0));
  const k = norm(sub(ubj, lbj));
  const trGuess = toWorld({ o: lbj, ...frameAxes(k, c.frame0) }, c.local.tro);
  const alpha = circleSphere(trGuess, lbj, k, tri, c.tieRodLength, 0);
  if (Number.isNaN(alpha)) return null;
  const tro = rotateAbout(trGuess, lbj, k, alpha);
  const f = uprightFrame(lbj, ubj, tro);
  const wc = toWorld(f, c.local.wc);
  const axis = sub(toWorld(f, c.local.axisEnd), wc);

  // Rod and rocker
  const rodOuter = c.rodOnUpper
    ? rotateAbout(P.rodOuter, P.ucaFront, c.ucaAxis, psi)
    : rotateAbout(P.rodOuter, P.lcaFront, c.lcaAxis, theta);
  const phi = circleSphere(P.rockerRod, P.rockerPivot, c.rockerAxis, rodOuter, c.rodLength, hint.phi ?? 0);
  if (Number.isNaN(phi)) return null;
  const rockerRod = rotateAbout(P.rockerRod, P.rockerPivot, c.rockerAxis, phi);
  const rockerDamper = rotateAbout(P.rockerDamper, P.rockerPivot, c.rockerAxis, phi);

  return { theta, psi, phi, lbj, ubj, tri, tro, wc, axis, rodOuter, rockerRod, rockerDamper,
    damperLength: dist(rockerDamper, P.damperChassis), travel: wc.z - c.wc0.z };
}

// Frame axes that turn frame0 so that its z axis becomes k with the smallest rotation
function frameAxes(k, f0) {
  const z0 = f0.ez, axis = cross(z0, k), s = len(axis), cAng = dot(z0, k);
  if (s < 1e-12) return { ex: f0.ex, ey: f0.ey, ez: k };
  const u = scale(axis, 1 / s), a = Math.atan2(s, cAng), O = vec();
  return { ex: rotateAbout(f0.ex, O, u, a), ey: rotateAbout(f0.ey, O, u, a), ez: k };
}

// Find theta where f(theta) crosses zero, marching away from theta = 0 on the
// branch that assembles (avoids jumping to another assembly mode), then bisecting.
function marchRoot(f) {
  const f0 = f(0);
  if (Number.isNaN(f0)) return NaN;
  if (Math.abs(f0) < 1e-9) return 0;
  const step = 0.01;
  // direction: the one that reduces |f|
  const fp = f(step), fm = f(-step);
  const dir = !Number.isNaN(fp) && (Number.isNaN(fm) || Math.abs(fp) < Math.abs(fm)) ? 1 : -1;
  let a = 0, fa = f0;
  for (let i = 1; i <= 150; i++) {
    const b = dir * step * i, fb = f(b);
    if (Number.isNaN(fb)) return NaN;
    if (fa * fb <= 0) {
      let lo = a, hi = b, flo = fa;
      for (let j = 0; j < 40; j++) {
        const mid = (lo + hi) / 2, fmid = f(mid);
        if (fmid * flo <= 0) hi = mid; else { lo = mid; flo = fmid; }
      }
      return (lo + hi) / 2;
    }
    a = b; fa = fb;
  }
  return NaN;
}

// Wheel travel (mm, + = bump) -> lower wishbone angle
export function thetaForTravel(c, travel, rack = 0) {
  return marchRoot((t) => { const s = solveLinks(c, t, rack); return s ? s.travel - travel : NaN; });
}

function travelRange(c) {
  const v = c.setup.vehicle;
  return { min: thetaForTravel(c, -v.travelDroop), max: thetaForTravel(c, v.travelBump) };
}

function rodLengthForRideHeight(c, rideHeight) {
  if (!rideHeight) return c.rodLength;
  const saved = c.rodLength;
  c.rodLength = 1e9; // rod length does not affect wheel travel: solve the wishbones only
  const th = thetaForTravelNoRod(c, -rideHeight);
  c.rodLength = saved;
  if (Number.isNaN(th)) return saved;
  const rodOuter = c.rodOnUpper
    ? rotateAbout(c.P.rodOuter, c.P.ucaFront, c.ucaAxis, circleSphere(c.P.ubj, c.P.ucaFront, c.ucaAxis, rotateAbout(c.P.lbj, c.P.lcaFront, c.lcaAxis, th), c.kingpinLength, 0))
    : rotateAbout(c.P.rodOuter, c.P.lcaFront, c.lcaAxis, th);
  return dist(rodOuter, c.P.rockerRod);
}

function thetaForTravelNoRod(c, travel) {
  return marchRoot((t) => {
    const lbj = rotateAbout(c.P.lbj, c.P.lcaFront, c.lcaAxis, t);
    const psi = circleSphere(c.P.ubj, c.P.ucaFront, c.ucaAxis, lbj, c.kingpinLength, 0);
    if (Number.isNaN(psi)) return NaN;
    const ubj = rotateAbout(c.P.ubj, c.P.ucaFront, c.ucaAxis, psi);
    const k = norm(sub(ubj, lbj));
    const tro = toWorld({ o: lbj, ...frameAxes(k, c.frame0) }, c.local.tro);
    const alpha = circleSphere(tro, lbj, k, c.P.tieRodInner, c.tieRodLength, 0);
    if (Number.isNaN(alpha)) return NaN;
    const fr = uprightFrame(lbj, ubj, rotateAbout(tro, lbj, k, alpha));
    return toWorld(fr, c.local.wc).z - c.wc0.z - travel;
  });
}

// ---------------------------------------------------------------------------
// Corner on the ground: find the wishbone angle that puts the tyre on the road
// ---------------------------------------------------------------------------
// ground: { n: unit normal of the road in the chassis frame, d: road plane n·p + d = 0 }
function tyreGap(c, s, ground) {
  const a = norm(s.axis), na = dot(ground.n, a);
  return dot(ground.n, s.wc) + ground.d - c.radius * Math.sqrt(Math.max(1 - na * na, 0));
}

export function solveOnGround(c, ground, rack, prev) {
  const { min, max } = c.thetaRange;
  const hint = prev || {};
  const g = (t) => { const s = solveLinks(c, t, rack, hint); return s ? { s, gap: tyreGap(c, s, ground) } : null; };
  let lo = g(min), hi = g(max);
  if (!lo || !hi) {
    const s = solveLinks(c, 0, rack, hint);
    return s ? decorate(c, s, ground, rack, 'locked') : null;
  }
  // the gap grows as the wheel moves up towards the chassis (bump)
  if (lo.gap >= 0) return decorate(c, lo.s, ground, rack, 'droop');   // fully drooped and still above the road: wheel in the air
  if (hi.gap <= 0) return decorate(c, hi.s, ground, rack, 'bump');    // bump stop reached
  let a = min, b = max, fa = lo.gap, fb = hi.gap, best = lo.s;
  for (let i = 0; i < 40; i++) {
    const t = (a + b) / 2, r = g(t);
    if (!r) break;
    best = r.s;
    if (Math.abs(r.gap) < 1e-4) break;
    if (r.gap * fa > 0) { a = t; fa = r.gap; } else { b = t; fb = r.gap; }
  }
  return decorate(c, best, ground, rack, 'ok');
}

// Alignment and geometry values relative to the road
function decorate(c, s, ground, rack, status) {
  const n = ground.n, side = c.side;
  const a = norm(s.axis);
  const heading = norm(scale(cross(a, n), side));
  const outH = scale(cross(n, heading), side);
  const yaw = Math.atan2(heading.y, heading.x);
  const kp = norm(sub(s.ubj, s.lbj));
  // contact patch: lowest point of the tyre towards the road
  const down = sub(scale(n, -1), scale(a, dot(scale(n, -1), a)));
  const cp = add(s.wc, scale(norm(down), c.radius));
  // kingpin axis meets the road
  const tK = -(dot(n, s.lbj) + ground.d) / dot(n, kp);
  const kGround = add(s.lbj, scale(kp, tK));
  const kpLocal = { fwd: dot(kp, heading), out: dot(kp, outH), up: dot(kp, n) };
  return {
    ...s, status, rack, cp, kGround,
    camber: -Math.asin(THREE_clamp(dot(a, n))) / DEG,
    yaw: yaw / DEG,
    toe: (-side * yaw) / DEG,
    caster: Math.atan2(-kpLocal.fwd, kpLocal.up) / DEG,
    kpi: Math.atan2(-kpLocal.out, kpLocal.up) / DEG,
    trail: dot(sub(kGround, cp), heading),
    scrub: dot(sub(cp, kGround), outH),
  };
}
const THREE_clamp = (x) => Math.max(-1, Math.min(1, x));

// Motion ratio = damper travel / wheel travel around the current state
export function motionRatio(c, theta, rack = 0) {
  const e = 0.004;
  const a = solveLinks(c, theta - e, rack), b = solveLinks(c, theta + e, rack);
  if (!a || !b) return NaN;
  return -(b.damperLength - a.damperLength) / (b.travel - a.travel);
}

// ---------------------------------------------------------------------------
// Whole car
// ---------------------------------------------------------------------------
export function buildCar(setup) {
  return {
    setup,
    fl: buildCorner(setup, 'front', 1), fr: buildCorner(setup, 'front', -1),
    rl: buildCorner(setup, 'rear', 1), rr: buildCorner(setup, 'rear', -1),
  };
}
export const CORNERS = ['fl', 'fr', 'rl', 'rr'];

// Chassis pose: heave (mm, + = up), roll (deg, + = left side up), pitch (deg, + = nose up),
// about a reference point at mid wheelbase on the ground. Returns the road plane in chassis frame
// and a function mapping chassis points to the world.
export function chassisPose(setup, pose) {
  const L = setup.vehicle.wheelbase;
  const ref = vec(-L / 2, 0, 0);
  const r = (pose.roll || 0) * DEG, p = (pose.pitch || 0) * DEG;
  // rotation R = Ry(-pitch) * Rx(roll): roll about x, then pitch about y (nose up = negative rotation about y)
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(-p), sp = Math.sin(-p);
  const R = [
    [cp, sp * sr, sp * cr],
    [0, cr, -sr],
    [-sp, cp * sr, cp * cr],
  ];
  const h = pose.heave || 0;
  const toWorldP = (q) => {
    const v = sub(q, ref);
    return vec(
      R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z + ref.x,
      R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z + ref.y,
      R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z + ref.z + h,
    );
  };
  // road plane z_world = 0 expressed in the chassis frame: n = R^T z, n·(p - ref) + h = 0
  const n = vec(R[2][0], R[2][1], R[2][2]);
  return { R, ref, h, toWorld: toWorldP, ground: { n, d: -dot(n, ref) + h } };
}

export function rackFromSteer(setup, steerDeg) {
  return (steerDeg / 360) * setup.vehicle.rackPerRev;
}

export function solveCar(car, pose, prev = {}) {
  const P = chassisPose(car.setup, pose);
  const rack = rackFromSteer(car.setup, pose.steer || 0);
  const out = { pose: P };
  for (const k of CORNERS) {
    const c = car[k];
    out[k] = solveOnGround(c, P.ground, k[0] === 'f' ? rack : 0, prev[k]);
    if (out[k]) out[k].motionRatio = motionRatio(c, out[k].theta, k[0] === 'f' ? rack : 0);
  }
  out.rcFront = rollCentre(car.fl, car.fr, out.fl, out.fr);
  out.rcRear = rollCentre(car.rl, car.rr, out.rl, out.rr);
  return out;
}

// ---------------------------------------------------------------------------
// Geometry analysis
// ---------------------------------------------------------------------------
// Front-view instant centre of one corner (chassis frame, plane x = wheel centre)
export function frontViewIC(c, s) {
  const P = c.P, x = s.wc.x;
  const at = (a, b) => { const t = (x - a.x) / (b.x - a.x || 1e-9); return [a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t]; };
  const l1 = at(P.lcaFront, P.lcaRear), u1 = at(P.ucaFront, P.ucaRear);
  const ic = intersect2(l1, [s.lbj.y, s.lbj.z], u1, [s.ubj.y, s.ubj.z]);
  return ic ? vec(x, ic[0], ic[1]) : null;
}

// Roll centre: where the lines contact patch -> instant centre of both sides cross
export function rollCentre(cl, cr, sl, sr) {
  if (!sl || !sr) return null;
  const icl = frontViewIC(cl, sl), icr = frontViewIC(cr, sr);
  if (!icl || !icr) return null;
  const p = intersect2([sl.cp.y, sl.cp.z], [icl.y, icl.z], [sr.cp.y, sr.cp.z], [icr.y, icr.z]);
  return p ? { point: vec((sl.wc.x + sr.wc.x) / 2, p[0], p[1]), icl, icr } : null;
}

// Side-view instant centre (x, z) of one corner from the wishbone pivot axes
export function sideViewIC(c) {
  const P = c.P;
  const ic = intersect2([P.lcaFront.x, P.lcaFront.z], [P.lcaRear.x, P.lcaRear.z], [P.ucaFront.x, P.ucaFront.z], [P.ucaRear.x, P.ucaRear.z]);
  return ic ? vec(ic[0], c.P.lbj.y, ic[1]) : null;
}

// Anti-dive (front, outboard brakes) and anti-squat (rear, inboard drive) in %
export function antiFeatures(car) {
  const v = car.setup.vehicle, L = v.wheelbase, h = v.cgHeight;
  const res = { antiDive: NaN, antiSquat: NaN, antiLift: NaN, svicFront: sideViewIC(car.fl), svicRear: sideViewIC(car.rl) };
  const f = res.svicFront, r = res.svicRear;
  if (f) { const cp = car.fl.cp0; res.antiDive = ((f.z - cp.z) / (cp.x - f.x)) * (L / h) * v.brakeBiasFront * 100; }
  else res.antiDive = 0;
  if (r) {
    const wc = car.rl.wc0, cp = car.rl.cp0;
    res.antiSquat = ((r.z - wc.z) / (r.x - wc.x)) * (L / h) * 100;
    res.antiLift = ((r.z - cp.z) / (r.x - cp.x)) * (L / h) * (1 - v.brakeBiasFront) * 100;
  } else { res.antiSquat = 0; res.antiLift = 0; }
  return res;
}

// Curves of one corner over its wheel travel (chassis level, no steering)
export function travelCurves(c, steps = 25) {
  const v = c.setup.vehicle, out = [];
  const ground0 = { n: vec(0, 0, 1), d: 0 };
  for (let i = 0; i <= steps; i++) {
    const travel = -v.travelDroop + ((v.travelBump + v.travelDroop) * i) / steps;
    const th = thetaForTravel(c, travel);
    if (Number.isNaN(th)) continue;
    const s = solveLinks(c, th);
    if (!s) continue;
    // values against a level road under the chassis
    const d = decorate(c, s, ground0, 0, 'ok');
    out.push({ travel, camber: d.camber, toe: d.toe, damper: s.damperLength, mr: motionRatio(c, th) });
  }
  return out;
}

// Steering sweep: inner/outer wheel angles and Ackermann percentage
export function steeringCurve(car, steps = 16, maxSteer = 180) {
  const out = [];
  const ground0 = { n: vec(0, 0, 1), d: 0 };
  const L = car.setup.vehicle.wheelbase, T = car.setup.front.track;
  const yaw0 = { l: decorate(car.fl, solveLinks(car.fl, 0), ground0, 0, 'ok').yaw, r: decorate(car.fr, solveLinks(car.fr, 0), ground0, 0, 'ok').yaw };
  for (let i = 1; i <= steps; i++) {
    const steer = (maxSteer * i) / steps, rack = rackFromSteer(car.setup, steer);
    const l = solveLinks(car.fl, 0, rack), r = solveLinks(car.fr, 0, rack);
    if (!l || !r) break;
    const dl = decorate(car.fl, l, ground0, rack, 'ok'), dr = decorate(car.fr, r, ground0, rack, 'ok');
    const inner = dl.yaw - yaw0.l, outer = dr.yaw - yaw0.r;
    const ideal = Math.atan(1 / (1 / Math.tan(outer * DEG) - T / L)) / DEG;
    const ack = Math.abs(ideal - outer) > 1e-6 ? ((inner - outer) / (ideal - outer)) * 100 : NaN;
    out.push({ steer, inner, outer, ackermann: ack });
  }
  return out;
}

// One corner at a given wheel travel, chassis level, no steering (null if out of reach)
export function cornerAtTravel(c, travel) {
  const th = thetaForTravel(c, travel);
  if (Number.isNaN(th)) return null;
  const s = solveLinks(c, th);
  return s ? decorate(c, s, { n: vec(0, 0, 1), d: 0 }, 0, 'ok') : null;
}
