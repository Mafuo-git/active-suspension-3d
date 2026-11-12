// Planar kinematics of one front corner (right side) of a 1/4-scale F2-style car, seen from the front.
// Pure math, no Three.js dependency, so it can be unit-tested with Node.
//
// Frame: x = lateral (positive outboard), y = vertical (positive up), origin on the car centre line at ground level.
// Units: 1 unit = 10 mm.
// Layout: double wishbone, pushrod from the upright to a rocker on top of the monocoque,
// coilover lying across the top of the chassis to a central mount.

export const UNIT_MM = 10;

const v = (x, y) => ({ x, y });
const add = (a, b) => v(a.x + b.x, a.y + b.y);
const sub = (a, b) => v(a.x - b.x, a.y - b.y);
const length = (a) => Math.hypot(a.x, a.y);
const rotate = (p, angle) => v(p.x * Math.cos(angle) - p.y * Math.sin(angle), p.x * Math.sin(angle) + p.y * Math.cos(angle));

// Hard points at static ride height
export const HP = {
  lowerInboard: v(6.5, 4.5),     // lower wishbone chassis pivot axis
  upperInboard: v(7.0, 13.0),    // upper wishbone chassis pivot axis
  lowerBallJoint: v(19.5, 4.2),
  upperBallJoint: v(18.8, 14.0),
  wheelCentre: v(22.0, 9.0),     // tyre radius 9 -> contact patch on the ground
  pushrodLower: v(18.2, 5.6),    // pushrod pick-up on the upright
  rockerPivot: v(7.0, 18.4),     // rocker axis (longitudinal), also the servo axis
  rockerPushrod: v(2.0, 1.5),    // pushrod mount, relative to the rocker pivot
  rockerCoilover: v(-0.3, 2.3),  // coilover mount, relative to the rocker pivot
  coiloverChassis: v(0.8, 20.7), // coilover mount on the central bracket
  pinHole: v(0.0, -1.0),         // coupling pin hole in rocker hub and drive disc, relative to the pivot
  holster: v(3.6, 17.4),         // pin holster (parking position) on the chassis
};

export const TRAVEL_LIMITS_MM = { bump: 15, rebound: -12 };

const LOWER_LENGTH = length(sub(HP.lowerBallJoint, HP.lowerInboard));
const UPPER_LENGTH = length(sub(HP.upperBallJoint, HP.upperInboard));
const KINGPIN_LENGTH = length(sub(HP.upperBallJoint, HP.lowerBallJoint));
const LOWER_ANGLE_0 = Math.atan2(HP.lowerBallJoint.y - HP.lowerInboard.y, HP.lowerBallJoint.x - HP.lowerInboard.x);
const KINGPIN_ANGLE_0 = Math.atan2(HP.upperBallJoint.y - HP.lowerBallJoint.y, HP.upperBallJoint.x - HP.lowerBallJoint.x);

export const rockerPoint = (relative, phi) => add(HP.rockerPivot, rotate(relative, phi));
export const PUSHROD_LENGTH = length(sub(rockerPoint(HP.rockerPushrod, 0), HP.pushrodLower));
export const COILOVER_STATIC_LENGTH = length(sub(rockerPoint(HP.rockerCoilover, 0), HP.coiloverChassis));

// Upper of the two intersections of circles (c0, r0) and (c1, r1)
function circleIntersectionUpper(c0, r0, c1, r1) {
  const d = length(sub(c1, c0));
  const a = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
  const h = Math.sqrt(Math.max(r0 * r0 - a * a, 0));
  const mid = add(c0, v(((c1.x - c0.x) * a) / d, ((c1.y - c0.y) * a) / d));
  const off = v((-(c1.y - c0.y) * h) / d, ((c1.x - c0.x) * h) / d);
  const s1 = add(mid, off), s2 = sub(mid, off);
  return s1.y > s2.y ? s1 : s2;
}

// Bisection on a monotonic function f over [lo, hi]
function bisect(f, target, lo, hi, iterations = 50) {
  const increasing = f(hi) > f(lo);
  for (let i = 0; i < iterations; i++) {
    const mid = (lo + hi) / 2;
    if ((f(mid) < target) === increasing) lo = mid; else hi = mid;
  }
  return (lo + hi) / 2;
}

// Corner pose for a given lower wishbone angle
function cornerFromLowerAngle(alpha) {
  const lbj = add(HP.lowerInboard, v(LOWER_LENGTH * Math.cos(alpha), LOWER_LENGTH * Math.sin(alpha)));
  const ubj = circleIntersectionUpper(lbj, KINGPIN_LENGTH, HP.upperInboard, UPPER_LENGTH);
  const uprightRotation = Math.atan2(ubj.y - lbj.y, ubj.x - lbj.x) - KINGPIN_ANGLE_0;
  const toUpright = (p) => add(lbj, rotate(sub(p, HP.lowerBallJoint), uprightRotation));
  return { lbj, ubj, uprightRotation, wheelCentre: toUpright(HP.wheelCentre), pushrodLower: toUpright(HP.pushrodLower) };
}

// Rocker angle that keeps the pushrod at its fixed length
function rockerAngleFor(pushrodLower) {
  return bisect((phi) => length(sub(rockerPoint(HP.rockerPushrod, phi), pushrodLower)), PUSHROD_LENGTH, -0.9, 0.9);
}

/**
 * Full corner solution for a wheel travel (units, positive = bump, wheel moves up relative to the chassis).
 * Returns every point needed to draw the corner plus engineering outputs.
 */
export function solveCorner(travel) {
  const alpha = bisect((a) => cornerFromLowerAngle(a).wheelCentre.y - HP.wheelCentre.y, travel, LOWER_ANGLE_0 - 0.6, LOWER_ANGLE_0 + 0.6);
  const corner = cornerFromLowerAngle(alpha);
  const phi = rockerAngleFor(corner.pushrodLower);
  const rockerPushrod = rockerPoint(HP.rockerPushrod, phi);
  const rockerCoilover = rockerPoint(HP.rockerCoilover, phi);
  const coiloverLength = length(sub(rockerCoilover, HP.coiloverChassis));
  return {
    ...corner,
    lowerAngle: alpha,
    phi,                       // rocker angle = servo angle when coupled (rad)
    camber: -corner.uprightRotation, // negative = top of the wheel leaning inboard
    rockerPushrod,
    rockerCoilover,
    coiloverLength,
    pinHole: rockerPoint(HP.pinHole, phi),
  };
}

/** Motion ratio (coilover travel / wheel travel) around a given wheel travel, by finite difference. */
export function motionRatio(travel, step = 0.05) {
  const a = solveCorner(travel - step), b = solveCorner(travel + step);
  return Math.abs(b.coiloverLength - a.coiloverLength) / (2 * step);
}

export const pointAt = (p, phi) => rockerPoint(p, phi);
export { rotate, v as vec2 };
