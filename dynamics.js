// Vehicle dynamics for the track test (pure math, no rendering).
//
// The 1/4-scale suspension geometry from the lab is simulated at full scale (x4) so speeds, g-forces
// and loads read like a real F2-style car. The model is a planar four-wheel car (x forward, y left,
// yaw) with quasi-static load transfer. At every frame the chassis roll, pitch and heave are fed to the
// suspension kinematics (suspension.js), which return the real camber and steer angle of each wheel:
// camber gain, bump steer, Ackermann and toe all change how the car drives.
//
// Also modelled: tyre relaxation length, asphalt / kerb / grass grip and rolling resistance, dirty
// tyres after an excursion, a 6-speed gearbox driven by a turbo V6 torque curve (rev limiter, engine
// braking, auto or manual shifts) and a reverse gear.

import * as S from './suspension.js';

export const SCALE = 4;           // lab model -> full-size car
const G = 9.81, RHO = 1.225;
const DEG = Math.PI / 180;

export const DEFAULT_DRIVE = {
  mass: 755,            // kg, car + driver
  weightFront: 0.44,    // static weight on the front axle
  yawInertia: 1000,     // kg·m²
  power: 460,           // kW
  maxBrakeForce: 30000, // N, all four wheels
  springFront: 220,     // N/mm at the damper
  springRear: 450,      // N/mm at the damper
  arbFront: 800,        // N·m/deg
  arbRear: 500,         // N·m/deg
  downforce: 3.2,       // ClA, m²
  drag: 1.15,           // CdA, m²
  aeroBalance: 0.42,    // share of downforce on the front axle
  grip: 1.75,           // peak tyre friction coefficient (dry asphalt)
  optimalCamber: -1.5,  // deg, inclination to the road giving peak grip
  steerLock: 180,       // deg of steering wheel at full lock
  tc: true,             // traction control
  abs: true,            // anti-lock brakes
  autoGearbox: true,    // automatic upshifts and downshifts
  steerAssist: true,    // keyboard / stick steering limited to the useful slip angle
};

// Engine and gearbox (F2-style turbo V6, 6 speeds)
export const ENGINE = { idle: 3500, launch: 6800, peak: 8750, redline: 10000, upshift: 9850 };
const GEAR_TOP_KMH = [108, 148, 188, 230, 274, 322]; // road speed at the rev limiter in each gear
const REVERSE_TOP_KMH = 38;
const SHIFT_TIME = 0.06; // s without drive during a shift
// Power at the crank as a share of the peak power
function powerShare(rpm) {
  const d = rpm - ENGINE.peak;
  return Math.max(0.12, 1 - (d / (d < 0 ? 7000 : 4200)) ** 2);
}

// Surfaces: grip factor and rolling resistance coefficient
export const SURFACES = {
  asphalt: { name: 'asphalt', mu: 1, roll: 0.015 },
  kerb: { name: 'kerb', mu: 0.86, roll: 0.03 },
  grass: { name: 'grass', mu: 0.42, roll: 0.16 },
};

export const WHEELS = ['fl', 'fr', 'rl', 'rr'];

// Geometry the dynamics need, read from the lab setup
export function geometryFromSetup(setup) {
  const car = S.buildCar(setup);
  const st = S.solveCar(car, {});
  const anti = S.antiFeatures(car);
  const m = (v) => (v * SCALE) / 1000;
  return {
    car,
    L: m(setup.vehicle.wheelbase),
    tf: m(setup.front.track), tr: m(setup.rear.track),
    h: m(setup.vehicle.cgHeight),
    rcF: m(st.rcFront?.point.z ?? 0), rcR: m(st.rcRear?.point.z ?? 0),
    mrF: st.fl.motionRatio, mrR: st.rl.motionRatio,
    antiDive: Number.isFinite(anti.antiDive) ? anti.antiDive / 100 : 0,
    antiSquat: Number.isFinite(anti.antiSquat) ? anti.antiSquat / 100 : 0,
    antiLift: Number.isFinite(anti.antiLift) ? anti.antiLift / 100 : 0,
    brakeBias: setup.vehicle.brakeBiasFront,
    tyreRadius: m(setup.front.tyreRadius),
    steerRatio: steeringRatio(car),
  };
}

// Steering wheel angle / road wheel angle, measured on the kinematics
function steeringRatio(car) {
  const st = S.solveCar(car, { steer: 90 });
  const yaw = (Math.abs(st.fl?.yaw ?? 0) + Math.abs(st.fr?.yaw ?? 0)) / 2;
  return yaw > 0.5 ? 90 / yaw : 8;
}

// Tyre lateral force shape (Magic Formula): peak near 7.5° slip, gentle fall-off past the peak
// (about 90 % of the peak at 20°), like a racing slick
const B = 13, C = 1.4, E = -0.6;
export const PEAK_SLIP = 7.5 * Math.PI / 180;
const lateralShape = (alpha) => { const x = B * alpha; return Math.sin(C * Math.atan(x - E * (x - Math.atan(x)))); };
const RELAXATION = 0.5; // m of rolling for the slip angle to build up

export class VehicleSim {
  constructor(setup, drive = DEFAULT_DRIVE) {
    this.configure(setup, drive);
    this.reset({ x: 0, y: 0, psi: 0 });
  }

  configure(setup, drive) {
    this.setup = setup;
    this.p = { ...DEFAULT_DRIVE, ...drive };
    this.g = geometryFromSetup(setup);
    const { L, tf, tr } = this.g, p = this.p;
    this.a = L * (1 - p.weightFront); // CG to front axle
    this.b = L * p.weightFront;       // CG to rear axle
    this.pos = [[this.a, tf / 2], [this.a, -tf / 2], [-this.b, tr / 2], [-this.b, -tr / 2]];
    // wheel rates (N/m) and roll stiffness (N·m/rad)
    this.kwF = p.springFront * 1000 * this.g.mrF ** 2;
    this.kwR = p.springRear * 1000 * this.g.mrR ** 2;
    this.KrF = 0.5 * this.kwF * tf * tf + p.arbFront / DEG;
    this.KrR = 0.5 * this.kwR * tr * tr + p.arbRear / DEG;
    // overall gear ratios (engine turns per wheel turn)
    const wRed = (ENGINE.redline * 2 * Math.PI) / 60;
    this.ratios = GEAR_TOP_KMH.map((k) => (wRed * this.g.tyreRadius) / (k / 3.6));
    this.reverseRatio = (wRed * this.g.tyreRadius) / (REVERSE_TOP_KMH / 3.6);
    this.kin = null;
    this.kinTimer = 0;
  }

  reset(pose) {
    Object.assign(this, { x: pose.x, y: pose.y, psi: pose.psi, vx: 0, vy: 0, r: 0, axF: 0, ayF: 0, ax: 0, ay: 0, roll: 0, dive: 0, heave: 0, steerDeg: 0 });
    Object.assign(this, { gear: 1, rpm: ENGINE.idle, shiftTimer: 0, limiter: false, shiftEvent: 0, driveForce: 0 });
    this.downforce = 0;
    const m = this.p.mass * G, wf = this.p.weightFront;
    this.wheels = WHEELS.map((k, i) => ({
      key: k, side: i % 2 ? -1 : 1, front: i < 2, fz: (i < 2 ? wf : 1 - wf) * m / 2, fx: 0, fy: 0,
      alpha: 0, util: 0, slip: '', sliding: 0, camber: 0, delta: 0, travel: 0, status: 'ok', dirt: 0, surface: SURFACES.asphalt,
    }));
    this.updateKinematics(true);
  }

  get speed() { return Math.hypot(this.vx, this.vy); }

  // Largest lateral acceleration the tyres can give at this speed (m/s²)
  maxLateral(v = this.speed) {
    return (this.p.grip * (this.p.mass * G + 0.5 * RHO * this.p.downforce * v * v)) / this.p.mass;
  }

  // Gearbox commands (manual shifts, reverse)
  shift(dir) {
    if (this.gear < 1 || this.shiftTimer > 0) return false;
    const g = this.gear + dir;
    if (g < 1 || g > this.ratios.length) return false;
    // refuse a downshift that would over-rev the engine
    if (dir < 0 && this.wheelRpm(g) > ENGINE.redline * 1.02) return false;
    this.gear = g; this.shiftTimer = SHIFT_TIME; this.shiftEvent = dir;
    return true;
  }
  setReverse(on) {
    if (on && this.gear !== -1) { this.gear = -1; this.shiftTimer = 0.25; this.shiftEvent = -1; }
    if (!on && this.gear === -1) { this.gear = 1; this.shiftTimer = 0.25; this.shiftEvent = 1; }
  }
  ratio(g = this.gear) { return g === -1 ? this.reverseRatio : this.ratios[g - 1]; }
  // engine speed imposed by the rear wheels in a given gear
  wheelRpm(g = this.gear) {
    const vr = Math.abs(this.vx);
    return (vr / this.g.tyreRadius) * this.ratio(g) * 60 / (2 * Math.PI);
  }

  // Chassis pose -> suspension kinematics -> camber and steer angle of each wheel
  updateKinematics(force = false) {
    const pose = {
      heave: -(this.heave * 1000) / SCALE, // lab mm, + = body up
      roll: this.roll / DEG,
      pitch: -this.dive / DEG,
      steer: this.steerDeg,
    };
    const sol = S.solveCar(this.g.car, pose, force ? {} : this.kin || {});
    if (!sol.fl || !sol.rl) return;
    this.kin = sol;
    WHEELS.forEach((k, i) => {
      const s = sol[k], w = this.wheels[i];
      if (!s) return;
      w.camber = s.camber;
      w.delta = s.yaw * DEG;
      w.travel = (s.travel * SCALE) / 1000;
      w.status = s.status;
    });
  }

  // Engine and gearbox: returns the drive force at the rear axle (N, + = forward in the body frame)
  powertrain(dt, throttle, rearSpin) {
    const p = this.p;
    if (this.shiftTimer > 0) this.shiftTimer -= dt;
    // automatic gearbox
    if (p.autoGearbox && this.gear > 0 && this.shiftTimer <= 0) {
      const top = this.gear >= this.ratios.length;
      if (!top && this.rpm >= ENGINE.upshift && throttle > 0.05) this.shift(1);
      // part throttle: short-shift like a road car to keep the revs reasonable
      else if (!top && throttle > 0.05 && throttle < 0.6 && this.rpm > 8600 && this.wheelRpm(this.gear + 1) > 6400) this.shift(1);
      else if (this.gear > 1 && this.wheelRpm(this.gear - 1) < (throttle > 0.5 ? 8600 : 7600)) this.shift(-1);
    }
    const ratio = this.ratio();
    const wheelRpm = this.wheelRpm();
    // clutch slips below the launch speed so the engine never stalls
    const slipRpm = ENGINE.idle + throttle * (ENGINE.launch - ENGINE.idle);
    let target = Math.max(wheelRpm, wheelRpm < slipRpm ? slipRpm : 0, ENGINE.idle);
    if (rearSpin) target = Math.max(target, Math.min(ENGINE.redline, wheelRpm * 1.25 + 1800 * throttle));
    if (this.shiftTimer > 0) target = Math.max(ENGINE.idle, wheelRpm);
    // engine inertia
    this.rpm += (target - this.rpm) * (1 - Math.exp(-dt / (target > this.rpm ? 0.07 : 0.05)));
    this.limiter = this.rpm >= ENGINE.redline - 20 && throttle > 0.05;
    if (this.rpm > ENGINE.redline) this.rpm = ENGINE.redline;
    let torque = 0;
    if (this.shiftTimer <= 0) {
      const rpmEng = Math.max(this.rpm, 1000);
      const omega = (rpmEng * 2 * Math.PI) / 60;
      const tDrive = this.limiter ? 0 : (throttle * p.power * 1000 * powerShare(rpmEng)) / omega;
      const tBrake = (1 - throttle) * (12 + rpmEng * 0.0045); // engine braking (pumping + friction)
      torque = tDrive - (wheelRpm > ENGINE.idle * 0.9 ? tBrake : 0);
    }
    const dir = this.gear === -1 ? -1 : 1;
    this.driveForce = (dir * torque * ratio * 0.92) / this.g.tyreRadius;
    return this.driveForce;
  }

  step(dt, input, surfaceAt = () => SURFACES.asphalt) {
    const p = this.p, g = this.g, m = p.mass;
    this.steerDeg = input.steer * p.steerLock;
    this.kinTimer += dt;
    if (this.kinTimer >= 1 / 60) { this.kinTimer = 0; this.updateKinematics(); }

    const vx = this.vx, vy = this.vy, r = this.r;
    const v2 = vx * vx;
    const downforce = 0.5 * RHO * p.downforce * v2;
    const drag = 0.5 * RHO * p.drag * v2 * Math.sign(vx);

    // --- vertical loads (quasi-static load transfer, filtered accelerations) ---
    const h = g.h, L = g.L;
    const hRollAxis = g.rcF + ((g.rcR - g.rcF) * this.a) / L;
    const arm = h - hRollAxis;
    const Kr = this.KrF + this.KrR;
    const latF = (m * this.ayF / g.tf) * ((this.KrF / Kr) * arm + p.weightFront * g.rcF);
    const latR = (m * this.ayF / g.tr) * ((this.KrR / Kr) * arm + (1 - p.weightFront) * g.rcR);
    const lon = (m * this.axF * h) / L; // load moving to the rear
    const staticF = (m * G * p.weightFront) / 2, staticR = (m * G * (1 - p.weightFront)) / 2;
    const aeroF = (downforce * p.aeroBalance) / 2, aeroR = (downforce * (1 - p.aeroBalance)) / 2;
    const fz = [
      staticF + aeroF - lon / 2 - latF,
      staticF + aeroF - lon / 2 + latF,
      staticR + aeroR + lon / 2 - latR,
      staticR + aeroR + lon / 2 + latR,
    ].map((f) => Math.max(f, 0));

    // --- chassis attitude for the kinematics (and the 3D view) ---
    this.roll = (m * this.ayF * arm) / Math.max(Kr - m * G * arm, 1e3);
    const braking = this.axF < 0;
    const lonF = -lon / 2 * (braking ? 1 - g.antiDive : 1);
    const lonR = lon / 2 * (braking ? 1 - g.antiLift : 1 - g.antiSquat);
    const zF = (aeroF + lonF) / this.kwF, zR = (aeroR + lonR) / this.kwR; // compression, m
    this.heave = (zF + zR) / 2;
    this.dive = Math.atan((zF - zR) / L);

    // --- longitudinal demand ---
    const rearSpin = this.wheels[2].slip === 'spin' || this.wheels[3].slip === 'spin';
    const driveForce = this.powertrain(dt, input.throttle, rearSpin);
    const brakeForce = input.brake * p.maxBrakeForce;

    let Fx = 0, Fy = 0, Mz = 0;
    let alphaF = 0, alphaR = 0;
    this.wheels.forEach((w, i) => {
      const [px, py] = this.pos[i];
      w.fz = fz[i];
      // wheel velocity in the body frame
      const wvx = vx - r * py, wvy = vy + r * px;
      const c = Math.cos(w.delta), s = Math.sin(w.delta);
      const lvx = wvx * c + wvy * s, lvy = -wvx * s + wvy * c; // in the wheel frame
      // slip angle, built up over the relaxation length (the tyre carcass needs to roll to deform)
      const alphaSS = -Math.atan2(lvy, Math.max(Math.abs(lvx), 3));
      const kRel = Math.min(1, ((Math.abs(lvx) + 0.5) * dt) / RELAXATION);
      w.alpha += (alphaSS - w.alpha) * kRel;
      const alpha = w.alpha;
      // surface under this tyre; grass leaves the tyre dirty for a while
      const surf = surfaceAt(i) || SURFACES.asphalt;
      w.surface = surf;
      const rolled = Math.abs(lvx) * dt;
      if (surf === SURFACES.grass) w.dirt = Math.min(1, w.dirt + rolled / 6);
      else w.dirt = Math.max(0, w.dirt - rolled / 400);
      // grip: surface, dirt, load sensitivity, camber
      const loadFactor = 1 - 0.1 * (fz[i] / 2500 - 1);
      const cam = w.camber - p.optimalCamber;
      const camberFactor = Math.max(0.7, 1 - 0.012 * cam * cam);
      const mu = p.grip * surf.mu * (1 - 0.15 * w.dirt) * Math.min(Math.max(loadFactor, 0.7), 1.15) * camberFactor;
      const fmax = mu * fz[i];
      // longitudinal force: rear-wheel drive, brakes split by the brake bias, rolling resistance
      let fx = 0;
      if (!w.front) fx += driveForce / 2;
      const bias = w.front ? g.brakeBias : 1 - g.brakeBias;
      if (brakeForce > 0 && Math.abs(lvx) > 0.3) fx -= Math.sign(lvx) * brakeForce * bias / 2;
      fx -= Math.sign(lvx) * surf.roll * fz[i] * Math.min(1, Math.abs(lvx));
      // lateral force from the slip angle, then combined slip (friction ellipse)
      const camberThrust = w.side * w.camber * 0.008 * fz[i] * surf.mu;
      let fy = fmax * lateralShape(alpha) + camberThrust;
      fy = Math.max(-fmax, Math.min(fmax, fy));
      const room = Math.sqrt(Math.max(fmax * fmax - fy * fy, 0)); // longitudinal grip left
      let slip = '';
      const braking = fx * Math.sign(lvx || 1) < 0;
      const assisted = (braking && p.abs) || (!braking && p.tc);
      if (assisted && Math.abs(fx) > Math.max(room, 0.3 * fmax)) {
        // TC / ABS: keep the tyre near its peak and protect the cornering force
        fx = Math.sign(fx) * Math.max(room, 0.3 * fmax);
        slip = braking ? 'ABS' : 'TC';
        const left = Math.sqrt(Math.max(fmax * fmax - fx * fx, 0));
        fy = Math.max(-left, Math.min(left, fy));
      } else if (Math.abs(fx) > fmax) {
        // locked or spinning wheel: sliding friction, very little side grip left
        fx = Math.sign(fx) * fmax * 0.85;
        fy *= 0.3;
        slip = braking ? 'lock' : 'spin';
      } else if (Math.abs(fx) > room) {
        // over the ellipse: the tyre slides, side force drops
        const left = Math.sqrt(Math.max(fmax * fmax - fx * fx, 0));
        fy = Math.max(-left, Math.min(left, fy));
      }
      // fade forces at walking pace to avoid jitter at standstill
      const lowSpeed = Math.min(1, Math.hypot(wvx, wvy) / 1.5);
      fy *= lowSpeed;
      w.fx = fx; w.fy = fy; w.slip = slip;
      w.util = fmax > 1 ? Math.hypot(fx, fy) / fmax : 0;
      // how much the tyre is sliding (for the squeal): past the peak slip angle, or locked / spinning
      const over = Math.abs(alpha) / PEAK_SLIP;
      w.sliding = Math.min(1, Math.max(0, (over - 0.85) * 1.5) + (slip === 'lock' || slip === 'spin' ? 0.8 : slip ? 0.25 : 0)) * Math.min(1, Math.abs(lvx) / 6);
      // to the body frame
      const bx = fx * c - fy * s, by = fx * s + fy * c;
      Fx += bx; Fy += by; Mz += px * by - py * bx;
      if (w.front) alphaF += alpha / 2; else alphaR += alpha / 2;
    });
    Fx -= drag;

    // --- integrate (semi-implicit Euler) ---
    const axb = Fx / m, ayb = Fy / m;
    this.vx += (axb + r * vy) * dt;
    this.vy += (ayb - r * vx) * dt;
    this.r += (Mz / p.yawInertia) * dt;
    // standstill: let the car settle instead of creeping
    if (this.speed < 0.3 && input.throttle < 0.05) { this.vx *= 0.9; this.vy *= 0.9; this.r *= 0.9; }
    const cps = Math.cos(this.psi), sps = Math.sin(this.psi);
    this.x += (this.vx * cps - this.vy * sps) * dt;
    this.y += (this.vx * sps + this.vy * cps) * dt;
    this.psi += this.r * dt;

    // accelerations felt by the driver, filtered like a damped suspension
    this.ax = axb; this.ay = ayb;
    const k = 1 - Math.exp(-dt / 0.09);
    this.axF += (axb - this.axF) * k;
    this.ayF += (ayb - this.ayF) * k;
    // handling balance: front minus rear slip angle (deg), + = understeer
    this.balance = (Math.abs(alphaF) - Math.abs(alphaR)) / DEG * (Math.abs(this.ayF) > 2 ? 1 : 0) || 0;
    this.downforce = downforce;
  }
}
