// Vehicle dynamics for the track test (pure math, no rendering).
//
// The 1/4-scale suspension geometry from the lab is simulated at full scale (x4) so speeds, g-forces
// and loads read like a real F2-style car. The model is a planar four-wheel car (x forward, y left,
// yaw) with quasi-static load transfer. At every frame the chassis roll, pitch and heave are fed to the
// suspension kinematics (suspension.js), which return the real camber and steer angle of each wheel:
// camber gain, bump steer, Ackermann and toe all change how the car drives.

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
  grip: 1.7,            // peak tyre friction coefficient
  optimalCamber: -1.5,  // deg, inclination to the road giving peak grip
  steerLock: 180,       // deg of steering wheel at full lock
  tc: true,             // traction control
  abs: true,            // anti-lock brakes
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
  };
}

// Tyre lateral force shape (simplified Magic Formula): peak near 6° slip, mild fall-off after
const B = 13.5, C = 1.65;
const lateralShape = (alpha) => Math.sin(C * Math.atan(B * alpha));

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
    this.kin = null;
    this.kinTimer = 0;
  }

  reset(pose) {
    Object.assign(this, { x: pose.x, y: pose.y, psi: pose.psi, vx: 0, vy: 0, r: 0, axF: 0, ayF: 0, ax: 0, ay: 0, roll: 0, dive: 0, heave: 0, steerDeg: 0 });
    this.downforce = 0;
    const m = this.p.mass * 9.81, wf = this.p.weightFront;
    this.wheels = WHEELS.map((k, i) => ({ key: k, side: i % 2 ? -1 : 1, front: i < 2, fz: (i < 2 ? wf : 1 - wf) * m / 2, fx: 0, fy: 0, alpha: 0, util: 0, slip: '', camber: 0, delta: 0, travel: 0, status: 'ok' }));
    this.updateKinematics(true);
  }

  get speed() { return Math.hypot(this.vx, this.vy); }

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

  step(dt, input, surfaceAt = () => 1) {
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
    const driveForce = input.throttle * Math.min((p.power * 1000) / Math.max(vx, 4), 14000);
    const brakeForce = input.brake * p.maxBrakeForce;
    const engineBrake = input.throttle < 0.05 && vx > 1 ? 450 : 0;

    let Fx = 0, Fy = 0, Mz = 0;
    let alphaF = 0, alphaR = 0;
    this.wheels.forEach((w, i) => {
      const [px, py] = this.pos[i];
      w.fz = fz[i];
      // wheel velocity in the body frame
      const wvx = vx - r * py, wvy = vy + r * px;
      const c = Math.cos(w.delta), s = Math.sin(w.delta);
      const lvx = wvx * c + wvy * s, lvy = -wvx * s + wvy * c; // in the wheel frame
      const alpha = -Math.atan2(lvy, Math.max(Math.abs(lvx), 3));
      w.alpha = alpha;
      // grip: surface, load sensitivity, camber
      const surface = surfaceAt(i);
      const loadFactor = 1 - 0.1 * (fz[i] / 2500 - 1);
      const cam = w.camber - p.optimalCamber;
      const camberFactor = Math.max(0.7, 1 - 0.012 * cam * cam);
      const mu = p.grip * surface * Math.min(Math.max(loadFactor, 0.7), 1.15) * camberFactor;
      const fmax = mu * fz[i];
      // longitudinal force: rear-wheel drive, brakes split by the brake bias
      let fx = 0;
      if (!w.front) fx += driveForce / 2 - engineBrake / 2;
      const bias = w.front ? g.brakeBias : 1 - g.brakeBias;
      if (brakeForce > 0 && Math.abs(lvx) > 0.3) fx -= Math.sign(lvx) * brakeForce * bias / 2;
      // lateral force from the slip angle, then combined slip (friction ellipse)
      const camberThrust = w.side * w.camber * 0.008 * fz[i];
      let fy = fmax * lateralShape(alpha) + camberThrust;
      fy = Math.max(-fmax, Math.min(fmax, fy));
      const room = Math.sqrt(Math.max(fmax * fmax - fy * fy, 0)); // longitudinal grip left
      let slip = '';
      const assisted = (fx < 0 && p.abs) || (fx > 0 && p.tc);
      if (assisted && Math.abs(fx) > Math.max(room, 0.3 * fmax)) {
        // TC / ABS: keep the tyre near its peak and protect the cornering force
        fx = Math.sign(fx) * Math.max(room, 0.3 * fmax);
        slip = fx < 0 ? 'ABS' : 'TC';
        const left = Math.sqrt(Math.max(fmax * fmax - fx * fx, 0));
        fy = Math.max(-left, Math.min(left, fy));
      } else if (Math.abs(fx) > fmax) {
        // locked or spinning wheel: sliding friction, very little side grip left
        fx = Math.sign(fx) * fmax * 0.85;
        fy *= 0.3;
        slip = fx < 0 ? 'lock' : 'spin';
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
      // to the body frame
      const bx = fx * c - fy * s, by = fx * s + fy * c;
      Fx += bx; Fy += by; Mz += px * by - py * bx;
      if (w.front) alphaF += alpha / 2; else alphaR += alpha / 2;
    });
    Fx -= drag + 120 * Math.sign(vx); // drag + rolling resistance

    // --- integrate (semi-implicit Euler) ---
    const axb = Fx / m, ayb = Fy / m;
    this.vx += (axb + r * vy) * dt;
    this.vy += (ayb - r * vx) * dt;
    this.r += (Mz / p.yawInertia) * dt;
    if (this.speed < 0.3 && input.throttle < 0.05) { this.vx *= 0.9; this.vy *= 0.9; this.r *= 0.9; }
    if (this.vx < -0.5 && input.throttle > 0) this.vx = 0;
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
