// Physically inspired engine sound for the track test, running in an AudioWorklet.
//
// No oscillators: the sound is built from the engine's own events. A 4-stroke V6 fires each cylinder
// once every two crank turns (every 120° of crank). Each firing sends a pressure pulse down the
// exhaust. Here the pulses (with a little cycle-to-cycle variation, as in a real engine) excite two
// exhaust pipes, one per bank, modelled as waveguides: a delay line the length of the pipe with a
// lossy, inverting reflection at the open end. Their resonances, the collector and the turbine
// (which smooths a turbo engine's note) give the timbre; the pitch follows the firing frequency
// (rpm / 60 × 3, 500 Hz at 10 000 rpm). On top: induction noise, straight-cut gear whine, turbo
// whistle, blow-off valve on lift-off, overrun pops and the ignition cut at the rev limiter.
//
// Parameters (k-rate): rpm, load (0..1 throttle actually delivered), limiter (0/1), level.

const TAU = Math.PI * 2;

class Biquad {
  constructor() { this.b0 = 1; this.b1 = 0; this.b2 = 0; this.a1 = 0; this.a2 = 0; this.x1 = 0; this.x2 = 0; this.y1 = 0; this.y2 = 0; }
  // RBJ cookbook
  set(type, f, q, sr, gainDb = 0) {
    const w = (TAU * Math.min(f, sr * 0.45)) / sr, c = Math.cos(w), s = Math.sin(w), al = s / (2 * q);
    let b0, b1, b2, a0, a1, a2;
    if (type === 'lp') { b0 = (1 - c) / 2; b1 = 1 - c; b2 = (1 - c) / 2; a0 = 1 + al; a1 = -2 * c; a2 = 1 - al; }
    else if (type === 'hp') { b0 = (1 + c) / 2; b1 = -(1 + c); b2 = (1 + c) / 2; a0 = 1 + al; a1 = -2 * c; a2 = 1 - al; }
    else if (type === 'bp') { b0 = al; b1 = 0; b2 = -al; a0 = 1 + al; a1 = -2 * c; a2 = 1 - al; }
    else { // peaking
      const A = 10 ** (gainDb / 40);
      b0 = 1 + al * A; b1 = -2 * c; b2 = 1 - al * A; a0 = 1 + al / A; a1 = -2 * c; a2 = 1 - al / A;
    }
    this.b0 = b0 / a0; this.b1 = b1 / a0; this.b2 = b2 / a0; this.a1 = a1 / a0; this.a2 = a2 / a0;
    return this;
  }
  run(x) {
    const y = this.b0 * x + this.b1 * this.x1 + this.b2 * this.x2 - this.a1 * this.y1 - this.a2 * this.y2;
    this.x2 = this.x1; this.x1 = x; this.y2 = this.y1; this.y1 = y;
    return y;
  }
}

// Exhaust pipe: delay line with a lossy, inverting reflection at the open end
class Pipe {
  constructor(sr, length) {
    this.n = Math.max(8, Math.round(((2 * length) / 343) * sr));
    this.buf = new Float32Array(this.n);
    this.i = 0; this.lp = 0;
  }
  run(x, refl, damp) {
    const back = this.buf[this.i];
    this.lp += (back - this.lp) * damp; // wall losses: high frequencies die faster
    const y = x - refl * this.lp;
    this.buf[this.i] = y;
    this.i = (this.i + 1) % this.n;
    return y;
  }
}

// Box-Muller
function gauss() { return Math.sqrt(-2 * Math.log(Math.random() + 1e-12)) * Math.cos(TAU * Math.random()); }

export class EngineDSP {
  constructor(sr) {
    this.sr = sr;
    this.phase = 0;          // crank cycle (0..1 = 720°)
    this.rpm = 3500; this.load = 0; this.level = 1; this.limiter = 0;
    // firing order 1-4-2-5-3-6 split over the two banks, slightly uneven (header lengths, tolerances)
    this.fire = [0, 1, 2, 3, 4, 5].map((k) => ({
      at: k / 6 + (Math.random() - 0.5) * 0.006,
      bank: k % 2,
      gain: 1 + (Math.random() - 0.5) * 0.12,
    }));
    this.exc = [0, 0]; this.noiseExc = [0, 0];
    this.pipes = [new Pipe(sr, 0.74), new Pipe(sr, 0.81)];
    this.secondary = [new Pipe(sr, 0.29), new Pipe(sr, 0.33)];
    this.body1 = new Biquad().set('pk', 160, 0.9, sr, 7);
    this.body2 = new Biquad().set('pk', 620, 1.4, sr, 4);
    this.body3 = new Biquad().set('pk', 1700, 2, sr, 2.5);
    this.tone = new Biquad();        // turbine / tailpipe low-pass, opens with load
    this.hp = new Biquad().set('hp', 38, 0.7, sr);
    this.intakeBp = new Biquad();
    this.whistleBp = new Biquad();
    this.bovHp = new Biquad().set('hp', 1800, 0.7, sr);
    this.boost = 0; this.bov = 0; this.prevLoad = 0;
    this.whinePh = 0;
    this.dc = 0;
    this.k = 0;
  }

  setParams(rpm, load, limiter, level) { this.rpmT = rpm; this.loadT = load; this.limiter = limiter; this.level = level; }

  process(out) {
    const sr = this.sr, n = out.length;
    // smooth the controls over the block
    const rpm0 = this.rpm, rpm1 = this.rpmT ?? rpm0;
    const load0 = this.load, load1 = this.loadT ?? load0;
    // filters that follow the engine state (updated per block)
    const lf = 1100 + 2600 * load1 + rpm1 * 0.12;
    this.tone.set('lp', lf, 0.6, sr);
    this.intakeBp.set('bp', 300 + rpm1 * 0.09, 1.2, sr);
    this.whistleBp.set('bp', 5200 + this.boost * 3800, 18, sr);
    // turbo: boost builds with load and revs; lift-off dumps it through the blow-off valve
    const boostT = Math.max(0, load1 * Math.min(1, (rpm1 - 4500) / 4000));
    if (this.prevLoad > 0.6 && load1 < 0.2 && this.boost > 0.45) this.bov = Math.max(this.bov, this.boost);
    this.prevLoad = load1;
    const overrun = load1 < 0.06 && rpm1 > 5200;
    for (let i = 0; i < n; i++) {
      const t = i / n;
      const rpm = rpm0 + (rpm1 - rpm0) * t, load = load0 + (load1 - load0) * t;
      const dph = rpm / 60 / 2 / sr;
      const prev = this.phase;
      this.phase += dph;
      const wrapped = this.phase >= 1;
      if (wrapped) this.phase -= 1;
      // firing events crossed during this sample
      for (const f of this.fire) {
        const crossed = wrapped ? (f.at > prev || f.at <= this.phase) : (f.at > prev && f.at <= this.phase);
        if (!crossed) continue;
        let a = (0.32 + 0.68 * load) * f.gain * (1 + 0.07 * gauss());
        let noisy = 0.25 + 0.2 * (1 - load);
        if (this.limiter && Math.random() < 0.55) { a *= 0.12; noisy = 0.8; } // ignition cut
        if (overrun) {
          a = 0.16 * f.gain * (1 + 0.15 * gauss());
          if (Math.random() < 0.025) { a = 1.1 + Math.random() * 0.6; noisy = 1.6; } // pop in the exhaust
        }
        this.exc[f.bank] += a;
        this.noiseExc[f.bank] += a * noisy;
      }
      // pulse shape: sharp pressure front + short turbulent burst
      let y = 0;
      for (let b = 0; b < 2; b++) {
        const x = this.exc[b] + this.noiseExc[b] * (Math.random() * 2 - 1);
        this.exc[b] *= 0.86;
        this.noiseExc[b] *= 0.955;
        const p = this.pipes[b].run(x, 0.62, 0.55);
        y += this.secondary[b].run(p, 0.35, 0.7);
      }
      // collector / silencer resonances, turbine low-pass
      y = this.body3.run(this.body2.run(this.body1.run(y)));
      y = this.tone.run(y);
      // a bit of saturation for the rasp under load
      const drive = 1.2 + 1.6 * load;
      y = Math.tanh(y * drive * 0.55) / Math.tanh(drive * 0.55);
      // induction roar: noise shaped by the intake, pulsing with the firing
      const intake = this.intakeBp.run((Math.random() * 2 - 1) * (this.exc[0] + this.exc[1] + 0.05)) * 0.35 * (0.2 + load);
      // straight-cut gears and cam drive: faint whine at a fixed multiple of engine speed
      this.whinePh += ((rpm / 60) * 11.5) / sr;
      if (this.whinePh > 1) this.whinePh -= 1;
      const whine = Math.sin(TAU * this.whinePh) * 0.012 * (0.4 + 0.6 * load) * Math.min(1, rpm / 9000);
      // turbo whistle and blow-off
      this.boost += (boostT - this.boost) * (boostT > this.boost ? 0.00004 : 0.00012);
      const whistle = this.whistleBp.run(Math.random() * 2 - 1) * this.boost * 0.05;
      const bov = this.bovHp.run(Math.random() * 2 - 1) * this.bov * 0.12;
      this.bov *= 0.99985;
      let s = y * 0.8 + intake + whine + whistle + bov;
      s = this.hp.run(s);
      out[i] = s * this.level;
    }
    this.rpm = rpm1; this.load = load1;
  }
}

// AudioWorklet wrapper (only defined inside an AudioWorkletGlobalScope)
if (typeof registerProcessor === 'function') {
  class EngineProcessor extends AudioWorkletProcessor {
    static get parameterDescriptors() {
      return [
        { name: 'rpm', defaultValue: 3500, minValue: 0, maxValue: 12000, automationRate: 'k-rate' },
        { name: 'load', defaultValue: 0, minValue: 0, maxValue: 1, automationRate: 'k-rate' },
        { name: 'limiter', defaultValue: 0, minValue: 0, maxValue: 1, automationRate: 'k-rate' },
        { name: 'level', defaultValue: 1, minValue: 0, maxValue: 2, automationRate: 'k-rate' },
      ];
    }
    constructor() { super(); this.dsp = new EngineDSP(sampleRate); }
    process(inputs, outputs, p) {
      const out = outputs[0][0];
      if (!out) return true;
      this.dsp.setParams(p.rpm[0], p.load[0], p.limiter[0] > 0.5 ? 1 : 0, p.level[0]);
      this.dsp.process(out);
      for (let c = 1; c < outputs[0].length; c++) outputs[0][c].set(out);
      return true;
    }
  }
  registerProcessor('f2-engine', EngineProcessor);
}
