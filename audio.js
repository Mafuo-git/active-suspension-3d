// Synthesized car sound for the track test (Web Audio API, no sample files).
//
// Engine: a turbo V6 fires 3 times per crankshaft turn, so the fundamental is rpm / 60 × 3
// (500 Hz at 10 000 rpm). A few detuned oscillators at that frequency and its harmonics go through
// a soft-clipping stage and a low-pass filter that opens with the throttle: more load = brighter and
// louder. On top: turbo whistle, tyre squeal from the sliding tyres, kerb rumble, grass noise and
// wind noise that grows with speed. Everything follows the simulation in real time.

export class CarAudio {
  constructor() {
    this.ctx = null;
    this.enabled = true;
    this.volume = 0.55;
    this.lastShift = 0;
    this.limiterPhase = 0;
  }

  // Must be called from a user gesture (click, key press, touch) the first time
  start() {
    if (!this.enabled) return;
    if (!this.ctx) {
      const AC = window.AudioContext || window.webkitAudioContext;
      if (!AC) { this.enabled = false; return; }
      this.ctx = new AC();
      this.build();
    }
    if (this.ctx.state === 'suspended') this.ctx.resume().catch(() => {});
  }
  get running() { return !!this.ctx && this.ctx.state === 'running'; }

  setEnabled(on) {
    this.enabled = on;
    if (!this.ctx) { if (on) this.start(); return; }
    const t = this.ctx.currentTime;
    this.master.gain.cancelScheduledValues(t);
    this.master.gain.setTargetAtTime(on ? this.volume : 0, t, 0.05);
    if (on) this.start();
  }
  suspend() { if (this.ctx && this.ctx.state === 'running') this.ctx.suspend().catch(() => {}); }

  noiseBuffer(seconds = 2) {
    const ctx = this.ctx, n = Math.floor(ctx.sampleRate * seconds);
    const buf = ctx.createBuffer(1, n, ctx.sampleRate), d = buf.getChannelData(0);
    for (let i = 0; i < n; i++) d[i] = Math.random() * 2 - 1;
    return buf;
  }
  noiseSource(buf) {
    const src = this.ctx.createBufferSource();
    src.buffer = buf; src.loop = true; src.start();
    return src;
  }

  build() {
    const ctx = this.ctx;
    this.master = ctx.createGain();
    this.master.gain.value = this.enabled ? this.volume : 0;
    const comp = ctx.createDynamicsCompressor();
    comp.threshold.value = -14; comp.ratio.value = 4;
    this.master.connect(comp).connect(ctx.destination);

    // --- engine ---
    this.engineFilter = ctx.createBiquadFilter();
    this.engineFilter.type = 'lowpass'; this.engineFilter.Q.value = 2.2;
    const shaper = ctx.createWaveShaper();
    const curve = new Float32Array(1024);
    for (let i = 0; i < 1024; i++) { const x = (i / 1023) * 2 - 1; curve[i] = Math.tanh(2.4 * x); }
    shaper.curve = curve;
    this.engineDrive = ctx.createGain();      // into the clipper: more = harsher
    this.engineGain = ctx.createGain();       // overall engine level
    this.engineGain.gain.value = 0;
    this.engineDrive.connect(shaper).connect(this.engineFilter).connect(this.engineGain).connect(this.master);
    // fundamental, half order (uneven firing / exhaust pulse), 2nd and 3rd harmonics
    this.osc = [
      { type: 'sawtooth', mult: 1, gain: 0.5, detune: 0 },
      { type: 'sawtooth', mult: 1, gain: 0.35, detune: 9 },
      { type: 'square', mult: 0.5, gain: 0.22, detune: 0 },
      { type: 'sawtooth', mult: 2, gain: 0.16, detune: -6 },
      { type: 'triangle', mult: 3, gain: 0.08, detune: 4 },
    ].map((o) => {
      const osc = ctx.createOscillator(), g = ctx.createGain();
      osc.type = o.type; osc.detune.value = o.detune; g.gain.value = o.gain;
      osc.connect(g).connect(this.engineDrive); osc.start();
      return { ...o, osc };
    });
    // combustion roughness: noise amplitude-modulated by the firing frequency
    const noise = this.noiseBuffer();
    const rough = this.noiseSource(noise);
    const roughBp = ctx.createBiquadFilter(); roughBp.type = 'bandpass'; roughBp.Q.value = 0.8;
    this.roughGain = ctx.createGain(); this.roughGain.gain.value = 0;
    rough.connect(roughBp).connect(this.roughGain).connect(this.engineDrive);
    this.roughBp = roughBp;

    // --- turbo whistle ---
    this.turbo = ctx.createOscillator(); this.turbo.type = 'sine';
    this.turboGain = ctx.createGain(); this.turboGain.gain.value = 0;
    this.turbo.connect(this.turboGain).connect(this.master); this.turbo.start();

    // --- tyre squeal: band-passed noise with a little pitch wobble ---
    const sq = this.noiseSource(noise);
    this.squealBp = ctx.createBiquadFilter(); this.squealBp.type = 'bandpass'; this.squealBp.Q.value = 9; this.squealBp.frequency.value = 1150;
    const sqBp2 = ctx.createBiquadFilter(); sqBp2.type = 'bandpass'; sqBp2.Q.value = 6; sqBp2.frequency.value = 2300;
    this.squealGain = ctx.createGain(); this.squealGain.gain.value = 0;
    sq.connect(this.squealBp).connect(this.squealGain);
    sq.connect(sqBp2).connect(this.squealGain);
    this.squealGain.connect(this.master);

    // --- kerb rumble: low square wave gated by the kerb stripes ---
    this.kerb = ctx.createOscillator(); this.kerb.type = 'square';
    const kerbLp = ctx.createBiquadFilter(); kerbLp.type = 'lowpass'; kerbLp.frequency.value = 180;
    this.kerbGain = ctx.createGain(); this.kerbGain.gain.value = 0;
    this.kerb.connect(kerbLp).connect(this.kerbGain).connect(this.master); this.kerb.start();

    // --- grass / gravel: low rumbling noise ---
    const gr = this.noiseSource(noise);
    const grLp = ctx.createBiquadFilter(); grLp.type = 'lowpass'; grLp.frequency.value = 320;
    this.grassGain = ctx.createGain(); this.grassGain.gain.value = 0;
    gr.connect(grLp).connect(this.grassGain).connect(this.master);

    // --- wind ---
    const wi = this.noiseSource(noise);
    this.windBp = ctx.createBiquadFilter(); this.windBp.type = 'bandpass'; this.windBp.Q.value = 0.5; this.windBp.frequency.value = 700;
    this.windGain = ctx.createGain(); this.windGain.gain.value = 0;
    wi.connect(this.windBp).connect(this.windGain).connect(this.master);
  }

  // Called every frame with the simulation state
  update(sim, input, opts = {}) {
    if (!this.ctx || this.ctx.state !== 'running') return;
    const t = this.ctx.currentTime, tc = 0.03;
    const set = (param, v, k = tc) => param.setTargetAtTime(v, t, k);
    const rpm = sim.rpm, thr = input.throttle; // engine demand (also in reverse)
    const paused = !!opts.paused;
    const load = paused ? 0 : thr;

    // limiter: the ignition cut makes the revs bounce at ~18 Hz
    let rpmEff = rpm;
    if (sim.limiter && !paused) { this.limiterPhase += 1; rpmEff -= (this.limiterPhase % 3 === 0 ? 260 : 0); }
    const f0 = (rpmEff / 60) * 3;
    for (const o of this.osc) set(o.osc.frequency, f0 * o.mult, 0.012);
    set(this.roughBp.frequency, f0 * 2.2, 0.02);
    // filter opens with throttle and revs; off-throttle the sound is darker (overrun)
    set(this.engineFilter.frequency, 380 + rpm * 0.16 + load * (900 + rpm * 0.28));
    set(this.engineDrive.gain, 0.5 + load * 1.6);
    set(this.roughGain.gain, 0.08 + load * 0.25);
    let lvl = paused ? 0.05 : 0.11 + load * 0.2 + (rpm / 10000) * 0.08;
    // short dip in a gearshift
    if (sim.shiftEvent) { this.lastShift = t; sim.shiftEvent = 0; }
    if (t - this.lastShift < 0.06) lvl *= 0.35;
    if (sim.limiter && !paused && this.limiterPhase % 3 === 0) lvl *= 0.55;
    set(this.engineGain.gain, lvl, 0.015);

    // turbo spools with throttle and revs
    const boost = paused ? 0 : load * Math.max(0, (rpm - 4000) / 6000);
    set(this.turbo.frequency, 2600 + rpm * 0.45);
    set(this.turboGain.gain, boost * 0.012, 0.08);

    const v = paused ? 0 : sim.speed;
    // tyres: loudest sliding tyre, louder with load
    let squeal = 0;
    for (const w of sim.wheels) if (w.surface?.name !== 'grass') squeal = Math.max(squeal, w.sliding * Math.min(1, w.fz / 4000));
    set(this.squealGain.gain, paused ? 0 : squeal * 0.09, 0.04);
    set(this.squealBp.frequency, 1050 + 250 * squeal + Math.sin(t * 13) * 40);

    // kerbs and grass
    const onKerb = sim.wheels.some((w) => w.surface?.name === 'kerb');
    const onGrass = sim.wheels.filter((w) => w.surface?.name === 'grass').length;
    set(this.kerb.frequency, Math.max(8, v / 0.9));
    set(this.kerbGain.gain, onKerb && v > 2 ? Math.min(0.22, 0.05 + v * 0.004) : 0, 0.02);
    set(this.grassGain.gain, onGrass && v > 1 ? Math.min(0.35, (0.05 + v * 0.008) * onGrass / 2) : 0, 0.05);

    // wind
    set(this.windGain.gain, Math.min(0.12, (v / 90) ** 2 * 0.12), 0.1);
    set(this.windBp.frequency, 400 + v * 12, 0.1);
  }
}
