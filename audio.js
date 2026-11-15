// Car sound for the track test (Web Audio API, no sample files).
//
// Engine: a physical model of the V6 (firing pulses through the exhaust pipes) runs in an
// AudioWorklet, see engine-worklet.js. On top: tyre squeal from the sliding tyres, kerb rumble, grass
// noise and wind noise that grows with speed. Everything follows the simulation in real time.

const clamp01 = (x) => Math.max(0, Math.min(1, x || 0));

export class CarAudio {
  constructor() {
    this.ctx = null;
    this.enabled = true;
    this.volume = 0.55;
    this.engine = null;
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

    // --- engine: physical model in an AudioWorklet (loaded asynchronously) ---
    this.engineOut = ctx.createGain();
    this.engineOut.gain.value = 0.9;
    this.engineOut.connect(this.master);
    const noise = this.noiseBuffer();
    if (ctx.audioWorklet) {
      ctx.audioWorklet.addModule(new URL('./engine-worklet.js', import.meta.url)).then(() => {
        this.engine = new AudioWorkletNode(ctx, 'f2-engine', { numberOfInputs: 0, outputChannelCount: [1] });
        this.engine.connect(this.engineOut);
      }).catch(() => { /* no engine sound on this browser */ });
    }

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
    const paused = !!opts.paused;
    if (this.engine) {
      const P = this.engine.parameters;
      // no drive during a gearshift: the ignition is cut, the revs drop
      const load = paused || sim.shiftTimer > 0 ? 0 : clamp01(input.throttle);
      P.get('rpm').setTargetAtTime(paused ? 3500 : sim.rpm, t, 0.01);
      P.get('load').setTargetAtTime(load, t, 0.02);
      P.get('limiter').setValueAtTime(!paused && sim.limiter ? 1 : 0, t);
      P.get('level').setTargetAtTime(paused ? 0.25 : 1, t, 0.1);
    }

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
