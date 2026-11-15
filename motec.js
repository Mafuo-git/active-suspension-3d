// MoTeC i2 export for the track test.
//
// Writes a MoTeC .ld log (binary, little-endian) and its .ldx companion (XML) holding the lap beacons,
// so a session driven in the browser opens in MoTeC i2 Pro like a logger download. The .ld layout
// follows the community documentation of the format (header, event block, a linked list of channel
// descriptors, then the channel data as float32 arrays).

const HEAD_SIZE = 1762, EVENT_SIZE = 1154, VENUE_SIZE = 1100, VEHICLE_SIZE = 260, CHAN_SIZE = 124;
// i2 Pro only opens "pro logging" files. The pro-logging key in the header goes with the logger's
// serial number: this pair is the one found in the logs written by the usual sim-racing exporters.
const DEVICE_SERIAL = 0x1f44, PRO_LOGGING_KEY = 0xc81a4;

export class TelemetryLog {
  // channels: [{ name, short, unit, get: () => number }]
  constructor(channels, rate = 50, maxMinutes = 60) {
    this.channels = channels;
    this.rate = rate;
    this.max = Math.round(maxMinutes * 60 * rate);
    this.clear();
  }
  clear() {
    this.n = 0;
    this.cap = 0;
    this.data = this.channels.map(() => new Float32Array(0));
    this.beacons = [];   // session times (s) of the start / finish line crossings
    this.laps = [];      // { t, valid }
    this.started = new Date();
    this.full = false;
  }
  get duration() { return this.n / this.rate; }
  record() {
    if (this.n >= this.max) { this.full = true; return false; }
    if (this.n >= this.cap) {
      this.cap = Math.max(1024, this.cap * 2);
      this.data = this.data.map((a) => { const b = new Float32Array(this.cap); b.set(a); return b; });
    }
    for (let c = 0; c < this.channels.length; c++) {
      const v = this.channels[c].get();
      this.data[c][this.n] = Number.isFinite(v) ? v : 0;
    }
    this.n++;
    return true;
  }
  beacon() { this.beacons.push(this.duration); }
  lap(t, valid) { this.laps.push({ t, valid }); }

  // ---------------------------------------------------------------- .ld
  buildLd(meta = {}) {
    const n = this.channels.length, len = this.n;
    const eventPtr = HEAD_SIZE, venuePtr = eventPtr + EVENT_SIZE, vehiclePtr = venuePtr + VENUE_SIZE;
    const metaPtr = vehiclePtr + VEHICLE_SIZE, dataPtr = metaPtr + n * CHAN_SIZE;
    const total = dataPtr + n * len * 4;
    const buf = new ArrayBuffer(total), dv = new DataView(buf), u8 = new Uint8Array(buf);
    const str = (off, s, size) => { const b = asciiBytes(s, size); u8.set(b, off); };
    const d = this.started;
    const pad = (x) => String(x).padStart(2, '0');
    // header
    dv.setUint32(0, 0x40, true);
    dv.setUint32(8, metaPtr, true);
    dv.setUint32(12, dataPtr, true);
    dv.setUint32(36, eventPtr, true);
    dv.setUint16(64, 1, true); dv.setUint16(66, 0x4240, true); dv.setUint16(68, 0xf, true);
    dv.setUint32(70, DEVICE_SERIAL, true);   // device serial
    str(74, 'ADL', 8);                         // device type
    dv.setUint16(82, 420, true);               // device version
    dv.setUint16(84, 0xadb0, true);
    dv.setUint32(86, n, true);                 // number of channels
    str(94, `${pad(d.getDate())}/${pad(d.getMonth() + 1)}/${d.getFullYear()}`, 16);
    str(126, `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`, 16);
    str(158, meta.driver || '', 64);
    str(222, meta.vehicle || '', 64);
    str(350, meta.venue || '', 64);
    dv.setUint32(1502, PRO_LOGGING_KEY, true); // "pro logging" key (matches the serial above)
    str(1572, meta.shortComment || '', 64);
    // event
    str(eventPtr, meta.event || '', 64);
    str(eventPtr + 64, meta.session || '', 64);
    str(eventPtr + 128, meta.comment || '', 1024);
    dv.setUint16(eventPtr + 1152, venuePtr, true);
    // venue and vehicle blocks, as in a logger download
    str(venuePtr, meta.venue || '', 64);
    dv.setUint16(venuePtr + 1098, vehiclePtr, true);
    str(vehiclePtr, meta.vehicle || '', 64);
    dv.setUint32(vehiclePtr + 192, Math.round(meta.weight || 0), true);
    str(vehiclePtr + 196, meta.vehicleType || '', 32);
    str(vehiclePtr + 228, meta.vehicleComment || '', 32);
    // channel descriptors (doubly linked list) and data
    for (let c = 0; c < n; c++) {
      const ch = this.channels[c], at = metaPtr + c * CHAN_SIZE, dp = dataPtr + c * len * 4;
      dv.setUint32(at, c === 0 ? 0 : at - CHAN_SIZE, true);
      dv.setUint32(at + 4, c === n - 1 ? 0 : at + CHAN_SIZE, true);
      dv.setUint32(at + 8, dp, true);
      dv.setUint32(at + 12, len, true);
      dv.setUint16(at + 16, 0x2ee1 + c, true);
      dv.setUint16(at + 18, 0x07, true);       // float
      dv.setUint16(at + 20, 4, true);          // 4 bytes
      dv.setUint16(at + 22, this.rate, true);  // Hz
      dv.setInt16(at + 24, 0, true);           // shift
      dv.setInt16(at + 26, 1, true);           // mul
      dv.setInt16(at + 28, 1, true);           // scale
      dv.setInt16(at + 30, 0, true);           // decimal places (must stay 0: it scales the data)
      str(at + 32, ch.name, 32);
      str(at + 64, ch.short || ch.name.slice(0, 8), 8);
      str(at + 72, ch.unit || '', 12);
      const arr = this.data[c];
      for (let i = 0; i < len; i++) dv.setFloat32(dp + i * 4, arr[i], true);
    }
    return buf;
  }

  // ---------------------------------------------------------------- .ldx (laps)
  buildLdx() {
    const markers = this.beacons.map((t, i) =>
      `      <Marker Version="100" ClassName="BCN" Name="Manual.${i + 1}" Flags="77" Time="${(t * 1e6).toFixed(2)}"/>`).join('\n');
    const valid = this.laps.filter((l) => l.valid);
    let fastest = null, fastestN = 0;
    this.laps.forEach((l, i) => { if (l.valid && (fastest == null || l.t < fastest)) { fastest = l.t; fastestN = i + 2; } }); // lap 1 = out lap
    const fmtT = (t) => `${Math.floor(t / 60)}:${(t % 60).toFixed(3).padStart(6, '0')}`;
    return `<?xml version="1.0"?>
<LDXFile Locale="English_United Kingdom.1252" DefaultLocale="C" Version="1.6">
 <Layers>
  <Layer>
   <MarkerBlock>
    <MarkerGroup Name="Beacons" Index="3">
${markers}
    </MarkerGroup>
   </MarkerBlock>
   <RangeBlock/>
  </Layer>
  <Details>
   <String Id="Total Laps" Value="${this.beacons.length + 1}"/>
${fastest != null ? `   <String Id="Fastest Time" Value="${fmtT(fastest)}"/>\n   <String Id="Fastest Lap" Value="${fastestN}"/>\n` : ''}   <String Id="Valid Laps" Value="${valid.length}"/>
  </Details>
 </Layers>
</LDXFile>
`;
  }
}

function asciiBytes(s, size) {
  const out = new Uint8Array(size);
  const clean = String(s).normalize('NFD').replace(/[^\x20-\x7e]/g, '');
  for (let i = 0; i < Math.min(clean.length, size - 1); i++) out[i] = clean.charCodeAt(i);
  return out;
}

export function download(name, data, type) {
  const blob = data instanceof Blob ? data : new Blob([data], { type });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  document.body.appendChild(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1500);
}
