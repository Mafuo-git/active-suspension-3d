// Test circuit (pure geometry, no rendering). Units: metres.
// World plane: X east, Y north (left of a car driving along +X). The car starts on the main straight.

const CONTROL = [
  [-150, 0], [100, 0], [330, 0], [420, 12], [465, 60], [460, 130],
  [420, 175], [360, 185], [315, 215], [310, 265], [345, 300],
  [420, 320], [470, 360], [455, 420], [390, 440],
  [250, 440], [120, 445], [60, 425], [45, 385], [80, 350], [60, 315],
  [-20, 300], [-110, 330], [-200, 330], [-270, 280], [-290, 190], [-270, 100], [-220, 30],
];

export const TRACK_WIDTH = 12;

// Closed Catmull-Rom spline, resampled every `step` metres
function buildTrack(step = 2) {
  const N = CONTROL.length, dense = [];
  for (let i = 0; i < N; i++) {
    const p0 = CONTROL[(i - 1 + N) % N], p1 = CONTROL[i], p2 = CONTROL[(i + 1) % N], p3 = CONTROL[(i + 2) % N];
    for (let j = 0; j < 60; j++) {
      const t = j / 60, t2 = t * t, t3 = t2 * t;
      dense.push([0, 1].map((k) => 0.5 * (2 * p1[k] + (-p0[k] + p2[k]) * t + (2 * p0[k] - 5 * p1[k] + 4 * p2[k] - p3[k]) * t2 + (-p0[k] + 3 * p1[k] - 3 * p2[k] + p3[k]) * t3)));
    }
  }
  // arc-length resampling
  const cum = [0];
  for (let i = 1; i <= dense.length; i++) {
    const a = dense[i - 1], b = dense[i % dense.length];
    cum.push(cum[i - 1] + Math.hypot(b[0] - a[0], b[1] - a[1]));
  }
  const length = cum[cum.length - 1];
  const n = Math.round(length / step), pts = [];
  let k = 0;
  for (let i = 0; i < n; i++) {
    const s = (i / n) * length;
    while (cum[k + 1] < s) k++;
    const a = dense[k], b = dense[(k + 1) % dense.length], u = (s - cum[k]) / (cum[k + 1] - cum[k]);
    pts.push({ x: a[0] + (b[0] - a[0]) * u, y: a[1] + (b[1] - a[1]) * u, s });
  }
  for (let i = 0; i < n; i++) {
    const a = pts[(i - 1 + n) % n], b = pts[i], c = pts[(i + 1) % n];
    const tx = c.x - a.x, ty = c.y - a.y, tl = Math.hypot(tx, ty);
    b.tx = tx / tl; b.ty = ty / tl;
    b.nx = -b.ty; b.ny = b.tx; // left normal
    const cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    const ab = Math.hypot(b.x - a.x, b.y - a.y), bc = Math.hypot(c.x - b.x, c.y - b.y), ca = Math.hypot(a.x - c.x, a.y - c.y);
    b.k = (2 * cross) / (ab * bc * ca + 1e-9); // signed curvature, + = left turn
  }
  // smooth curvature for kerb placement
  for (let pass = 0; pass < 3; pass++) {
    const k0 = pts.map((p) => p.k);
    pts.forEach((p, i) => { p.k = (k0[(i - 2 + n) % n] + k0[(i - 1 + n) % n] + k0[i] + k0[(i + 1) % n] + k0[(i + 2) % n]) / 5; });
  }
  return { pts, length, width: TRACK_WIDTH };
}

export const TRACK = buildTrack();

// Nearest centre-line sample, searching around a previous index for speed
export function locate(x, y, hint = -1) {
  const { pts } = TRACK, n = pts.length;
  let best = 0, bestD = Infinity;
  const scan = (i) => { const p = pts[(i + n) % n], d = (p.x - x) ** 2 + (p.y - y) ** 2; if (d < bestD) { bestD = d; best = (i + n) % n; } };
  if (hint < 0) for (let i = 0; i < n; i++) scan(i);
  else for (let i = hint - 40; i <= hint + 40; i++) scan(i);
  const p = pts[best];
  const lateral = (x - p.x) * p.nx + (y - p.y) * p.ny; // + = left of the centre line
  return { index: best, s: p.s, lateral, point: p };
}

// Start pose on the main straight, facing the racing direction
export function startPose() {
  const p = TRACK.pts[0];
  return { x: p.x, y: p.y, psi: Math.atan2(p.ty, p.tx) };
}
