// Bodywork of a 1/4-scale F2-style car, built from lofted cross-sections.
// Coordinates: ISO chassis frame in mm (x forward, y left, z up), front axle at x = 0.

import * as THREE from 'three';

// Rounded cross-section (superellipse) centred on (yc, (zb+zt)/2)
function ring(s, n, e) {
  const pts = [];
  const zm = (s.zb + s.zt) / 2, hh = (s.zt - s.zb) / 2, ex = 2 / (s.e ?? e);
  for (let i = 0; i < n; i++) {
    const t = (i / n) * Math.PI * 2, c = Math.cos(t), si = Math.sin(t);
    pts.push(new THREE.Vector3(s.x, (s.yc ?? 0) + s.w * Math.sign(c) * Math.abs(c) ** ex, zm + hh * Math.sign(si) * Math.abs(si) ** ex));
  }
  return pts;
}

// Smooth surface through the sections, closed at both ends
export function loft(sections, n = 32, e = 3.2) {
  const rings = sections.map((s) => ring(s, n, e));
  const pos = [], idx = [];
  rings.forEach((r) => r.forEach((p) => pos.push(p.x, p.y, p.z)));
  for (let k = 0; k < rings.length - 1; k++) {
    for (let i = 0; i < n; i++) {
      const a = k * n + i, b = k * n + ((i + 1) % n), c = (k + 1) * n + i, d = (k + 1) * n + ((i + 1) % n);
      idx.push(a, c, b, b, c, d);
    }
  }
  // end caps
  for (const [k, flip] of [[0, true], [rings.length - 1, false]]) {
    const ctr = rings[k].reduce((acc, p) => acc.add(p), new THREE.Vector3()).multiplyScalar(1 / n);
    const ci = pos.length / 3;
    pos.push(ctr.x, ctr.y, ctr.z);
    for (let i = 0; i < n; i++) {
      const a = k * n + i, b = k * n + ((i + 1) % n);
      flip ? idx.push(ci, a, b) : idx.push(ci, b, a);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

// Wing element: airfoil drawn in the x-z plane, extruded along y (span)
function wing(chord, thick, span, camber = 0.08) {
  const N = 24, up = [], lo = [];
  for (let i = 0; i <= N; i++) {
    const t = (1 - Math.cos((i / N) * Math.PI)) / 2; // cosine spacing
    const yt = 5 * thick * chord * (0.2969 * Math.sqrt(t) - 0.126 * t - 0.3516 * t * t + 0.2843 * t ** 3 - 0.1036 * t ** 4);
    const yc = camber * chord * 4 * t * (1 - t);
    up.push([-chord * t, yc + yt]);
    lo.push([-chord * t, yc - yt]);
  }
  // leading edge at x = 0, trailing edge at x = -chord (pointing rearward)
  const s = new THREE.Shape();
  up.forEach(([x, y], i) => (i ? s.lineTo(x, y) : s.moveTo(x, y)));
  lo.slice(1, -1).reverse().forEach(([x, y]) => s.lineTo(x, y));
  s.closePath();
  const g = new THREE.ExtrudeGeometry(s, { depth: span, bevelEnabled: false, curveSegments: 8 });
  g.rotateX(Math.PI / 2);          // extrusion (local z) -> -y, profile y -> z
  g.translate(0, span / 2, 0);     // centre the span on y = 0
  return g;
}

function plateXZ(points, thickness) {
  const s = new THREE.Shape();
  points.forEach(([x, z], i) => (i ? s.lineTo(x, z) : s.moveTo(x, z)));
  s.closePath();
  const g = new THREE.ExtrudeGeometry(s, { depth: thickness, bevelEnabled: false });
  g.rotateX(Math.PI / 2);
  g.translate(0, thickness / 2, 0);
  return g;
}

export function buildBody(M, wheelbase = 784) {
  const body = new THREE.Group();   // painted shell (can be ghosted or hidden)
  const inner = new THREE.Group();  // driver, engine, gearbox (always drawn)
  const add = (geo, mat, parent = body) => { const m = new THREE.Mesh(geo, mat); m.castShadow = m.receiveShadow = true; parent.add(m); return m; };
  const xr = -wheelbase; // rear axle

  // Monocoque and nose
  add(loft([
    { x: 255, w: 10, zb: 62, zt: 76 },
    { x: 245, w: 18, zb: 56, zt: 86 },
    { x: 180, w: 28, zb: 50, zt: 104 },
    { x: 90, w: 42, zb: 40, zt: 132 },
    { x: 20, w: 48, zb: 32, zt: 148 },
    { x: -80, w: 56, zb: 26, zt: 156 },
    { x: -170, w: 76, zb: 22, zt: 158 },
    { x: -330, w: 92, zb: 20, zt: 152 },
    { x: -345, w: 90, zb: 20, zt: 150 },
  ]), M.body);

  // Engine cover with airbox, tapering to the gearbox
  add(loft([
    { x: -320, w: 88, zb: 22, zt: 160 },
    { x: -360, w: 70, zb: 24, zt: 250, e: 2.6 },
    { x: -400, w: 62, zb: 26, zt: 262, e: 2.6 },
    { x: -470, w: 54, zb: 28, zt: 214 },
    { x: -600, w: 42, zb: 30, zt: 150 },
    { x: xr + 60, w: 30, zb: 34, zt: 112 },
    { x: xr - 40, w: 24, zb: 38, zt: 96 },
    { x: xr - 90, w: 16, zb: 46, zt: 84 },
  ]), M.body);
  // Airbox intake
  add(loft([{ x: -352, w: 26, zb: 214, zt: 250, e: 2.2 }, { x: -360, w: 24, zb: 216, zt: 248, e: 2.2 }], 24), M.dark);

  // Sidepods (both sides)
  for (const s of [1, -1]) {
    add(loft([
      { x: -175, yc: s * 140, w: 34, zb: 24, zt: 98 },
      { x: -200, yc: s * 146, w: 46, zb: 22, zt: 108 },
      { x: -300, yc: s * 146, w: 50, zb: 22, zt: 112 },
      { x: -460, yc: s * 128, w: 46, zb: 22, zt: 96 },
      { x: -600, yc: s * 98, w: 28, zb: 24, zt: 64 },
      { x: -650, yc: s * 84, w: 16, zb: 26, zt: 46 },
    ]), M.body);
    // radiator inlet
    add(loft([{ x: -176, yc: s * 140, w: 28, zb: 34, zt: 92 }, { x: -178, yc: s * 140, w: 27, zb: 35, zt: 91 }], 24), M.dark);
  }

  // Cockpit opening and headrest
  add(loft([
    { x: -175, w: 30, zb: 150, zt: 162 },
    { x: -200, w: 44, zb: 148, zt: 164 },
    { x: -300, w: 46, zb: 146, zt: 164 },
    { x: -318, w: 30, zb: 146, zt: 164 },
  ], 32, 2.4), M.cockpit);

  // Floor with front plank and rear taper
  {
    const s = new THREE.Shape();
    const pts = [[-130, 60], [-150, 200], [-560, 205], [-660, 120], [xr + 10, 110], [xr - 20, 60]];
    pts.forEach(([x, y], i) => (i ? s.lineTo(x, y) : s.moveTo(x, y)));
    pts.slice().reverse().forEach(([x, y]) => s.lineTo(x, -y));
    s.closePath();
    const g = new THREE.ExtrudeGeometry(s, { depth: 4, bevelEnabled: false });
    g.translate(0, 0, 16);
    add(g, M.carbon);
  }
  // Diffuser
  add(plateXZ([[xr + 10, 20], [xr - 110, 20], [xr - 110, 62], [xr + 10, 22]], 220), M.carbon);

  // Front wing: main plane, flap, endplates, pylons
  add(wing(52, 0.14, 470, -0.06), M.carbon).position.set(290, 0, 22);
  { const f = add(wing(32, 0.12, 440, -0.1), M.accent); f.position.set(244, 0, 40); f.rotation.y = 0.22; }
  for (const s of [1, -1]) {
    add(plateXZ([[296, 14], [206, 14], [206, 70], [250, 70], [296, 40]], 3), M.carbon).position.y = s * 236;
    add(new THREE.BoxGeometry(46, 5, 30), M.carbon).position.set(262, s * 22, 42);
  }

  // Rear wing: main plane, flap, endplates, pylon
  const rx = xr - 205;
  add(wing(60, 0.12, 360, -0.08), M.carbon).position.set(rx + 30, 0, 214);
  { const f = add(wing(36, 0.1, 352, -0.12), M.accent); f.position.set(rx - 34, 0, 238); f.rotation.y = 0.35; }
  for (const s of [1, -1]) add(plateXZ([[rx + 40, 140], [rx - 90, 140], [rx - 90, 268], [rx + 26, 268]], 3), M.carbon).position.y = s * 182;
  add(plateXZ([[xr - 70, 60], [rx - 10, 160], [rx - 30, 214], [rx + 10, 214], [xr - 40, 80]], 8), M.carbon);

  // Halo (hoop and centre pillar)
  const halo = new THREE.CatmullRomCurve3([
    new THREE.Vector3(-318, 70, 158), new THREE.Vector3(-290, 72, 196), new THREE.Vector3(-232, 52, 214),
    new THREE.Vector3(-196, 0, 214), new THREE.Vector3(-232, -52, 214), new THREE.Vector3(-290, -72, 196),
    new THREE.Vector3(-318, -70, 158),
  ]);
  add(new THREE.TubeGeometry(halo, 64, 5, 12, false), M.halo);
  add(new THREE.TubeGeometry(new THREE.CatmullRomCurve3([
    new THREE.Vector3(-197, 0, 212), new THREE.Vector3(-186, 0, 190), new THREE.Vector3(-172, 0, 160),
  ]), 16, 5, 12, false), M.halo);

  // Inner parts: driver helmet, engine block, gearbox casing
  add(new THREE.SphereGeometry(27, 24, 18), M.helmet, inner).position.set(-268, 0, 178);
  add(new THREE.BoxGeometry(250, 84, 80), M.engine, inner).position.set(-560, 0, 74);
  add(new THREE.BoxGeometry(170, 50, 64), M.alu, inner).position.set(xr - 10, 0, 76);

  return { body, inner };
}
