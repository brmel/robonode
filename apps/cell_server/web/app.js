import * as THREE from './three.module.min.js';

// Drag to orbit, wheel to zoom, right/shift-drag to pan. A tiny self-contained
// orbit camera so the scene is yours to inspect — no external controls module.
class Orbit {
  constructor(camera, dom, target, radius) {
    this.cam = camera; this.target = target.clone();
    this.r = radius; this.theta = 0.7; this.phi = 1.05; this.drag = null;
    dom.addEventListener('contextmenu', e => e.preventDefault());
    dom.addEventListener('pointerdown', e => { this.drag = { x: e.clientX, y: e.clientY, pan: e.button === 2 || e.shiftKey }; });
    addEventListener('pointerup', () => { this.drag = null; });
    addEventListener('pointermove', e => this.move(e));
    dom.addEventListener('wheel', e => { e.preventDefault(); this.r = THREE.MathUtils.clamp(this.r * (1 + Math.sign(e.deltaY) * 0.1), 0.6, 12); }, { passive: false });
    this.update();
  }
  move(e) {
    if (!this.drag) return;
    const dx = e.clientX - this.drag.x, dy = e.clientY - this.drag.y;
    this.drag.x = e.clientX; this.drag.y = e.clientY;
    if (this.drag.pan) {
      const s = this.r * 0.0016, right = new THREE.Vector3().setFromMatrixColumn(this.cam.matrix, 0);
      this.target.addScaledVector(right, -dx * s).addScaledVector(new THREE.Vector3().setFromMatrixColumn(this.cam.matrix, 1), dy * s);
    } else {
      this.theta -= dx * 0.006;
      this.phi = THREE.MathUtils.clamp(this.phi - dy * 0.006, 0.15, Math.PI / 2 - 0.02);
    }
    this.update();
  }
  update() {
    const sp = Math.sin(this.phi);
    this.cam.position.set(this.target.x + this.r * sp * Math.sin(this.theta), this.target.y + this.r * Math.cos(this.phi), this.target.z + this.r * sp * Math.cos(this.theta));
    this.cam.lookAt(this.target);
  }
}

// ---- scene ---------------------------------------------------------------
const view = document.getElementById('view');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e1116);

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(devicePixelRatio);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
view.appendChild(renderer.domElement);

const orbit = new Orbit(camera, renderer.domElement, new THREE.Vector3(0.6, 0.5, -0.2), 2.6);

scene.add(new THREE.HemisphereLight(0xbcd0ff, 0x0c0e12, 0.85));
const key = new THREE.DirectionalLight(0xffffff, 1.6);
key.position.set(2.5, 4.5, 3);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
Object.assign(key.shadow.camera, { left: -3, right: 3, top: 3, bottom: -3, near: 0.5, far: 14 });
scene.add(key);
scene.add(new THREE.DirectionalLight(0x4c6a99, 0.4).translateX(-3));

const floor = new THREE.Mesh(new THREE.PlaneGeometry(12, 12),
  new THREE.MeshStandardMaterial({ color: 0x14181f, metalness: 0.1, roughness: 0.95 }));
floor.rotation.x = -Math.PI / 2;
floor.receiveShadow = true;
scene.add(floor);
const grid = new THREE.GridHelper(12, 48, 0x2b3644, 0x1c232d);
scene.add(grid);

// MuJoCo is Z-up, the viewer Y-up: everything physical lives under this group so
// the menagerie offsets (Z-up, metres) render upright and sit on the floor.
const zup = new THREE.Group();
zup.rotation.x = -Math.PI / 2;
scene.add(zup);

const railMesh = new THREE.Mesh(new THREE.BoxGeometry(1.65, 0.26, 0.05),
  new THREE.MeshStandardMaterial({ color: 0x2a3340, metalness: 0.4, roughness: 0.6 }));
railMesh.position.set(0.5, 0, 0.03);
railMesh.receiveShadow = true;
zup.add(railMesh);

const carriage = new THREE.Group();
carriage.position.z = 0.1;
zup.add(carriage);
const carBox = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.22, 0.12),
  new THREE.MeshStandardMaterial({ color: 0xcf8a3a, metalness: 0.3, roughness: 0.6 }));
carBox.castShadow = true;
carriage.add(carBox);

const MAT = {
  black: new THREE.MeshStandardMaterial({ color: 0x0b0b0b, metalness: 0.4, roughness: 0.5 }),
  jointgray: new THREE.MeshStandardMaterial({ color: 0x474747, metalness: 0.5, roughness: 0.45 }),
  linkgray: new THREE.MeshStandardMaterial({ color: 0xd1d1d1, metalness: 0.3, roughness: 0.5 }),
  urblue: new THREE.MeshStandardMaterial({ color: 0x7dadcc, metalness: 0.25, roughness: 0.45 }),
};

// Minimal OBJ → BufferGeometry (v / vn / f, triangulated) — no loader addon.
async function loadOBJ(name, material, parent) {
  try {
    const txt = await (await fetch(`./assets/${name}.obj`)).text();
    const V = [], N = [], P = [], No = [];
    for (const ln of txt.split('\n')) {
      const t = ln.split(/\s+/);
      if (t[0] === 'v') V.push([+t[1], +t[2], +t[3]]);
      else if (t[0] === 'vn') N.push([+t[1], +t[2], +t[3]]);
      else if (t[0] === 'f') {
        const f = t.slice(1).map(s => s.split('/').map(x => (x ? +x - 1 : -1)));
        for (let i = 1; i < f.length - 1; i++) for (const v of [f[0], f[i], f[i + 1]]) {
          P.push(...V[v[0]]);
          if (v[2] >= 0 && N[v[2]]) No.push(...N[v[2]]);
        }
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(P, 3));
    if (No.length === P.length) g.setAttribute('normal', new THREE.Float32BufferAttribute(No, 3));
    else g.computeVertexNormals();
    const m = new THREE.Mesh(g, material);
    m.castShadow = true;
    parent.add(m);
  } catch { /* mesh missing → hierarchy still poses */ }
}

// UR10e chain, verbatim from the menagerie model: body offset (pos), fixed
// orientation (quat [w,x,y,z]), joint axis, and the meshes on that link.
const UR = [
  { pos: [0, 0, 0], quat: [0, 0, 0, -1], meshes: [['base_0', 'black'], ['base_1', 'jointgray']] },
  { pos: [0, 0, 0.181], axis: [0, 0, 1], meshes: [['shoulder_0', 'urblue'], ['shoulder_1', 'black'], ['shoulder_2', 'jointgray']] },
  { pos: [0, 0.176, 0], quat: [1, 0, 1, 0], axis: [0, 1, 0], meshes: [['upperarm_0', 'black'], ['upperarm_1', 'jointgray'], ['upperarm_2', 'urblue'], ['upperarm_3', 'linkgray']] },
  { pos: [0, -0.137, 0.613], axis: [0, 1, 0], meshes: [['forearm_0', 'urblue'], ['forearm_1', 'black'], ['forearm_2', 'jointgray'], ['forearm_3', 'linkgray']] },
  { pos: [0, 0, 0.571], quat: [1, 0, 1, 0], axis: [0, 1, 0], meshes: [['wrist1_0', 'black'], ['wrist1_1', 'urblue'], ['wrist1_2', 'jointgray']] },
  { pos: [0, 0.135, 0], axis: [0, 0, 1], meshes: [['wrist2_0', 'black'], ['wrist2_1', 'urblue'], ['wrist2_2', 'jointgray']] },
  { pos: [0, 0, 0.12], axis: [0, 1, 0], meshes: [['wrist3', 'linkgray']] },
];
const joints = [];  // {group, axis} for j1..j6, in order
let parent = new THREE.Group();
parent.position.set(0, 0, 0.06);  // UR base on the carriage
carriage.add(parent);
for (const b of UR) {
  const fixed = new THREE.Group();
  fixed.position.set(...b.pos);
  if (b.quat) { const [w, x, y, z] = b.quat; fixed.quaternion.set(x, y, z, w).normalize(); }
  parent.add(fixed);
  const joint = new THREE.Group();
  fixed.add(joint);
  for (const [name, mat] of b.meshes) loadOBJ(name, MAT[mat], joint);
  if (b.axis) joints.push({ group: joint, axis: new THREE.Vector3(...b.axis) });
  parent = joint;
}
const tcpMark = new THREE.Mesh(new THREE.SphereGeometry(0.012, 12, 12),
  new THREE.MeshStandardMaterial({ color: 0x39d98a, emissive: 0x145036 }));
tcpMark.position.set(0, 0.1, 0);
parent.add(tcpMark);

const partMesh = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, 0.06),
  new THREE.MeshStandardMaterial({ color: 0xe23b3b, emissive: 0x330606 }));
partMesh.castShadow = true;
zup.add(partMesh);

// ---- live pose (lerp toward latest telemetry) ----------------------------
let target = [0, 0, 0, 0, 0, 0, 0];   // [rail_mm, j1..j6 rad]
let shown = [0, 0, 0, 0, 0, 0, 0];
let partPose = null;                  // vision target (MuJoCo x,y,z), live

function applyPose() {
  for (let i = 0; i < 7; i++) shown[i] += (target[i] - shown[i]) * 0.25;
  carriage.position.x = shown[0] / 1000;      // mm → m (MuJoCo X)
  for (let i = 0; i < 6; i++) {
    joints[i].group.quaternion.setFromAxisAngle(joints[i].axis, shown[i + 1]);
  }
  if (partPose) partMesh.position.set(partPose[0], partPose[1], partPose[2]);
}

function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);
resize();

function loop() {
  applyPose();
  renderer.render(scene, camera);
  requestAnimationFrame(loop);
}
loop();

// ---- data: SSE + commands ------------------------------------------------
const nodesBody = document.getElementById('nodes');
const statusEl = document.getElementById('status');
const famBadge = document.getElementById('fambadge');
let nodeMeta = [];
let available = [];
let posCells = [];
let errVals = [];
let liveIn = [];        // governed setpoint (in), per node
let selected = 0;       // inspected node index
const inspectorEl = document.getElementById('inspector');
const versionsEl = document.getElementById('versions');
const physFollowEl = document.getElementById('physFollow');

// Version manager (#47): every registered version of the selected node as a
// chip — active highlighted, click to swap live, plus a bring-your-own slot.
function renderVersions() {
  if (!versionsEl) return;
  const n = nodeMeta[selected];
  if (!n) { versionsEl.innerHTML = ''; return; }
  const chips = available.map(v =>
    `<span class="chip${v === n.driver ? ' on' : ''}" data-v="${v}">${v.replace('robonode.', '')}</span>`);
  chips.push('<span class="chip byo" title="Bring your own: copy motion/byo_axis.hpp (#23)">＋ your own</span>');
  versionsEl.innerHTML = chips.join('');
  versionsEl.querySelectorAll('.chip[data-v]').forEach(c => {
    c.onclick = () => cmd({ cmd: 'set_driver', node: n.id, driver: c.dataset.v });
  });
}

// Dashboard Physics card: peak |following error| across the joints (rad) — a
// live read of how hard the physics is working vs the commanded setpoints.
function updateDash() {
  if (!physFollowEl || !errVals.length) return;
  let peak = 0;
  for (let i = 1; i < errVals.length; i++) peak = Math.max(peak, Math.abs(errVals[i] ?? 0));
  physFollowEl.textContent = peak.toFixed(3) + ' rad';
}

// Rebuilt on a 'nodes' event (initial + after any swap): each node gets a
// dropdown of the driver versions it can be swapped to. Only the position
// cells update on telemetry, so the dropdowns stay stable.
function renderNodes(tree) {
  nodeMeta = tree.nodes;
  available = tree.available || [];
  famBadge.textContent = tree.family;
  famBadge.className = 'badge ' + tree.family;
  const robotDrv = document.getElementById('robotDrv');
  if (robotDrv) robotDrv.textContent = tree.family;
  for (const b of document.querySelectorAll('#family button'))
    b.classList.toggle('on', b.dataset.fam === tree.family);

  nodesBody.innerHTML = '';
  posCells = [];
  nodeMeta.forEach((n, i) => {
    const tr = document.createElement('tr');
    const tdId = document.createElement('td'); tdId.textContent = n.id;
    const tdDrv = document.createElement('td');
    const sel = document.createElement('select');
    for (const d of available) {
      const o = document.createElement('option');
      o.value = d; o.textContent = d.replace('robonode.', '');
      if (d === n.driver) o.selected = true;
      sel.appendChild(o);
    }
    sel.onchange = () => cmd({ cmd: 'set_driver', node: n.id, driver: sel.value });
    tdDrv.appendChild(sel);
    const tdPos = document.createElement('td'); tdPos.className = 'val';
    tr.append(tdId, tdDrv, tdPos);
    tr.onclick = ev => { if (ev.target.tagName !== 'SELECT') selectNode(i); };
    nodesBody.appendChild(tr);
    posCells.push(tdPos);
  });
  if (selected >= nodeMeta.length) selected = 0;
  markSelected();
  updatePositions();
  renderInspector();
}

function selectNode(i) { selected = i; markSelected(); renderInspector(); }

function markSelected() {
  [...nodesBody.children].forEach((tr, i) => tr.classList.toggle('sel', i === selected));
}

// The clean I/O contract, legible: what the selected node declares (capability,
// limits) and its live in/out — setpoint, actual, following error.
function renderInspector() {
  const n = nodeMeta[selected];
  if (!n) { inspectorEl.innerHTML = '<span class="drv">select a node…</span>'; return; }
  const unit = ' ' + n.unit;
  const fmt = v => v == null ? '—' : (n.unit === 'mm' ? v.toFixed(0) : v.toFixed(3)) + unit;
  const rows = [
    ['capability', 'MotionAxis@1'],
    ['driver', n.driver.replace('robonode.', '')],
    ['limits', `${fmt(n.lo)} … ${fmt(n.hi)}`],
    ['setpoint · in', fmt(liveIn[selected])],
    ['actual · out', fmt(target[selected])],
    ['following err', fmt(errVals[selected] == null ? null : Math.abs(errVals[selected]))],
    ['state', n.state],
  ];
  inspectorEl.innerHTML =
    `<div class="kv"><span class="k">node</span><span class="v">${n.id}</span></div>` +
    rows.map(([k, v]) => `<div class="kv"><span class="k">${k}</span><span class="v">${v}</span></div>`).join('');
  renderVersions();
}
function updatePositions() {
  nodeMeta.forEach((n, i) => {
    if (!posCells[i]) return;
    const p = target[i] ?? 0;                       // actual (out)
    const e = Math.abs(errVals[i] ?? 0);            // |following error|
    const mm = n.unit === 'mm';
    const val = mm ? p.toFixed(0) + ' mm' : p.toFixed(3) + ' rad';
    // clean I/O made visible: actual + the gap to the commanded setpoint.
    const gap = e > (mm ? 0.5 : 0.002)
      ? `<span class="gap">Δ${mm ? e.toFixed(0) : e.toFixed(3)}</span>` : '';
    posCells[i].innerHTML = `${val} ${gap}`;
  });
}

const logsEl = document.getElementById('logs');
function renderLogs(lines) {
  if (!logsEl) return;
  logsEl.innerHTML = lines.map(l => {
    const cls = /error/i.test(l) ? ' class="err"' : /warn/i.test(l) ? ' class="warn"' : '';
    return `<div${cls}>${l.replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]))}</div>`;
  }).join('');
  logsEl.scrollTop = logsEl.scrollHeight;
}

const es = new EventSource('/events');
es.addEventListener('nodes', e => { statusEl.textContent = 'live'; renderNodes(JSON.parse(e.data)); });
es.addEventListener('logs', e => renderLogs(JSON.parse(e.data)));
es.onmessage = e => {
  const f = JSON.parse(e.data);
  if (f.pos) {
    target = f.pos; errVals = f.err || []; liveIn = f.target || [];
    updatePositions();
    renderInspector();
    updateDash();
  }
  if (f.tcp) setTargetDefault(f.tcp);
  if (f.vision?.part) {
    partPose = f.vision.part;
    if (visionPartEl) visionPartEl.textContent = partPose.map(v => v.toFixed(2)).join(', ');
  }
};
es.onerror = () => { statusEl.textContent = 'reconnecting…'; };

async function cmd(body) {
  await fetch('/command', { method: 'POST', body: JSON.stringify(body) });
}
document.getElementById('run').onclick = () => cmd({ cmd: 'run' });

// Cartesian move (#22): the target inputs default to the current TCP (so you
// nudge from where the tool is); Move TCP asks for real IK to a straight moveL.
const txEl = document.getElementById('tx'), tyEl = document.getElementById('ty'), tzEl = document.getElementById('tz');
let tcpInit = false;
function setTargetDefault(tcp) {
  if (tcpInit || !txEl || !tcp) return;
  txEl.value = tcp[0].toFixed(2); tyEl.value = tcp[1].toFixed(2); tzEl.value = tcp[2].toFixed(2);
  tcpInit = true;
}
const movelBtn = document.getElementById('movel');
if (movelBtn) movelBtn.onclick = () =>
  cmd({ cmd: 'move_l', x: parseFloat(txEl.value), y: parseFloat(tyEl.value), z: parseFloat(tzEl.value) });

// Vision (#6): the toy detector reports a part's pose; Pick moves the TCP to it.
const visionPartEl = document.getElementById('visionPart');
const pickBtn = document.getElementById('pick');
if (pickBtn) pickBtn.onclick = () =>
  partPose && cmd({ cmd: 'move_l', x: partPose[0], y: partPose[1], z: partPose[2] });

// Application library (#57/#59): saved apps come from the store (GET /apps),
// so the catalog is data-driven, not hardcoded. Deploying an app runs its
// program on the worker (#61/#64) — bin picking is a real app now (vision →
// pick → place). Palletizing / machine tending arrive next (#62/#63).
async function loadApps() {
  const el = document.getElementById('apps');
  if (!el) return;
  let saved = [];
  try { saved = await (await fetch('/apps')).json(); } catch { /* offline */ }
  const cards = saved.map(a =>
    `<div class="ncard app" data-file="${a.file}"><h3>🎯 ${a.name}</h3><div class="st">ready · click to run</div></div>`);
  el.innerHTML = cards.join('');
  el.querySelectorAll('.ncard.app').forEach(c => c.onclick = () => cmd({ cmd: 'run_app', file: c.dataset.file }));
}
loadApps();

// Stations (#31): conveyor / deck / pallet capability modules, from /stations.
async function loadStations() {
  const el = document.getElementById('stationCard');
  if (!el) return;
  try {
    const st = await (await fetch('/stations')).json();
    if (!st.length) return;
    const s = st[0];
    const detail = Object.entries(s.config || {}).map(([k, v]) => `${k} ${v}`).join(' · ');
    el.innerHTML = `<h3>📦 Station</h3><div class="st"><span style="color:var(--ok)">● ${s.type}</span> · ${s.id}${detail ? ' · ' + detail : ''}</div>`;
  } catch { /* offline */ }
}
loadStations();

// Vision capability (ADR-11): show the chosen algorithm version + chips to swap
// it live. The same shape trajectory/control get. Swapping changes detection —
// and where Pick goes — with nothing else touched.
async function loadVision() {
  const el = document.getElementById('visionCap');
  if (!el) return;
  let v;
  try { v = await (await fetch('/vision')).json(); } catch { return; }
  const short = s => s.replace(/^robonode\./, '');
  const chips = (v.available || []).map(name =>
    `<span class="chip ${name === v.version ? 'on' : ''}" data-v="${name}">${short(name)}</span>`).join('');
  el.innerHTML = `<h3>👁 Vision / Tracking</h3><div class="st"><span style="color:var(--ok)">● ${short(v.version)}</span> · swappable algorithm</div><div class="chips">${chips}</div>`;
  el.querySelectorAll('.chip').forEach(c => c.onclick = async () => {
    await cmd({ cmd: 'set_version', capability: 'vision', version: c.dataset.v });
    setTimeout(loadVision, 300);
  });
}
loadVision();
for (const b of document.querySelectorAll('#family button'))
  b.onclick = () => cmd({ cmd: 'driver', family: b.dataset.fam });
