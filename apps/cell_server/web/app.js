import * as THREE from './three.module.min.js';

// ---- scene ---------------------------------------------------------------
const view = document.getElementById('view');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e1116);

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
camera.position.set(1.6, 1.35, 2.1);
camera.lookAt(0.4, 0.5, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(devicePixelRatio);
view.appendChild(renderer.domElement);

scene.add(new THREE.HemisphereLight(0xbcd0ff, 0x202028, 0.9));
const key = new THREE.DirectionalLight(0xffffff, 1.4);
key.position.set(2, 4, 3);
scene.add(key);

// floor grid
const grid = new THREE.GridHelper(6, 24, 0x33404f, 0x222a34);
grid.position.y = 0;
scene.add(grid);

// rail track (0..1.45 m along X)
const rail = new THREE.Mesh(
  new THREE.BoxGeometry(1.45, 0.03, 0.24),
  new THREE.MeshStandardMaterial({ color: 0x2a3340, metalness: 0.3, roughness: 0.7 }));
rail.position.set(1.45 / 2, 0.015, 0);
scene.add(rail);

// ---- robot chain ---------------------------------------------------------
// carriage rides the rail (X, metres); base + UR-style 6R arm on top.
const carriage = new THREE.Group();
scene.add(carriage);

const steel = new THREE.MeshStandardMaterial({ color: 0x8894a6, metalness: 0.4, roughness: 0.5 });
const blue = new THREE.MeshStandardMaterial({ color: 0x4c8dff, metalness: 0.3, roughness: 0.45 });
const orange = new THREE.MeshStandardMaterial({ color: 0xcf8a3a, metalness: 0.3, roughness: 0.6 });

function box(w, h, d, mat, y = 0) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  m.position.y = y;
  return m;
}
// carriage block
carriage.add(box(0.18, 0.1, 0.18, orange, 0.08));
const base = new THREE.Group();
base.position.y = 0.13;
carriage.add(base);
base.add(new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.08, 0.08, 24), steel));

// Joint chain: each entry rotates about `axis`, then a link of `len` extends
// along +Y to the next joint. Believable UR10e proportions (visual twin).
const chain = [
  { axis: 'y', len: 0.10, r: 0.062, mat: steel },  // j1 base yaw
  { axis: 'z', len: 0.61, r: 0.055, mat: blue },   // j2 shoulder
  { axis: 'z', len: 0.57, r: 0.045, mat: blue },   // j3 elbow
  { axis: 'z', len: 0.12, r: 0.040, mat: steel },  // j4 wrist1
  { axis: 'y', len: 0.12, r: 0.040, mat: steel },  // j5 wrist2
  { axis: 'z', len: 0.09, r: 0.032, mat: blue },   // j6 wrist3
];
const joints = [];
let parent = base;
for (const seg of chain) {
  const g = new THREE.Group();
  parent.add(g);
  // link cylinder from origin to +len along Y
  const link = new THREE.Mesh(new THREE.CylinderGeometry(seg.r, seg.r, seg.len, 20), seg.mat);
  link.position.y = seg.len / 2;
  g.add(link);
  // joint collar
  const collar = new THREE.Mesh(new THREE.CylinderGeometry(seg.r * 1.25, seg.r * 1.25, seg.r * 1.1, 20),
    new THREE.MeshStandardMaterial({ color: 0x39d98a, metalness: 0.2, roughness: 0.4 }));
  collar.rotation.z = seg.axis === 'y' ? 0 : Math.PI / 2;
  g.add(collar);
  const next = new THREE.Group();
  next.position.y = seg.len;
  g.add(next);
  joints.push({ group: g, axis: seg.axis });
  parent = next;
}
// TCP marker
parent.add(new THREE.Mesh(new THREE.SphereGeometry(0.02, 16, 16),
  new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0x224466 })));

const AX = { x: new THREE.Vector3(1, 0, 0), y: new THREE.Vector3(0, 1, 0), z: new THREE.Vector3(0, 0, 1) };

// ---- live pose (lerp toward latest telemetry) ----------------------------
let target = [0, 0, 0, 0, 0, 0, 0];   // [rail_mm, j1..j6 rad]
let shown = [0, 0, 0, 0, 0, 0, 0];

function applyPose() {
  for (let i = 0; i < 7; i++) shown[i] += (target[i] - shown[i]) * 0.25;
  carriage.position.x = shown[0] / 1000;      // mm → m
  for (let i = 0; i < 6; i++) {
    joints[i].group.quaternion.setFromAxisAngle(AX[joints[i].axis], shown[i + 1]);
  }
}

function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);
resize();

let spin = 0;
function loop() {
  applyPose();
  spin += 0.0015;
  camera.position.x = 0.4 + 1.9 * Math.cos(spin);
  camera.position.z = 1.9 * Math.sin(spin);
  camera.position.y = 1.35;
  camera.lookAt(0.4, 0.55, 0);
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
let partPose = null;
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
for (const b of document.querySelectorAll('#family button'))
  b.onclick = () => cmd({ cmd: 'driver', family: b.dataset.fam });
