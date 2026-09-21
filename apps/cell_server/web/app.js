import { restore, toggle } from './theme.js';
import { connect, store, submit, loadCapabilities, pollCapabilities } from './transport.js';
import { initScene } from './scene.js';
import { initNodeTable, initInspector } from './inspector.js';
import { initLogs, initDash, initErrorToast, initStatus, initCameraFeed } from './observe.js';
import { initCapabilityCards, initEditor, loadLibrary } from './capabilities.js';
import { initCompare } from './compare.js';
import { initApps, initAppEditor, initStations } from './apps.js';
import { initScenes, initSessions } from './scenes.js';
import { initRobots, initRobotEditor } from './robots.js';
import { initDock } from './dock.js';
import { t, num } from './i18n.js';

restore();
document.getElementById('themeToggle').onclick = toggle;

// Static copy comes from the catalogue, so a second language is a data change.
document.querySelectorAll('[data-t]').forEach(e => { e.textContent = t(e.dataset.t); });
document.querySelectorAll('[data-t-title]').forEach(e => {
  e.title = t(e.dataset.tTitle);
  e.setAttribute('aria-label', t(e.dataset.tTitle));
});

initScene(document.getElementById('view'));
initStatus();
initNodeTable();
initInspector();
initLogs();
initDash();
initCameraFeed();
initErrorToast();
const dock = initDock();
const compare = initCompare();
initCapabilityCards(dock, compare);
initEditor(dock);
initApps(dock);
initAppEditor();
initStations();
initRobots();
initRobotEditor();
initScenes(dock);
// Switching session re-reads everything a session owns.
initSessions(() => Promise.all([initApps(dock), initScenes(dock), loadLibrary()]));

connect();
pollCapabilities();

const tx = document.getElementById('tx'), ty = document.getElementById('ty'), tz = document.getElementById('tz');
const trx = document.getElementById('trx'), try_ = document.getElementById('try_'),
      trz = document.getElementById('trz');

// Orientation is entered in degrees because that is what a person reading a
// drawing has; the platform speaks quaternions, so the conversion lives here —
// once, next to the inputs, not in every caller.
const deg = Math.PI / 180;
const quatFromRpy = (r, p, y) => {
  const [cr, sr] = [Math.cos(r * deg / 2), Math.sin(r * deg / 2)];
  const [cp, sp] = [Math.cos(p * deg / 2), Math.sin(p * deg / 2)];
  const [cy, sy] = [Math.cos(y * deg / 2), Math.sin(y * deg / 2)];
  return {
    qw: cr * cp * cy + sr * sp * sy,
    qx: sr * cp * cy - cr * sp * sy,
    qy: cr * sp * cy + sr * cp * sy,
    qz: cr * cp * sy - sr * sp * cy,
  };
};

const rpyFromQuat = ([w, x, y, z]) => {
  const sinp = 2 * (w * y - z * x);
  return [
    Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) / deg,
    (Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp)) / deg,
    Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) / deg,
  ];
};

// Seed both halves from where the tool actually is, so the first move is a
// nudge from the current pose rather than a jump to whatever was in the boxes.
let seeded = false;
store.subscribe(s => s.tcp, tcp => {
  if (seeded || !tcp) return;
  [tx.value, ty.value, tz.value] = tcp.map(v => num(v, 2));
  seeded = true;
});
let seededRpy = false;
store.subscribe(s => s.tcpQuat, q => {
  if (seededRpy || !q) return;
  [trx.value, try_.value, trz.value] = rpyFromQuat(q).map(v => Math.round(v));
  seededRpy = true;
});

const on = (id, body) => {
  const b = document.getElementById(id);
  if (b) b.onclick = () => submit(typeof body === 'function' ? body() : body);
};
on('run', { cmd: 'run' });
on('stop', { cmd: 'stop' });
on('estop', { cmd: 'estop' });
on('resume', { cmd: 'resume' });
on('movel', () => ({ cmd: 'move_l', x: +tx.value, y: +ty.value, z: +tz.value }));
on('movep', () => ({
  cmd: 'move_pose', x: +tx.value, y: +ty.value, z: +tz.value,
  ...quatFromRpy(+trx.value, +try_.value, +trz.value),
}));
on('pick', () => {
  const p = store.get().part;
  return p ? { cmd: 'move_l', x: p[0], y: p[1], z: p[2] } : { cmd: 'noop' };
});


// An e-stop you have to aim at is not an e-stop. The keys drive the same
// buttons, so there is one command path — and they stay out of the way while
// someone is typing an algorithm or editing a scene.
const SHORTCUTS = { ' ': 'run', s: 'stop', escape: 'estop', r: 'resume' };
const typing = el => el instanceof HTMLInputElement || el instanceof HTMLTextAreaElement ||
  el instanceof HTMLSelectElement || el?.isContentEditable;

for (const [key, id] of Object.entries(SHORTCUTS)) {
  const b = document.getElementById(id);
  const label = key === ' ' ? 'Space' : key === 'escape' ? 'Esc' : key.toUpperCase();
  b.setAttribute('aria-keyshortcuts', label);
  b.title = `${b.textContent.trim()} · ${label}`;
}

document.addEventListener('keydown', e => {
  if (e.metaKey || e.ctrlKey || e.altKey || typing(e.target)) return;
  const id = SHORTCUTS[e.key.toLowerCase()];
  if (!id) return;
  e.preventDefault();
  document.getElementById(id).click();
});
