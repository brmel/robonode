import { store, send, loadCollection, getJSON, setCell, connect } from './transport.js';
import { renderRecords } from './records.js';
import { t } from './i18n.js';
import { el, esc } from './dom.js';

// A robot is a cell descriptor: which joints, which carrier, which world. The
// catalogue is a document collection like any other, so starting a second
// machine is picking one from a list — and what makes it a DIFFERENT machine is
// visible in what it runs with (a chain of six joints, or seven).
export async function initRobots() {
  const list = el('robotList');
  if (!list) return;
  await loadCollection('/robots', 'robots');

  const running = async () => {
    const cells = await getJSON('/cells').catch(() => []);
    el('cellList').innerHTML = cells.map(c =>
      `<div class="libitem${c.id === (store.get().cell || 'main') ? ' on' : ''}">
        <span class="libname">${esc(c.id)}</span>
        <span class="drv">${esc(c.nodes)} ${t('robots.joints')} · ${esc(c.family)}</span>
        <button class="celldrive" data-id="${esc(c.id)}">${t('robots.drive')}</button>
        ${c.id === 'main' ? '' : `<button class="cellstop" data-id="${esc(c.id)}">${t('robots.stop')}</button>`}
      </div>`).join('');
    // Choosing a robot re-points every read AND every command: "run" means the
    // machine you are looking at, or the button is a trap.
    el('cellList').querySelectorAll('.celldrive').forEach(b => b.onclick = () => {
      setCell(b.dataset.id === 'main' ? '' : b.dataset.id);
      connect();
      running();
    });
    el('cellList').querySelectorAll('.cellstop').forEach(b => b.onclick = async () => {
      await send(`/cells/${b.dataset.id}`, 'DELETE');
      running();
    });
  };

  store.subscribe(s => s.robots.map(r => r.file).join(','), (_, s) => {
    list.innerHTML = s.robots.map(r =>
      `<div class="ncard"><h3>🦾 ${esc(r.name)}</h3>
        <div class="st">${esc(r.file)}</div>
        <div class="chips">
          <button class="chip robotstart" data-file="${esc(r.file)}">${t('robots.start')}</button>
          <button class="chip robotfork" data-file="${esc(r.file)}">${t('robots.fork')}</button>
          ${r.origin === 'session' ? `<button class="chip robotedit" data-file="${esc(r.file)}">${t('robots.edit')}</button>` : ''}
        </div>
      </div>`).join('') || `<span class="drv">${t('robots.none')}</span>`;
    // Forking is how a shipped robot becomes yours: the copy lands in your
    // session, and the platform refuses to store one whose axes disagree with
    // themselves — so an edited robot fails when you save it, not when you run it.
    list.querySelectorAll('.robotfork').forEach(b => b.onclick = async () => {
      const to = b.dataset.file.replace(/\.cell\.json$/, '-mine.cell.json');
      await send(`/robots/${b.dataset.file}/fork`, 'POST', JSON.stringify({ to }));
      loadCollection('/robots', 'robots');
    });
    list.querySelectorAll('.robotstart').forEach(b => b.onclick = async () => {
      const id = b.dataset.file.replace(/\.cell\.json$/, '');
      await send('/cells', 'POST', JSON.stringify({ id, robot: b.dataset.file }));
      running();
    });

    // Only what you own is editable: the shipped catalogue is the thing
    // everybody else started from.
    list.querySelectorAll('.robotedit').forEach(b => b.onclick = () => openRobot(b.dataset.file));
  });

  running();
}

// A robot's joints, with the travel and speed each one is allowed. This is the
// same record widget the application and scene editors use — a third list of
// things with fields, not a third implementation of one.
const NODE_FIELDS = [
  { key: 'id', label: 'axis' },
  { key: 'unit', label: 'unit' },
  { key: 'position_min', label: 'min', type: 'number' },
  { key: 'position_max', label: 'max', type: 'number' },
  { key: 'velocity_max', label: 'v', type: 'number' },
  { key: 'acceleration_max', label: 'a', type: 'number' },
  { key: 'jerk_max', label: 'jerk', type: 'number' },
];

let robot = null;  // the document being edited, whole, so nothing is lost on save

async function openRobot(file) {
  const editor = el('robotEditor');
  robot = await getJSON(`/robots/${file}`);
  el('robotFile').value = file;
  editor.hidden = false;
  drawNodes();
}

function drawNodes() {
  const rows = (robot.nodes ?? []).map(n => ({ id: n.id, unit: n.unit, ...n.limits }));
  renderRecords(el('robotNodes'), rows, NODE_FIELDS, {
    onChange: applied => write(applied),
    onStructure: applied => { write(applied); drawNodes(); },
  });
}

// The rows are a VIEW of the document: writing them back keeps every field the
// editor does not show (joints, actuators, stations, motions) exactly as it was.
function write(rows) {
  const by = new Map(rows.map(r => [r.id, r]));
  robot.nodes = (robot.nodes ?? [])
    .filter(n => by.has(n.id))
    .map(n => {
      const r = by.get(n.id);
      return { ...n, unit: r.unit, limits: {
        position_min: r.position_min, position_max: r.position_max,
        velocity_max: r.velocity_max, acceleration_max: r.acceleration_max,
        jerk_max: r.jerk_max,
      } };
    });
}

export function initRobotEditor() {
  const editor = el('robotEditor');
  if (!editor) return;
  el('robotClose').onclick = () => { editor.hidden = true; };
  el('robotSave').onclick = async () => {
    const out = el('robotOut');
    const r = await send(`/robots/${el('robotFile').value}`, 'PUT', JSON.stringify(robot, null, 2));
    out.textContent = r.ok ? t('robots.saved') : r.error;
    out.classList.toggle('ok', !!r.ok);
    out.classList.toggle('err', !r.ok);
  };
}
