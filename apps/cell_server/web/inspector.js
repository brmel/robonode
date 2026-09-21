import { store, submit, short } from './transport.js';
import { t, num } from './i18n.js';
import { el, esc } from './dom.js';

// The Cell tab: the robot node's joints as a table, and whichever one is
// selected in detail. Everything here is one subject — what the arm is doing,
// joint by joint.

// A joint speaks its own unit: millimetres for a rail, radians for an arm.
const unit = (value, n) => (n?.unit === 'mm' ? num(value, 0) : num(value, 3)) + ' ' + (n?.unit ?? '');

export function initNodeTable() {
  const body = el('nodes');
  if (!body) return;
  let cells = [];

  store.subscribe(s => s.nodes.map(n => `${n.id}:${n.driver}`).join('|'), (_, s) => {
    body.innerHTML = '';
    cells = s.nodes.map((n, i) => {
      const tr = document.createElement('tr');
      tr.tabIndex = 0;
      tr.setAttribute('role', 'button');
      tr.setAttribute('aria-label', `${n.id}, ${short(n.driver)}`);

      const id = document.createElement('td');
      id.textContent = n.id;

      const drv = document.createElement('td');
      const sel = document.createElement('select');
      sel.setAttribute('aria-label', `${n.id} driver`);
      for (const d of s.drivers) {
        const o = document.createElement('option');
        o.value = d; o.textContent = short(d);
        o.selected = d === n.driver;
        sel.appendChild(o);
      }
      sel.onchange = () => submit({ cmd: 'set_driver', node: n.id, driver: sel.value });
      drv.appendChild(sel);

      const val = document.createElement('td');
      val.className = 'val';
      tr.append(id, drv, val);
      const select = ev => { if (ev.target.tagName !== 'SELECT') store.patch({ selected: i }); };
      tr.onclick = select;
      tr.onkeydown = ev => { if (ev.key === 'Enter' || ev.key === ' ') { ev.preventDefault(); select(ev); } };
      body.appendChild(tr);
      return val;
    });
  });

  store.subscribe(s => [s.selected, s.nodes.length].join(':'), () => {
    [...body.children].forEach((tr, i) => {
      const on = i === store.get().selected;
      tr.classList.toggle('sel', on);
      tr.setAttribute('aria-pressed', String(on));
    });
  });

  store.subscribe(s => s.pos, (pos, s) => {
    s.nodes.forEach((n, i) => {
      if (!cells[i]) return;
      const e = Math.abs(s.err[i] ?? 0);
      const mm = n.unit === 'mm';
      const gap = e > (mm ? 0.5 : 0.002)
        ? `<span class="gap">Δ${mm ? num(e, 0) : num(e, 3)}</span>` : '';
      cells[i].innerHTML = `${unit(pos[i] ?? 0, n)} ${gap}`;
    });
  });
}

export function initInspector() {
  const box = el('inspector');
  if (!box) return;
  const render = () => {
    const s = store.get();
    const n = s.nodes[s.selected];
    if (!n) { box.innerHTML = `<span class="drv">${t('inspector.selectNode')}</span>`; return; }
    const rows = [
      [t('inspector.capability'), 'MotionAxis@1'],
      [t('inspector.driver'), short(n.driver)],
      [t('inspector.limits'), `${unit(n.lo, n)} … ${unit(n.hi, n)}`],
      [t('inspector.setpoint'), unit(s.target[s.selected], n)],
      [t('inspector.actual'), unit(s.pos[s.selected], n)],
      [t('inspector.following'), unit(Math.abs(s.err[s.selected] ?? 0), n)],
      [t('inspector.state'), n.state],
    ];
    box.innerHTML =
      `<div class="kv"><span class="k">node</span><span class="v">${esc(n.id)}</span></div>` +
      rows.map(([k, v]) => `<div class="kv"><span class="k">${esc(k)}</span><span class="v">${esc(v)}</span></div>`).join('');
  };
  // Values, not the raw stream: a redraw happens when a shown number changes.
  store.subscribe(s => [s.selected, s.nodes.length, s.pos[s.selected], s.target[s.selected], s.err[s.selected]].join(','), render);
}

