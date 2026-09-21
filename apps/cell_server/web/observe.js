import { store, submit } from './transport.js';
import { t, num } from './i18n.js';
import { el, esc } from './dom.js';

// What the cell reports about itself: its state, its log, the dashboard cards,
// the sensor's view, and the errors it refuses to swallow. Reading surfaces
// only — nothing here commands anything.

export function initStatus() {
  const status = el('status');
  // Which machine this page drives, in the header: the transport bar is at the
  // other end of the window from the Robots panel, and "Run" has to be
  // unambiguous from wherever you are looking.
  const driving = el('drivingCell');
  store.subscribe(s => s.cell, id => {
    if (driving) driving.textContent = id || 'main';
  });
  const runstate = el('runstate');
  const badge = el('fambadge');
  store.subscribe(s => s.connection, c => { if (status) status.textContent = t(`status.${c}`); });
  // While a program runs, the state word alone ("moving") hides which step is
  // moving. The runner already numbers them for the log; show that instead.
  const step = el('runstep');
  store.subscribe(s => [s.state, s.latched].join(':'), (_, s) => {
    if (!runstate) return;
    runstate.textContent = t(`state.${s.state}`) ?? s.state;
    runstate.classList.toggle('run', s.state !== 'idle');
    runstate.classList.toggle('held', s.latched);
  });
  store.subscribe(s => s.step, current => {
    if (!step) return;
    step.textContent = current;
    step.classList.toggle('on', !!current);
  });
  // WHICH driver families exist is the cell's business, not the markup's: the
  // buttons are built from what the node tree declares, so a descriptor with a
  // third family needs no edit here.
  const seg = el('family');
  store.subscribe(s => s.families.join(','), (_, s) => {
    if (!seg) return;
    seg.innerHTML = s.families.map(f =>
      `<button data-fam="${esc(f)}" aria-pressed="false">${esc(f)}</button>`).join('');
    seg.querySelectorAll('button').forEach(b =>
      b.onclick = () => submit({ cmd: 'driver', family: b.dataset.fam }));
    paintFamily(s.family);
  });
  store.subscribe(s => s.family, paintFamily);

  function paintFamily(fam) {
    if (badge) { badge.textContent = fam; badge.className = `badge ${fam}`; }
    document.querySelectorAll('#family button').forEach(b => {
      const on = b.dataset.fam === fam;
      b.classList.toggle('on', on);
      b.setAttribute('aria-pressed', String(on));
    });
  }
}
export function initLogs() {
  const box = el('logs');
  if (!box) return;
  store.subscribe(s => s.logs, lines => {
    box.innerHTML = lines.map(l => {
      const cls = /error/i.test(l) ? ' class="err"' : /warn/i.test(l) ? ' class="warn"' : '';
      return `<div${cls}>${esc(l)}</div>`;
    }).join('');
    box.scrollTop = box.scrollHeight;
  });
}

export function initDash() {
  const follow = el('physFollow');
  const vision = el('visionPart');
  const speed = el('trackSpeed');
  // How fast the platform thinks the target is moving — the number the whole
  // interception depends on, so it belongs on the dashboard.
  store.subscribe(s => s.capabilities, caps => {
    if (!speed) return;
    const tracking = caps.find(c => c.id === 'tracking');
    speed.textContent = tracking ? `${num(tracking.speed ?? 0, 3)} m/s` : '—';
  });
  store.subscribe(s => s.err, err => {
    if (!follow || !err.length) return;
    let worst = 0;
    for (let i = 1; i < err.length; i++) worst = Math.max(worst, Math.abs(err[i] ?? 0));
    follow.textContent = `${num(worst, 3)} rad`;
  });
  // What the arm is touching: the difference between "the move stopped short"
  // and knowing why.
  const contacts = el('contacts');
  store.subscribe(s => s.contacts.map(c => c.join('|')).join(','), (_, s) => {
    if (!contacts) return;
    contacts.textContent = s.contacts.length
      ? s.contacts.map(([a, b]) => `${a} ↔ ${b}`).join(' · ')
      : t('dash.noContacts');
  });
  store.subscribe(s => s.part, part => {
    if (!vision) return;
    vision.textContent = part ? part.map(v => num(v, 2)).join(', ') : t('dash.noPart');
  });
}

// Failures are visible: the platform's reason, in the UI, not the console.
// A failure is announced once and then RECORDED. The announcement used to stay
// up until the next command succeeded — a banner across the transport bar,
// covering the controls you would use to recover. It fades; the reason stays
// beside the run state, where it belongs, for as long as the cell is faulted.
export function initErrorToast(holdMs = 6000) {
  const box = el('toast');
  const reason = el('runreason');
  if (!box) return;
  let fade = null;
  store.subscribe(s => s.error, err => {
    if (reason) {
      reason.textContent = err ?? '';
      reason.title = err ?? '';
      reason.classList.toggle('on', !!err);
    }
    if (!err) {
      box.classList.remove('on');
      return;
    }
    box.textContent = `${t('error.commandFailed')}: ${err}`;
    box.classList.add('on');
    clearTimeout(fade);
    fade = setTimeout(() => box.classList.remove('on'), holdMs);
  });
}

// The sensor's own view, refreshed while the page is visible. Tuning a detector
// without seeing its input is guesswork — and the crosshair the platform draws
// is where IT thinks the part is, so a confidently wrong detector looks wrong.
export function initCameraFeed(periodMs = 500) {
  const img = el('camFeed');
  if (!img) return;
  img.onerror = () => img.classList.add('off');
  img.onload = () => img.classList.remove('off');
  const tick = () => {
    // The feed follows the robot the page is driving, like every other read.
    const cell = store.get().cell;
    if (document.visibilityState === 'visible') {
      img.src = `/camera.bmp?t=${Date.now()}` + (cell ? `&cell=${encodeURIComponent(cell)}` : '');
    }
  };
  tick();
  return setInterval(tick, periodMs);
}

