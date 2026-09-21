import { store, send, loadCollection, loadCapabilities, short, getJSON } from './transport.js';
import { t, num } from './i18n.js';
import { el, esc } from './dom.js';
import { renderLibrary } from './records.js';

// The comparison itself lives in the platform (POST /compare): it runs the work
// that exercises THIS capability once per version and reports what happened.
// The panel renders that answer — it does not measure a second one of its own,
// because two definitions of "A vs B" is exactly the drift ADR-8 forbids.
export function initCompare() {
  const body = el('compareBody');
  if (!body) return;
  body.innerHTML = `
    <div class="erow">
      <select id="cmpCap" aria-label="capability"></select>
      <select id="cmpA" aria-label="version A"></select>
      <span class="tlabel">vs</span>
      <select id="cmpB" aria-label="version B"></select>
      <button id="cmpRun" class="primary">▶ ${t('compare.run')}</button>
    </div>
    <div id="cmpTrial" class="ehint"></div>
    <div id="cmpOut" class="drv" style="margin-top:var(--sp4)">${t('compare.hint')}</div>
    <h3 class="ehead" data-t="compare.kept">${t('compare.kept')}</h3>
    <div id="cmpRuns" class="drv"></div>`;

  const cap = el('cmpCap'), a = el('cmpA'), b = el('cmpB');

  const capabilityOf = id => store.get().capabilities.find(c => c.id === id);
  const versionsOf = id => capabilityOf(id)?.available ?? [];
  const trialOf = id => capabilityOf(id)?.trial ?? '';

  store.subscribe(s => s.capabilities.map(c => `${c.id}:${(c.available ?? []).join(',')}`).join('|'), (_, s) => {
    cap.innerHTML = s.capabilities.map(c =>
      `<option value="${esc(c.id)}">${esc(c.descriptor?.icon ?? '')} ${esc(c.descriptor?.title ?? c.id)}</option>`).join('');
    fill();
  });

  function fill() {
    const opts = versionsOf(cap.value).map(v => `<option value="${esc(v)}">${esc(short(v))}</option>`).join('');
    a.innerHTML = opts;
    b.innerHTML = opts;
    if (versionsOf(cap.value).length > 1) b.selectedIndex = 1;
    // A capability with no trial cannot be compared honestly — say so instead
    // of running some other capability's work and reporting a number.
    const trial = trialOf(cap.value);
    el('cmpTrial').textContent = trial ? `${t('compare.trial')}: ${trial}` : t('compare.noTrial');
    el('cmpRun').disabled = !trial;
  }
  cap.onchange = fill;

  el('cmpRun').onclick = () => runCompare(cap.value, [a.value, b.value]);

  // Judging an algorithm should start where you were looking at it. A card
  // hands the capability over with the LIVE version on one side, so the
  // question the panel opens on is "is the other one better than what is
  // running?" rather than an empty form to fill in again.
  const focus = id => {
    if (!versionsOf(id).length) return;
    cap.value = id;
    fill();
    const live = capabilityOf(id)?.version;
    if (live && versionsOf(id).includes(live)) {
      a.value = live;
      const other = versionsOf(id).find(v => v !== live);
      if (other) b.value = other;
    }
  };

  // Every comparison is kept in the session that ran it, so the evidence
  // survives the page: it is a document like a scene or an app.
  store.subscribe(s => s.runs.map(r => r.file).join(','), (_, s) => {
    const host = el('cmpRuns');
    if (!host) return;
    renderLibrary(host, s.runs, {
      name: r => r.name ?? r.file,
      load: t('editor.load'),
      empty: t('compare.noRuns'),
      onLoad: async file => showReport(await getJSON(`/runs/${file}`)),
      onDelete: async file => {
        await send(`/runs/${file}`, 'DELETE');
        loadRuns();
      },
    });
  });
  loadRuns();
  return { focus };
}

const loadRuns = () => loadCollection('/runs', 'runs');

// Shared with the algorithm editor: author a version, then put it under the
// same load, against the one it replaced.
export async function runCompare(capability, versions) {
  const out = el('cmpOut');
  if (out) {
    out.textContent = t('compare.running');
    out.classList.remove('err');
  }
  // The trial runs on the robot the page is driving, not always the first one.
  const report = await send('/compare', 'POST',
                            JSON.stringify({ capability, versions, cell: store.get().cell }));
  if (!out) return report;
  if (!report?.results) {
    out.textContent = report?.error ?? t('compare.failed');
    out.classList.add('err');
    return report;
  }
  showReport(report);
  loadCapabilities();
  loadRuns();
  return report;
}

// A stored report and a fresh one are the same document, so they render through
// the same function — a history that looked different from the live result
// would be a second answer to the same question.
function showReport(report) {
  const out = el('cmpOut');
  if (!out || !report?.results) return;
  // Two control versions take the same time and track the path differently —
  // so the number that separates them is the following error, per axis, in
  // that axis's own unit.
  const follow = r => {
    const w = r.quality?.worst_axis;
    return w && w.id !== '-' ? `${num(w.following_error ?? 0, 3)} ${esc(w.unit ?? '')} · ${esc(w.id)}` : '—';
  };
  const row = r =>
    `<tr><td>${esc(short(r.version))}</td>
      <td class="${r.ok ? 'ok' : 'warn'}">${r.ok ? t('compare.pass') : t('compare.fail')}</td>
      <td>${follow(r)}</td>
      <td>${num(r.seconds ?? 0, 1)} s</td><td class="drv">${esc(r.error ?? '')}</td></tr>`;
  out.classList.remove('err');
  out.innerHTML = `<div class="ehint">${t('compare.trial')}: ${esc(report.trial)}</div>
    <table><thead><tr><th>${t('compare.version')}</th><th>${t('compare.outcome')}</th>
      <th>${t('compare.follow')}</th><th>${t('compare.time')}</th>
      <th>${t('compare.why')}</th></tr></thead>
    <tbody>${report.results.map(row).join('')}</tbody></table>`;
}
