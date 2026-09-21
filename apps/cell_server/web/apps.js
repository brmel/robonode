import { store, send, submit, loadCollection, getJSON } from './transport.js';
import { t } from './i18n.js';
import { el, esc } from './dom.js';
import { renderRecords } from './records.js';


export async function initApps(dock) {
  const host = document.getElementById('apps');
  if (!host) return;
  await loadCollection('/apps', 'apps');
  store.subscribe(s => s.apps.map(a => `${a.file}:${a.origin}`).join(','), (_, s) => {
    host.innerHTML = s.apps.map(a =>
      `<div class="approw"><button class="ncard app" data-file="${esc(a.file)}" data-origin="${esc(a.origin ?? '')}"><h3>🎯 ${esc(a.name)}</h3>
        <div class="st">${esc(a.origin ?? '')} · ${t('apps.clickToRun')}</div></button>
        <button class="appedit" data-file="${esc(a.file)}" aria-label="edit ${esc(a.name)}">✎</button></div>`).join('');
    host.querySelectorAll('.app').forEach(c =>
      c.onclick = () => submit({ cmd: 'run_app', file: c.dataset.file }));
    host.querySelectorAll('.appedit').forEach(b =>
      b.onclick = () => openApp(b.dataset.file, dock));
  });

  // Which app is running is platform state: the card that is live says so, and
  // says which step it is on, instead of the operator matching a log line to a
  // card by hand.
  store.subscribe(s => `${s.app}:${s.step}`, (_, s) => {
    host.querySelectorAll('.app').forEach(card => {
      const live = card.dataset.file === s.app;
      card.classList.toggle('running', live);
      const status = card.querySelector('.st');
      if (status) {
        status.textContent = live ? s.step || t('apps.running')
                                  : `${card.dataset.origin ?? ''} · ${t('apps.clickToRun')}`;
      }
    });
  });
}

// The page has one application editor; its program and its verb table live in
// its closure, so nothing else can reach in and change what is on screen.
let editor = null;

const openApp = (file, dock) => editor?.open(file, dock);

// An application is a composition, not code: a cell plus a list of steps. The
// editor never knows which steps exist — GET /verbs is the same table the
// runner executes, so what can be authored is exactly what can run.
export async function initAppEditor() {
  if (!el('appSteps')) return;
  const verbs = await getJSON('/verbs').catch(() => []);
  let program = [];

  const say = (msg, ok = true) => {
    const out = el('appOut');
    out.textContent = msg;
    out.classList.toggle('err', !ok);
    out.classList.toggle('ok', ok);
  };

  // A record holds its arguments as plain fields; the wire form nests them
  // under `args`, so the translation happens once, here.
  const descriptor = () => ({
    name: el('appName').value.trim() || el('appFile').value,
    cell: el('appCell').value.trim(),
    program: program.map(({ verb, ...args }) => ({
      verb,
      args: Object.fromEntries(Object.entries(args).filter(([, v]) => v !== '' && v != null)),
    })),
  });

  // Suggestions come from what the platform reports it has, so the editor never
  // carries a list of capability or station ids of its own.
  const suggestions = arg => {
    const s = store.get();
    if (arg === 'capability') return s.capabilities.map(c => c.id);
    if (arg === 'version') return s.capabilities.flatMap(c => c.available ?? []);
    if (arg === 'station') return s.stations.map(x => x.id);
    if (arg === 'run') return ['true', 'false'];  // the belt is on or it is not
    return [];
  };

  // Rows are the shared widget (records.js); what differs is that a step's
  // fields depend on its verb, which the platform publishes.
  const fieldsFor = step => {
    const spec = verbs.find(v => v.verb === step.verb) ?? { args: [] };
    // An argument the platform can enumerate is chosen, not typed. The
    // suggestions were already computed and then thrown away: every field
    // rendered as free text, so a mistyped station saved fine and failed at run
    // time, which is the worst place to find out.
    return spec.args.map(a => {
      const options = suggestions(a);
      return options.length ? { key: a, label: a, type: 'select', options } : { key: a, label: a };
    });
  };

  const verbPicker = (step, i) =>
    `<select class="stepverb" data-i="${i}" aria-label="step ${i + 1} verb">
      ${verbs.map(v => `<option value="${esc(v.verb)}"${v.verb === step.verb ? ' selected' : ''}>${esc(v.verb)}</option>`).join('')}
    </select>`;

  function render() {
    const host = el('appSteps');
    renderRecords(host, program, fieldsFor, {
      onStructure: render,
      empty: t('apps.noSteps'),
      lead: verbPicker,
    });
    // Changing the verb changes which arguments exist, so that one IS structural.
    host.querySelectorAll('.stepverb').forEach(sel =>
      sel.onchange = e => {
        program[Number(e.currentTarget.dataset.i)] = { verb: e.currentTarget.value };
        render();
      });
  }

  el('appVerb').innerHTML = verbs.map(v => `<option value="${esc(v.verb)}">${esc(v.verb)}</option>`).join('');
  // Which robot an app runs on is a document in the catalogue, so offer them.
  store.subscribe(s => s.robots.map(r => r.file).join(','), (_, s) => {
    const list = el('appCellList');
    if (list) list.innerHTML = s.robots.map(r => `<option value="${esc(r.file)}"></option>`).join('');
  });
  el('appAdd').onclick = () => { program.push({ verb: el('appVerb').value }); render(); };
  el('appNew').onclick = () => {
    program = [];
    el('appFile').value = 'my-app.app.json';
    el('appName').value = 'My application';
    render();
    say(t('apps.blank'));
  };
  el('appSave').onclick = async () => {
    const r = await send(`/apps/${el('appFile').value}`, 'PUT', JSON.stringify(descriptor(), null, 2));
    say(r.ok ? t('apps.saved') : r.error, r.ok);
    if (r.ok) loadCollection('/apps', 'apps');
  };
  el('appDelete').onclick = async () => {
    const r = await send(`/apps/${el('appFile').value}`, 'DELETE');
    say(r.ok ? t('apps.deleted') : r.error, r.ok);
    if (r.ok) loadCollection('/apps', 'apps');
  };
  el('appRun').onclick = async () => {
    const ack = await submit({ cmd: 'run_app', file: el('appFile').value });
    say(ack.ok ? t('apps.running') : ack.error, ack.ok);
  };

  editor = {
    async open(file, dock) {
      try {
        const app = await getJSON(`/apps/${file}`);
        dock?.show('apps');
        el('appFile').value = file;
        el('appName').value = app.name ?? file;
        el('appCell').value = app.cell ?? '';
        program = (app.program ?? []).map(s => ({ verb: s.verb, ...(s.args ?? {}) }));
        render();
        say(t('apps.opened'));
      } catch (e) {
        say(String(e.message ?? e), false);
      }
    },
  };
  render();
}

export async function initStations() {
  const host = document.getElementById('stationCard');
  if (!host) return;
  await loadCollection('/stations', 'stations');
  const st = store.get().stations;
  if (!st.length) return;
  // A station's whole config is a descriptor, not a dashboard: show what it IS
  // and how fast it runs. The rest is one GET /stations away.
  host.innerHTML = st.map(s => {
    const speed = s.config?.speed_m_s;
    return `<h3>📦 ${esc(s.id)}</h3><div class="st"><span class="ok">● ${esc(s.type)}</span>${
      speed ? ` · ${esc(speed)} m/s` : ''}</div>`;
  }).join('');
}
