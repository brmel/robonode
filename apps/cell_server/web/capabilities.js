import { store, submit, run, send, getJSON, loadCapabilities, loadCollection, short } from './transport.js';
import { runCompare } from './compare.js';
import { t } from './i18n.js';
import { el, esc } from './dom.js';
import { renderLibrary } from './records.js';


// One card per capability the platform declares. Nothing here knows that
// "vision" or "planner" exist — the descriptor carries title, icon and ABI.
export function initCapabilityCards(dock, compare) {
  const host = el('capabilityCards');
  if (!host) return;
  store.subscribe(s => s.capabilities, caps => {
    host.innerHTML = caps.map(c => {
      const d = c.descriptor ?? {};
      const chips = (c.available ?? []).map(v =>
        `<button class="chip${v === c.version ? ' on' : ''}" data-cap="${esc(c.id)}" data-v="${esc(v)}"
           aria-pressed="${v === c.version}">${esc(short(v))}</button>`).join('');
      return `<div class="ncard" id="cap-${esc(c.id)}">
        <h3>${esc(d.icon ?? '')} ${esc(d.title ?? c.id)}</h3>
        <div class="st"><span class="ok">● ${esc(short(c.version))}</span>${d.authorable ? ' · authorable' : ''}</div>
        <div class="chips">${chips}</div>
        ${(c.available ?? []).length > 1 && c.trial
          ? `<button class="capjudge" data-cap="${esc(c.id)}"
               title="${esc(t('compare.judge'))}">⇄ ${esc(t('compare.judgeShort'))}</button>`
          : ''}</div>`;
    }).join('');
    // Swapping a version and judging it are the same question asked twice; the
    // card answers both, instead of sending you to a panel to re-pick what you
    // were already looking at.
    host.querySelectorAll('.capjudge').forEach(b => {
      b.onclick = () => {
        compare?.focus(b.dataset.cap);
        dock?.show('compare');
      };
    });
    host.querySelectorAll('.chip').forEach(c => {
      c.onclick = async () => {
        await run({ cmd: 'set_version', capability: c.dataset.cap, version: c.dataset.v });
        loadCapabilities();
      };
    });
  });
}

// The editor's language, template and hint all come from the descriptor's ABI.
export function initEditor(dock) {
  const pick = el('modCap'), src = el('modSrc'), hint = el('modHint'), out = el('modOut');
  if (!pick) return;

  const descriptorOf = id => store.get().capabilities.find(c => c.id === id)?.descriptor;
  const syncAbi = () => {
    const d = descriptorOf(pick.value);
    if (!d) return;
    src.value = d.template ?? '';
    hint.innerHTML = d.hint ?? '';
  };
  pick.onchange = syncAbi;

  store.subscribe(s => s.capabilities.filter(c => c.descriptor?.authorable).map(c => c.id).join(','), (_, s) => {
    const authorable = s.capabilities.filter(c => c.descriptor?.authorable);
    pick.innerHTML = authorable.map(c =>
      `<option value="${esc(c.id)}">${esc(c.descriptor.icon ?? '')} ${esc(c.descriptor.title)}</option>`).join('');
    syncAbi();
  });

  // Authoring without running it against anything is a compile check, not a
  // test. Whatever version was live when you pressed compile becomes the
  // baseline, so the next click answers "is mine better than what it replaced?"
  let baseline = null;
  const test = el('modTest');

  el('modRun').onclick = async () => {
    const capability = pick.value;
    const name = el('modName').value.trim() || 'my-algo';
    baseline = { capability, version: store.get().capabilities.find(c => c.id === capability)?.version };
    out.className = 'eout';
    out.textContent = '…';
    const ack = await submit({ cmd: 'define_module', capability, name, source: src.value });
    out.className = `eout ${ack.ok ? 'ok' : 'err'}`;
    out.textContent = ack.ok ? `● ${ack.version}` : `✗ ${ack.error}`;
    if (!ack.ok) return;
    loadCapabilities();
    loadLibrary();
    if (!test) return;
    baseline.mine = ack.version;
    test.disabled = false;
    test.textContent = `${t('editor.test')} ${short(baseline.version ?? '')}`;
    test.title = `${t('editor.test')} ${baseline.version ?? ''}`;
  };

  if (test) {
    test.disabled = true;
    // A disabled control that does not say why reads as broken. It says what
    // unlocks it until it is unlocked, and what it will judge against after.
    test.title = t('editor.testWhy');
    test.onclick = async () => {
      if (!baseline?.mine) return;
      dock?.show('compare');
      await runCompare(baseline.capability, [baseline.version, baseline.mine].filter(Boolean));
    };
  }
  loadLibrary();
}

export async function loadLibrary() {
  const box = el('modLib');
  if (!box) return;
  await loadCollection('/modules', 'modules');
  const mods = store.get().modules;
  renderLibrary(box, mods, {
    name: m => m.name,
    load: t('editor.load'),
    empty: t('editor.none'),
    onLoad: async file => {
      const j = await getJSON(`/modules/${file}`);
      el('modCap').value = j.capability;
      el('modName').value = j.name;
      el('modSrc').value = j.source;
    },
    onDelete: async file => {
      await send(`/modules/${file}`, 'DELETE');
      loadLibrary();
    },
  });
}
