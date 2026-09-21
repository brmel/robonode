import { store, send, submit, loadCollection, getJSON, setSession } from './transport.js';
import { t } from './i18n.js';
import { el, esc } from './dom.js';
import { renderRecords } from './records.js';


// The scenario as data — and the form is the editor. A user places an object,
// moves what the base world already declares, and saves; the JSON underneath is
// shown but never typed, because two ways to author the same document is two
// ways for them to disagree.
//
// The shipped library is read-only, so editing always means "yours now": a save
// lands in your session and shadows the one everybody started from.

const OBJECT_FIELDS = [
  { key: 'name', label: 'name' },
  { key: 'type', label: 'shape', type: 'select', options: ['box', 'sphere', 'cylinder'] },
  { key: 'pos', label: 'pos ', type: 'vec3' },
  { key: 'size', label: 'size ', type: 'vec3' },
  { key: 'mass', label: 'mass', type: 'number' },
  { key: 'collides', label: 'collides', type: 'bool' },
];

const OVERRIDE_FIELDS = [
  { key: 'name', label: 'body' },
  { key: 'pos', label: 'pos ', type: 'vec3' },
  { key: 'visible', label: 'visible', type: 'bool' },
  { key: 'material', label: 'material' },
  { key: 'mass', label: 'mass', type: 'number' },
];

const blankObject = () => ({
  name: 'crate', type: 'box', pos: [0.55, 0, 0.12], size: [0.1, 0.1, 0.12],
  mass: 8, collides: true, material: 'part',
});
const blankOverride = () => ({ name: '', pos: [0, 0, 0], visible: true, material: '', mass: 0 });

export async function initScenes(dock) {
  const list = el('sceneList');
  if (!list) return;
  const file = el('sceneFile'), out = el('sceneOut');
  let scene = { name: 'My scene', base: 'rail_ur10e.xml', objects: [], overrides: [] };

  const say = (msg, ok = true) => {
    out.textContent = msg;
    out.classList.toggle('err', !ok);
    out.classList.toggle('ok', ok);
  };

  const refresh = () => loadCollection('/scenes', 'scenes');

  // An override says what CHANGED and nothing else: a field left at its blank
  // value is not sent, or the platform would read it as "make it zero".
  const descriptor = () => ({
    name: el('sceneName').value.trim() || file.value,
    base: el('sceneBase').value.trim(),
    objects: scene.objects,
    overrides: scene.overrides.map(o => {
      const out = { name: o.name };
      if (o.pos?.some(v => v !== 0)) out.pos = o.pos;
      if (o.visible === false) out.visible = false;
      if (o.material) out.material = o.material;
      if (o.mass) out.mass = o.mass;
      return out;
    }),
  });

  const preview = () => { el('sceneSrc').textContent = JSON.stringify(descriptor(), null, 2); };

  const draw = () => {
    renderRecords(el('sceneObjects'), scene.objects, OBJECT_FIELDS,
                  { onChange: preview, onStructure: draw, empty: t('scenes.noObjects') });
    renderRecords(el('sceneOverrides'), scene.overrides, OVERRIDE_FIELDS,
                  { onChange: preview, onStructure: draw, empty: t('scenes.noOverrides') });
    preview();
  };

  const open = async name => {
    try {
      dock?.show('scenes');
      const loaded = await getJSON(`/scenes/${name}`);
      file.value = name;
      el('sceneName').value = loaded.name ?? name;
      el('sceneBase').value = loaded.base ?? '';
      scene = {
        objects: (loaded.objects ?? []).map(o => ({ ...blankObject(), ...o })),
        overrides: (loaded.overrides ?? []).map(o => ({ ...blankOverride(), ...o })),
      };
      draw();
      say(t('scenes.opened'));
    } catch (e) {
      say(String(e.message ?? e), false);
    }
  };

  store.subscribe(s => s.scenes.map(x => `${x.file}:${x.origin}`).join(','), (_, s) => {
    list.innerHTML = s.scenes.map(x =>
      `<button class="ncard scene" data-file="${esc(x.file)}"><h3>🗺 ${esc(x.name)}</h3>
        <div class="st">${esc(x.file)} · <span class="${x.origin === 'session' ? 'ok' : 'drv'}">${esc(x.origin)}</span></div>
      </button>`).join('') || `<span class="drv">${t('scenes.none')}</span>`;
    list.querySelectorAll('.scene').forEach(c => { c.onclick = () => open(c.dataset.file); });
  });

  el('sceneNew').onclick = () => {
    file.value = 'my-line.scene.json';
    el('sceneName').value = 'My line';
    el('sceneBase').value = 'rail_ur10e.xml';
    scene = { objects: [], overrides: [] };
    draw();
    say(t('scenes.blank'));
  };
  el('sceneAddObject').onclick = () => { scene.objects.push(blankObject()); draw(); };
  el('sceneAddOverride').onclick = () => { scene.overrides.push(blankOverride()); draw(); };

  el('sceneSave').onclick = async () => {
    const r = await send(`/scenes/${file.value}`, 'PUT', JSON.stringify(descriptor(), null, 2));
    say(r.ok ? t('scenes.saved') : r.error, r.ok);
    if (r.ok) refresh();
  };
  el('sceneDelete').onclick = async () => {
    const r = await send(`/scenes/${file.value}`, 'DELETE');
    say(r.ok ? t('scenes.deleted') : r.error, r.ok);
    if (r.ok) refresh();
  };
  el('sceneUse').onclick = async () => {
    const ack = await submit({ cmd: 'load_scene', file: file.value });
    say(ack.ok ? t('scenes.live') : ack.error, ack.ok);
  };

  el('sceneBase').value = 'rail_ur10e.xml';
  draw();
  await refresh();
}

// A session is a workspace. Naming one is all it takes to have your own scenes,
// applications and algorithms; everything reloads in that context.
export async function initSessions(reload) {
  const input = el('sessionId');
  if (!input) return;
  input.value = store.get().session;
  await loadCollection('/sessions', 'sessions');
  store.subscribe(s => s.sessions.join(','), (_, s) => {
    el('sessionList').innerHTML = s.sessions.map(i => `<option value="${esc(i)}">`).join('');
  });
  el('sessionSet').onclick = async () => {
    setSession(input.value);
    await Promise.all([loadCollection('/sessions', 'sessions'), reload()]);
  };
}
