import { createStore } from './store.js';

// Every byte in and out of the platform. Commands are acknowledged with an id
// and completion is observed on the stream (`applied_id`), so no caller sleeps,
// polls, or guesses. Failures land in the store instead of being swallowed.
export const store = createStore({
  connection: 'connecting',
  nodes: [],
  drivers: [],
  family: '',
  families: [],
  selected: 0,
  pos: [],
  target: [],
  err: [],
  tcp: null,
  tcpQuat: null,
  part: null,
  workpiece: null,
  holding: false,
  contacts: [],
  state: 'idle',
  step: '',
  app: '',
  latched: false,
  appliedId: 0,
  acceptedId: 0,
  logs: [],
  capabilities: [],
  apps: [],
  stations: [],
  modules: [],
  scenes: [],
  runs: [],
  robots: [],
  sessions: [],
  session: localStorage.getItem('robonode-session') || 'shared',
  cell: '',
  error: null,
});

// Which workspace this browser works in. Everything a user authors — scenes,
// applications, algorithms — is scoped to it; the shipped library shows
// through underneath, so a fresh session still sees everything that ships.
export function setSession(id) {
  const session = id?.trim() || 'shared';
  localStorage.setItem('robonode-session', session);
  store.patch({ session });
}

// Every read carries who is asking and which robot they mean.
const scoped = path => {
  const { session, cell } = store.get();
  const query = `session=${encodeURIComponent(session)}` + (cell ? `&cell=${encodeURIComponent(cell)}` : '');
  return `${path}${path.includes('?') ? '&' : '?'}${query}`;
};

// Which robot the controls drive. The dashboard follows it, so "run" means the
// machine you are looking at.
export function setCell(id) {
  store.patch({ cell: id ?? '' });
}

export async function getJSON(path) {
  const r = await fetch(scoped(path));
  if (!r.ok) throw new Error(`${path}: ${r.status}`);
  return r.json();
}

// Fire a command. Returns the ack; a rejection is surfaced, never dropped.
export async function submit(body) {
  try {
    const r = await fetch('/command', {
      method: 'POST',
      body: JSON.stringify({
        session: store.get().session,
        ...(store.get().cell ? { cell: store.get().cell } : {}),
        ...body,
      }),
    });
    const ack = await r.json();
    store.patch({ error: ack.ok ? null : ack.error ?? 'command rejected' });
    if (ack.ok) store.patch({ acceptedId: ack.id ?? store.get().acceptedId });
    return ack;
  } catch (e) {
    store.patch({ error: String(e.message ?? e) });
    return { ok: false, error: String(e) };
  }
}

// Resolve once the worker has applied `id` and the cell is idle again.
// `subscribe` reports the current value immediately, so settle before the
// subscription exists is a real case and must not depend on binding order.
function awaitApplied(id, timeoutMs = 120000) {
  if (!id) return Promise.resolve(false);
  return new Promise(resolve => {
    let unsubscribe = null;
    let settled = false;
    const finish = ok => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      if (unsubscribe) unsubscribe();
      resolve(ok);
    };
    const check = () => {
      const s = store.get();
      if (s.appliedId >= id && s.state === 'idle') finish(true);
    };
    const timer = setTimeout(() => finish(false), timeoutMs);
    unsubscribe = store.subscribe(s => `${s.appliedId}:${s.state}`, check);
    if (settled) unsubscribe();
  });
}

export async function run(body) {
  const ack = await submit(body);
  return ack.ok ? awaitApplied(ack.id) : false;
}

// Capabilities publish live state (a tracker's speed estimate), so the panel
// that shows it needs more than the one read at page load.
export function pollCapabilities(periodMs = 1000) {
  loadCapabilities();
  return setInterval(loadCapabilities, periodMs);
}

export function connect() {
  const es = new EventSource(scoped('/events'));
  es.addEventListener('nodes', e => {
    const tree = JSON.parse(e.data);
    store.patch({
      connection: 'live',
      nodes: tree.nodes ?? [],
      drivers: tree.available ?? [],
      family: tree.family ?? '',
      families: tree.families ?? [],
    });
  });
  es.addEventListener('logs', e => store.patch({ logs: JSON.parse(e.data) }));
  es.onmessage = e => {
    const f = JSON.parse(e.data);
    store.patch({
      connection: 'live',
      pos: f.pos ?? [],
      target: f.target ?? [],
      err: f.err ?? [],
      tcp: f.tcp ?? null,
      tcpQuat: f.tcp_quat ?? null,
      part: f.vision?.part ?? null,
      workpiece: f.workpiece?.pose ?? null,
      holding: !!f.workpiece?.held,
      contacts: f.contacts ?? [],
      state: f.state ?? 'idle',
      step: f.step ?? '',
      app: f.app ?? '',
      latched: !!f.latched,
      appliedId: f.applied_id ?? 0,
      acceptedId: f.accepted_id ?? 0,
      error: f.last_error ?? null,
    });
  };
  es.onerror = () => store.patch({ connection: 'reconnecting' });
  return es;
}

// The capability contract comes from the platform, never from constants here.
export async function loadCapabilities() {
  try {
    store.patch({ capabilities: await getJSON('/capabilities') });
  } catch (e) {
    store.patch({ error: String(e.message ?? e) });
  }
}

// Writes go to the caller's session, always: the shipped library is read-only.
export async function send(path, method, body) {
  try {
    const r = await fetch(scoped(path), { method, body: body == null ? undefined : body });
    const j = await r.json().catch(() => ({ ok: r.ok }));
    if (!j.ok) store.patch({ error: j.error ?? `${path}: ${r.status}` });
    else store.patch({ error: null });
    return j;
  } catch (e) {
    store.patch({ error: String(e.message ?? e) });
    return { ok: false, error: String(e) };
  }
}

export async function loadCollection(path, key) {
  try {
    store.patch({ [key]: await getJSON(path) });
  } catch (e) {
    store.patch({ error: String(e.message ?? e) });
  }
}

export const short = s => String(s).replace(/^robonode\./, '');
