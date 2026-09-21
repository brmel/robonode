// The wire contract, checked instead of asserted. Every surface (web, CLI, SDK)
// binds to these payloads, so a schema that drifts from the server is the bug
// ADR-8 exists to prevent. Schemas live in contracts/.
import { test, expect, request } from '@playwright/test';
import Ajv from 'ajv/dist/2020';  // the schemas declare draft 2020-12
import { readFileSync } from 'node:fs';
import { join } from 'node:path';

const ajv = new Ajv({ allErrors: true, strict: false });
const compiled = new Map<string, ReturnType<typeof ajv.compile>>();

// Compiled once per schema: ajv rejects a second registration of the same $id,
// and more than one test validates the same contract.
const schema = (name: string) => {
  const cached = compiled.get(name);
  if (cached) return cached;
  const validate = ajv.compile(
    JSON.parse(readFileSync(join('contracts', `${name}.schema.json`), 'utf8')),
  );
  compiled.set(name, validate);
  return validate;
};

const check = (validate: ReturnType<typeof ajv.compile>, payload: unknown) => {
  const ok = validate(payload);
  expect(ok, ajv.errorsText(validate.errors ?? [], { separator: '\n' })).toBe(true);
};

test.describe('wire contract', () => {
  test('telemetry matches its schema', async ({ baseURL }) => {
    const api = await request.newContext({ baseURL });
    check(schema('telemetry'), await (await api.get('/telemetry')).json());
  });

  test('node tree matches its schema', async ({ baseURL }) => {
    const api = await request.newContext({ baseURL });
    check(schema('nodes'), await (await api.get('/nodes')).json());
  });

  test('capabilities match their schema', async ({ baseURL }) => {
    const api = await request.newContext({ baseURL });
    const caps = await (await api.get('/capabilities')).json();
    check(schema('capabilities'), caps);
    // Every declared capability is individually addressable.
    for (const c of caps) {
      const one = await (await api.get(`/capabilities/${c.id}`)).json();
      expect(one.id).toBe(c.id);
    }
  });

  test('the kinematic chain matches its schema and binds to the node tree', async ({ baseURL }) => {
    const api = await request.newContext({ baseURL });
    const model = await (await api.get('/model')).json();
    check(schema('model'), model);

    // Every driven axis names a joint that exists in the chain: that name is
    // what lets the viewer pose the model without assuming an ordering.
    const jointNames = new Set(model.links.map((l: { joint: string }) => l.joint).filter(Boolean));
    const nodes = (await (await api.get('/nodes')).json()).nodes as { joint: string }[];
    for (const n of nodes) expect(jointNames).toContain(n.joint);
    // The model carries the geometry, so the viewer invents none of it.
    const geoms = model.links.flatMap((l: { geoms: unknown[] }) => l.geoms);
    expect(geoms.some((g: { type: string }) => g.type === 'mesh')).toBe(true);
    expect(geoms.some((g: { type: string }) => g.type === 'box')).toBe(true);
  });

  // The scene descriptor is a contract too: it is what a user authors, so a
  // shipped scene that no longer validates is a breaking change for everyone
  // who forked one.
  test('every scene in the library matches the scene schema', async ({ baseURL }) => {
    const api = await request.newContext({ baseURL });
    const scenes = (await (await api.get('/scenes')).json()) as { file: string }[];
    expect(scenes.length).toBeGreaterThan(0);
    const validate = schema('scene');
    for (const s of scenes) check(validate, await (await api.get(`/scenes/${s.file}`)).json());
  });

  test('command acks match their schema — accepted and refused', async ({ baseURL }) => {
    const api = await request.newContext({ baseURL });
    const validate = schema('command-ack');
    // `driver` is idempotent and always valid, so this exercises the accepted shape.
    const ok = await (await api.post('/command', { data: { cmd: 'driver', family: 'physics' } })).json();
    check(validate, ok);
    expect(ok.ok).toBe(true);

    const bad = await (await api.post('/command', { data: { cmd: 'driver', family: 'warp' } })).json();
    check(validate, bad);
    expect(bad.ok).toBe(false);
  });

  // Every path that accepts a command answers in the same shape — including
  // the ones that resolve a store first. A path that returns `ok` without an id
  // is unobservable, which is exactly the bug this test exists to catch.
  test('store-backed verbs acknowledge like every other command', async ({ baseURL }) => {
    const api = await request.newContext({ baseURL });
    const validate = schema('command-ack');

    const deploy = await (
      await api.post('/command', { data: { cmd: 'run_app', file: 'pick-demo.app.json' } })
    ).json();
    check(validate, deploy);
    expect(deploy.id).toBeGreaterThan(0);

    const define = await (
      await api.post('/command', {
        data: { cmd: 'define_module', capability: 'vision', name: 'contract', source: 'x; y; z' },
      })
    ).json();
    check(validate, define);
    expect(define.id).toBeGreaterThan(0);
    expect(define.version).toBe('user.contract');

    const scene = await (
      await api.post('/command', { data: { cmd: 'load_scene', file: 'conveyor-line.scene.json' } })
    ).json();
    check(validate, scene);
    expect(scene.id).toBeGreaterThan(0);

    const missing = await (
      await api.post('/command', { data: { cmd: 'run_app', file: 'no-such.app.json' } })
    ).json();
    check(validate, missing);
    expect(missing.ok).toBe(false);
  });

  // The verb table is a contract too: an editor authors against it, so a verb
  // that runs but is not published (or vice versa) breaks authoring silently.
  test('program verbs match their schema and are the ones a program may use', async ({ baseURL }) => {
    const api = await request.newContext({ baseURL });
    const verbs = await (await api.get('/verbs')).json();
    check(schema('verbs'), verbs);
    const names = new Set(verbs.map((v: { verb: string }) => v.verb));
    expect(names.has('pick')).toBe(true);

    // A program is refused at save time when it names a verb nothing can run.
    const bad = await api.put('/apps/contract-bad.app.json', {
      data: { name: 'bad', program: [{ verb: 'no-such-verb' }] },
    });
    expect((await bad.json()).ok).toBe(false);

    const good = await api.put('/apps/contract-ok.app.json', {
      data: { name: 'ok', cell: 'ur10e.cell.json', program: [{ verb: 'wait', args: { seconds: '0.1' } }] },
    });
    expect((await good.json()).ok).toBe(true);
    await api.delete('/apps/contract-ok.app.json');
  });

  // Scenes, apps and modules are one document surface: whatever a client can do
  // to one kind it can do to any, because there is one implementation.
  test('every document collection lists, reads, forks and deletes the same way', async ({ baseURL }) => {
    const api = await request.newContext({ baseURL });
    for (const kind of ['scenes', 'apps', 'modules']) {
      const list = await (await api.get(`/${kind}`)).json();
      expect(Array.isArray(list)).toBe(true);
      for (const doc of list) expect(doc).toHaveProperty('origin');
    }

    const forked = await api.post('/apps/pick-demo.app.json/fork', {
      data: { to: 'contract-fork.app.json', name: 'Forked' },
    });
    expect((await forked.json()).ok).toBe(true);
    expect((await (await api.get('/apps/contract-fork.app.json')).json()).name).toBe('Forked');
    expect((await (await api.delete('/apps/contract-fork.app.json')).json()).ok).toBe(true);
    expect((await api.get('/apps/contract-fork.app.json')).status()).toBe(404);
  });

  // The cell's state is a declared set, not free text: a surface that renders it
  // (or waits on it) is entitled to know what it can be.
  test('the reported state is one the contract declares', async ({ baseURL }) => {
    const api = await request.newContext({ baseURL });
    const telemetry = await (await api.get('/telemetry')).json();
    check(schema('telemetry'), telemetry);
    const declared = JSON.parse(
      readFileSync(join('contracts', 'telemetry.schema.json'), 'utf8'),
    ).properties.state.enum as string[];
    expect(declared).toContain(telemetry.state);
  });

  // Every cell-scoped view refuses a robot the platform does not have. The list
  // is spelled out because there is no endpoint that publishes it — but the
  // server builds all eight from one helper, so a view that answered about the
  // wrong robot would have to be written to bypass it deliberately.
  test('a live view of a robot that is not there is refused, for every view', async ({ baseURL }) => {
    const api = await request.newContext({ baseURL });
    const views = ['model', 'stations', 'capabilities', 'verbs', 'telemetry', 'nodes', 'logs',
                   'last-run'];
    for (const v of views) {
      expect((await api.get(`/${v}`)).status(), `${v} on the default cell`).toBe(200);
      expect((await api.get(`/${v}?cell=ghost`)).status(),
             `${v} on a cell that does not exist`).toBe(404);
    }
  });

  // An unknown capability is a caller error, not an empty answer.
  test('an unknown capability is refused', async ({ baseURL }) => {
    const api = await request.newContext({ baseURL });
    const res = await api.get('/capabilities/nope');
    expect(res.status()).toBe(404);
    expect((await res.json()).ok).toBe(false);
  });

  // The id in an ack is what the stream reports back: the whole point of
  // giving commands identity.
  test('an accepted id is reported as applied', async ({ baseURL }) => {
    const api = await request.newContext({ baseURL });
    const ack = await (await api.post('/command', { data: { cmd: 'driver', family: 'physics' } })).json();
    expect(ack.ok).toBe(true);
    await expect
      .poll(async () => (await (await api.get('/telemetry')).json()).applied_id, { timeout: 20_000 })
      .toBeGreaterThanOrEqual(ack.id);
  });
});
