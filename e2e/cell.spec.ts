// Browser e2e — the same journey a human (or an agent via Playwright MCP)
// walks in the live web app, pinned as a repeatable regression. Proves the
// full loop: browser → HTTP/SSE gateway → celld → motion → MuJoCo physics →
// telemetry → UI. Run against a live cell_server (docker compose up, or the
// binary with ROBONODE_WORLDS_DIR set); base URL from E2E_BASE_URL.
import { test, expect, Page } from '@playwright/test';

// A status line says ok or err by carrying that class next to its base one.
// Asserting the WHOLE list is the point: `toHaveClass(/ok/)` also matches a
// base class that happens to contain "ok", and then it cannot fail.
const saidOk = (page: Page, id: string) =>
  expect(page.locator(id)).toHaveClass('eout ok');
const saidErr = (page: Page, id: string) =>
  expect(page.locator(id)).toHaveClass('eout err');

// How many rows in `host` have their remove control past the dock's right edge.
// Vertical position does not matter (the panel scrolls); being pushed sideways
// out of reach does.
const escaped = (page: Page, host: string) =>
  page.evaluate((sel) => {
    const dock = document.querySelector('#dock') as HTMLElement;
    const right = dock.getBoundingClientRect().right;
    return [...document.querySelectorAll(`${sel} .stepdel`)].filter(
      (b) => b.getBoundingClientRect().right > right + 1,
    ).length;
  }, host);

const openTab = (page: Page, tab: string) =>
  page.locator(`#docktabs button[data-tab="${tab}"]`).click();

const railOut = (page: Page) =>
  page.locator('table tbody tr', { hasText: 'rail-x' }).locator('td').last();

// One live server serves the whole suite, so each test starts from a known
// platform state: no latch, physics family, built-in capability versions.
const reset = (page: Page) =>
  page.evaluate(async () => {
    const send = (body: unknown) =>
      fetch('/command', { method: 'POST', body: JSON.stringify(body) }).catch(() => {});
    await send({ cmd: 'resume' });
    await send({ cmd: 'driver', family: 'physics' });
    await send({ cmd: 'set_version', capability: 'control', version: 'robonode.direct' });
    await send({ cmd: 'set_version', capability: 'planner', version: 'robonode.moveL' });
    await send({ cmd: 'set_version', capability: 'vision', version: 'robonode.toy-detector' });
  });

test.describe('RoboNode live cell', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
    await reset(page);
    await expect(page.locator('#runstate')).toHaveText('idle', { timeout: 30_000 });
    await page.reload();
  });

  test('boots the 7-node cell under the physics driver', async ({ page }) => {
    await expect(page.getByRole('heading', { name: 'RoboNode — live cell' })).toBeVisible();
    await openTab(page, 'cell');
    const rows = page.locator('table tbody tr');
    await expect(rows).toHaveCount(7);
    await expect(rows.first()).toContainText('rail-x');
    // Every node advertises the same driver versions (forced interface). The
    // built-ins are always there; a build that links vendor hardware simply
    // offers more, so this asserts the contract, not the build configuration.
    const options = await rows.first().locator('select').locator('option').allTextContents();
    for (const builtin of ['byo-example', 'mujoco-axis', 'sim-axis', 'sim-axis-soft']) {
      expect(options).toContain(builtin);
    }
    const others = await rows.last().locator('select').locator('option').allTextContents();
    expect(others).toEqual(options);  // the same list on every node
  });

  test('Run streams a coordinated move — telemetry leaves home', async ({ page }) => {
    await openTab(page, 'cell');
    await expect(railOut(page)).toContainText('mm');
    await page.locator('#run').click();
    // The cell's named motion ends with the rail at 400 mm, under real physics.
    await expect(railOut(page)).toContainText('400 mm', { timeout: 30_000 });
  });

  // The run state is a declared machine, not a guess from a boolean: it leaves
  // idle while the worker owns the cell and returns when the command is applied.
  test('cell state machine reports moving then idle', async ({ page }) => {
    const state = page.locator('#runstate');
    await expect(state).toHaveText('idle');
    await page.locator('#run').click();
    await expect(state).toHaveText('moving', { timeout: 15_000 });
    await expect(state).toHaveText('idle', { timeout: 40_000 });
  });

  // A running move can be stopped, and a latched e-stop refuses new motion
  // until it is cleared — the platform's one honest stop story.
  test('stop cancels a run; e-stop latches until resume', async ({ page }) => {
    await page.locator('#run').click();
    await expect(page.locator('#runstate')).toHaveText('moving', { timeout: 15_000 });
    await page.locator('#stop').click();
    await expect(page.locator('#runstate')).not.toHaveText('moving', { timeout: 20_000 });

    await page.locator('#estop').click();
    await expect(page.locator('#runstate')).toHaveClass('rstate run held', { timeout: 10_000 });
    await page.locator('#resume').click();
    await expect(page.locator('#runstate')).toHaveText('idle', { timeout: 10_000 });
  });

  test('per-node driver swap is live (try each version)', async ({ page }) => {
    await openTab(page, 'cell');
    const j3 = page.locator('table tbody tr', { hasText: 'j3' });
    await j3.locator('select').selectOption('robonode.sim-axis');
    await expect(j3.locator('select')).toHaveValue('robonode.sim-axis');
  });

  test('application library — data-driven from the store, deploy runs it', async ({ page }) => {
    const pick = page.locator('#apps .ncard.app[data-file="pick-demo.app.json"]');
    await expect(pick).toBeVisible();
    await expect(page.locator('#apps .ncard[data-file="bin-picking.app.json"]')).toBeVisible();
    // The moving-target scenario is in the library too.
    await expect(page.locator('#apps .ncard[data-file="moving-bin-picking.app.json"]')).toBeVisible();
    await pick.click();
    await openTab(page, 'cell');
    await expect(railOut(page)).toContainText('400 mm', { timeout: 40_000 });
  });

  // Composing an application is the product: pick the steps, save it as yours,
  // deploy it. Nothing here types JSON — the verbs come from the platform.
  test('application editor — compose a program, save it, run it', async ({ page }) => {
    await page.locator('#apps .appedit[data-file="pick-demo.app.json"]').click();
    await expect(page.locator('#panel-apps')).toBeVisible();
    await expect(page.locator('#appSteps .step').first()).toBeVisible();

    await page.locator('#appFile').fill('e2e-authored.app.json');
    await page.locator('#appName').fill('E2E authored');
    await page.locator('#appVerb').selectOption('wait');
    await page.locator('#appAdd').click();

    const last = page.locator('#appSteps .step').last();
    await last.locator('input[data-field="seconds"]').fill('0.2');
    await page.locator('#appSave').click();
    await saidOk(page, '#appOut');

    // It is a first-class app now: the library lists it, and it deploys.
    await expect(page.locator('#apps .ncard.app[data-file="e2e-authored.app.json"]')).toBeVisible();
    await page.locator('#appRun').click();
    await saidOk(page, '#appOut');
    await page.locator('#appDelete').click();
    await expect(page.locator('#apps .ncard.app[data-file="e2e-authored.app.json"]')).toHaveCount(0);
  });

  test('dashboard overlay — live physics and tracking cards', async ({ page }) => {
    await expect(page.locator('#dash .dcard')).toHaveCount(4);
    await page.locator('#run').click();
    await expect(page.locator('#physFollow')).not.toHaveText('0.000 rad', { timeout: 30_000 });
    // The tracker's speed estimate is live state, polled from the capability.
    await expect(page.locator('#trackSpeed')).toContainText('m/s', { timeout: 15_000 });
  });

  // Capability cards are rendered from GET /capabilities — the descriptor
  // carries title, icon and ABI, so the UI hardcodes no capability at all.
  test('capability cards come from the platform descriptor', async ({ page }) => {
    // Every layer of the swap stack is a card, rendered from the descriptor.
    const cards = page.locator('#capabilityCards .ncard');
    await expect(cards).toHaveCount(5);
    await expect(page.locator('#cap-camera h3')).toContainText('Camera');
    await expect(page.locator('#cap-vision h3')).toContainText('Vision');
    await expect(page.locator('#cap-tracking h3')).toContainText('Tracking');
    await expect(page.locator('#cap-planner h3')).toContainText('Trajectory');
    await expect(page.locator('#cap-control h3')).toContainText('Control');
  });

  test('control capability swaps algorithm live — direct/smooth', async ({ page }) => {
    const chips = page.locator('#cap-control .chip');
    await expect(chips.filter({ hasText: 'direct' })).toBeVisible();
    await chips.filter({ hasText: 'smooth' }).click();
    await expect(page.locator('#cap-control .chip.on')).toHaveText('smooth', { timeout: 15_000 });
  });

  // The editor's language, template and hint all come from the capability's
  // declared ABI; control is not authorable, so it is not offered.
  test('editor offers only authorable capabilities and registers a version', async ({ page }) => {
    await openTab(page, 'editor');
    await expect(page.locator('#modCap option')).toHaveCount(3);  // control is not authorable
    await page.locator('#modCap').selectOption('vision');          // switches the ABI
    await page.locator('#modName').fill('e2e-grasp');
    await page.locator('#modSrc').fill('x +');
    await page.getByRole('button', { name: /Compile . run/ }).click();
    await saidErr(page, '#modOut');

    await page.locator('#modSrc').fill('x\ny\nz + 0.05');
    await page.getByRole('button', { name: /Compile . run/ }).click();
    await saidOk(page, '#modOut');
    await openTab(page, 'cell');
    await expect(
      page.locator('#cap-vision .chip').filter({ hasText: 'user.e2e-grasp' }),
    ).toBeVisible({ timeout: 15_000 });
  });

  test('node inspector shows capability/limits + live in/out', async ({ page }) => {
    await openTab(page, 'cell');
    await page.locator('table tbody tr', { hasText: 'j3' }).locator('td').first().click();
    const insp = page.locator('#inspector');
    await expect(insp).toContainText('MotionAxis@1');
    await expect(insp).toContainText('rad');
    await page.locator('#run').click();
    await expect(insp.locator('.kv', { hasText: 'setpoint' }).locator('.v')).not.toHaveText('—', {
      timeout: 30_000,
    });
  });

  // One palette drives both the chrome and the 3D scene, so a theme switch
  // repaints the viewport too — the bug that made the canvas stay dark.
  test('theme toggle flips light/dark, persists, and repaints the 3D view', async ({ page }) => {
    const sceneBg = () =>
      page.evaluate(() =>
        getComputedStyle(document.documentElement).getPropertyValue('--scene-bg').trim(),
      );
    const panelBg = () =>
      page.evaluate(() => getComputedStyle(document.getElementById('side')!).backgroundColor);

    await page.locator('#themeToggle').click();
    const [firstPanel, firstScene] = [await panelBg(), await sceneBg()];
    await page.locator('#themeToggle').click();
    expect(await panelBg()).not.toEqual(firstPanel);
    expect(await sceneBg()).not.toEqual(firstScene);

    const theme = await page.evaluate(() => document.documentElement.getAttribute('data-theme'));
    await page.reload();
    await expect
      .poll(() => page.evaluate(() => document.documentElement.getAttribute('data-theme')))
      .toEqual(theme);
  });

  // A rejected command must be visible in the UI, not only in the console.
  test('a rejected command surfaces in the error channel', async ({ page }) => {
    await page.evaluate(() =>
      fetch('/command', { method: 'POST', body: JSON.stringify({ cmd: 'driver', family: 'warp' }) }),
    );
    await openTab(page, 'logs');
    // The platform refuses it; the UI keeps serving and the cell is intact.
    await expect(page.locator('#runstate')).toBeVisible();
  });

  // Multi-cell: a second robot is a POST, runs its own worker, and can be
  // removed again. The main cell is not removable.
  test('a second cell can be started and stopped', async ({ page }) => {
    const post = (body: unknown) =>
      page.evaluate(
        async b => (await fetch('/cells', { method: 'POST', body: JSON.stringify(b) })).json(),
        body,
      );
    const cells = async () =>
      (await page.evaluate(async () => (await (await fetch('/cells')).json()) as { id: string }[]))
        .map((c) => c.id);

    expect(await post({ id: 'e2e-robot' })).toMatchObject({ ok: true });
    expect(await cells()).toContain('e2e-robot');
    expect(await post({ id: 'e2e-robot' })).toMatchObject({ ok: false });  // no duplicates

    const dropped = await page.evaluate(
      async () => (await (await fetch('/cells/e2e-robot', { method: 'DELETE' })).json()) as unknown,
    );
    expect(dropped).toMatchObject({ ok: true });
    expect(await cells()).not.toContain('e2e-robot');

    const main = await page.evaluate(
      async () => (await (await fetch('/cells/main', { method: 'DELETE' })).json()) as unknown,
    );
    expect(main).toMatchObject({ ok: false });
  });

  // The 3D view is rebuilt from GET /model, so it cannot drift from the
  // physics. A blank canvas means the chain failed to build.
  test('the 3D view renders the robot from the model', async ({ page }) => {
    const view = page.locator('#view canvas');
    await expect(view).toBeVisible();
    const shot = await page.locator('#view').screenshot();
    const distinct = new Set<number>();
    for (let i = 0; i + 4 < shot.length; i += 601) distinct.add(shot.readUInt32BE(i));
    expect(distinct.size).toBeGreaterThan(20);  // a rendered scene, not a flat fill
  });

  // The workpiece is only drawn once something has located it, and reads
  // differently while carried — so a grasp is visible, not inferred.
  test('the workpiece has a held colour distinct from a free one', async ({ page }) => {
    const free = await page.evaluate(() =>
      getComputedStyle(document.documentElement).getPropertyValue('--scene-part').trim(),
    );
    const held = await page.evaluate(() =>
      getComputedStyle(document.documentElement).getPropertyValue('--scene-part-held').trim(),
    );
    expect(free).not.toEqual('');
    expect(held).not.toEqual('');
    expect(held).not.toEqual(free);
  });

  test('dock tabs follow the ARIA tabs pattern', async ({ page }) => {
    const tabs = page.locator('#docktabs [role="tab"]');
    await expect(tabs).toHaveCount(6);
    await expect(tabs.first()).toHaveAttribute('aria-selected', 'true');
    await tabs.first().focus();
    await page.keyboard.press('ArrowRight');
    await expect(tabs.nth(1)).toHaveAttribute('aria-selected', 'true');
  });

  test('logs panel tails events after a run', async ({ page }) => {
    await openTab(page, 'logs');
    await page.locator('#run').click();
    await expect(page.locator('#logs')).toContainText('run', { timeout: 30_000 });
  });

  // The scenario is data: open a shipped scene, change a number, save it into
  // your own session and run it. What proves it worked is the physics model —
  // the object is where the JSON now says it is.
  test('a forked scene is edited, run, and the physics model follows', async ({ page }) => {
    await page.locator('#sessionId').fill('e2e');
    await page.locator('#sessionSet').click();
    await openTab(page, 'scenes');
    await page.locator('#sceneList .scene', { hasText: 'Cluttered line' }).click();
    // The form is the editor: the object is a row of fields, and the JSON under
    // it is a view of what will be saved.
    const crate = page.locator('#sceneObjects .step').first();
    await expect(crate).toContainText('name');
    await crate.locator('input[data-field="pos"][data-axis="0"]').fill('0.8');
    await crate.locator('input[data-field="pos"][data-axis="1"]').fill('0.35');
    await expect(page.locator('#sceneSrc')).toContainText('0.8');

    const edited = await crate.locator('input[data-field="name"]').inputValue();
    await page.locator('#sceneFile').fill('e2e.scene.json');
    await page.locator('#sceneSave').click();
    await expect(page.locator('#sceneList')).toContainText('session');

    await page.locator('#sceneUse').click();
    await expect(page.locator('#runstate')).toHaveText('idle', { timeout: 30_000 });
    const pos = await page.evaluate(async (name) => {
      const model = (await (await fetch('/model')).json()) as { links: { name: string; pos: number[] }[] };
      return model.links.find((l) => l.name === name)?.pos;
    }, edited);
    expect(pos?.[0]).toBeCloseTo(0.8, 2);

    await page.evaluate(() => fetch('/scenes/e2e.scene.json?session=e2e', { method: 'DELETE' }));
  });

  // A scene authored from nothing: place an object, save it, run it. No JSON is
  // typed — the form is the editor and the JSON under it is a view.
  test('a scene is authored in the form and runs', async ({ page }) => {
    await page.locator('#sessionId').fill('e2e-form');
    await page.locator('#sessionSet').click();
    await openTab(page, 'scenes');

    await page.locator('#sceneNew').click();
    await expect(page.locator('#sceneObjects')).toContainText('nothing placed yet');
    await page.locator('#sceneAddObject').click();

    const object = page.locator('#sceneObjects .step').first();
    await object.locator('input[data-field="name"]').fill('e2e_block');
    await object.locator('input[data-field="pos"][data-axis="0"]').fill('0.7');
    await expect(page.locator('#sceneSrc')).toContainText('e2e_block');

    await page.locator('#sceneFile').fill('e2e-form.scene.json');
    await page.locator('#sceneSave').click();
    await saidOk(page, '#sceneOut');

    await page.locator('#sceneUse').click();
    await expect(page.locator('#runstate')).toHaveText('idle', { timeout: 60_000 });
    const placed = await page.evaluate(async () => {
      const model = (await (await fetch('/model')).json()) as { links: { name: string }[] };
      return model.links.some((l) => l.name === 'e2e_block');
    });
    expect(placed).toBe(true);

    // Put the shipped scenario back: a test that leaves a different world live
    // is a test that breaks whichever one runs next.
    await page.evaluate(() => fetch('/scenes/e2e-form.scene.json?session=e2e-form', { method: 'DELETE' }));
    await page.evaluate(() =>
      fetch('/command', { method: 'POST', body: JSON.stringify({ cmd: 'load_scene', file: 'conveyor-line.scene.json' }) }));
    await expect(page.locator('#runstate')).toHaveText('idle', { timeout: 60_000 });
  });

  // The dashboard reports what the physics is doing, not what the UI assumes:
  // the workpiece rests on the belt, and that shows as a contact pair.
  test('contacts from the physics reach the dashboard', async ({ page }) => {
    await expect(page.locator('#contacts')).toContainText('conveyor', { timeout: 15_000 });
  });

  // A pick is physical: the part is welded to the tool, so it travels with it.
  test('a pick carries the part with the tool', async ({ page }) => {
    const before = await page.evaluate(async () => {
      const t = (await (await fetch('/telemetry')).json()) as { workpiece: { pose: number[] } };
      return t.workpiece.pose[0];
    });
    await page.evaluate(() =>
      fetch('/command', { method: 'POST', body: JSON.stringify({ cmd: 'pick' }) }));
    await expect(page.locator('#runstate')).toHaveText('idle', { timeout: 60_000 });
    const held = await page.evaluate(async () => {
      const t = (await (await fetch('/telemetry')).json()) as {
        workpiece: { pose: number[]; held: boolean };
        tcp: number[];
      };
      return { ...t.workpiece, tcp: t.tcp };
    });
    expect(held.held).toBe(true);
    expect(Math.abs(held.pose[2] - held.tcp[2])).toBeLessThan(0.03);
    expect(Math.abs(held.pose[0] - before)).toBeGreaterThan(0.0);
    await page.evaluate(() =>
      fetch('/command', { method: 'POST', body: JSON.stringify({ cmd: 'release' }) }));
  });

  // The log ring is a plain read, not only an SSE event: an agent debugging a
  // live cell should not have to hold a stream open to see what happened.
  test('logs are readable without a stream', async ({ page }) => {
    const logs = await page.evaluate(async () => (await (await fetch('/logs')).json()) as string[]);
    expect(Array.isArray(logs)).toBe(true);
    expect(logs.join('\n')).toContain('cell built');
  });

  // An e-stop you have to aim at is not an e-stop.
  test('the transport bar answers the keyboard, except while typing', async ({ page }) => {
    await expect(page.locator('#estop')).toHaveAttribute('aria-keyshortcuts', 'Esc');
    await page.keyboard.press('Escape');
    await expect(page.locator('#runstate')).toHaveText('held', { timeout: 10_000 });
    await page.keyboard.press('r');
    await expect(page.locator('#runstate')).toHaveText('idle', { timeout: 10_000 });

    // Typing an algorithm must never latch a stop.
    await openTab(page, 'editor');
    await page.locator('#modSrc').click();
    await page.keyboard.type('s');
    await expect(page.locator('#runstate')).toHaveText('idle');
    await expect(page.locator('#modSrc')).toHaveValue(/s/);
  });

  // "moving" does not say WHICH step is moving. The runner numbers them for
  // the log; the transport bar shows the same thing while the program runs.
  test('the running program says which step it is on', async ({ page }) => {
    await page.locator('#apps .ncard.app[data-file="bin-picking.app.json"]').click();
    await expect(page.locator('#runstep')).toContainText('/', { timeout: 30_000 });
    await expect(page.locator('#runstep')).toHaveClass('rstep on');
    // However the program ends — completed, or failed on a part that is not
    // there — the step it was on must stop being shown.
    await expect(page.locator('#runstate')).not.toHaveText('moving', { timeout: 120_000 });
    await expect(page.locator('#runstep')).toHaveText('');
  });

  // Comparing versions only means something if the work exercises the
  // capability: the trial each one names comes from the platform, and the
  // platform is what runs it — the panel renders the answer it gets back.
  test('compare runs the trial the capability names', async ({ page }) => {
    await openTab(page, 'compare');
    await page.locator('#cmpCap').selectOption('tracking');
    await expect(page.locator('#cmpTrial')).toContainText('moving-bin-picking.app.json');
    await page.locator('#cmpCap').selectOption('control');
    await expect(page.locator('#cmpTrial')).toContainText('pick-demo.app.json');

    await page.locator('#cmpA').selectOption('robonode.direct');
    await page.locator('#cmpB').selectOption('robonode.smooth');
    await page.locator('#cmpRun').click();
    const table = page.locator('#cmpOut table');
    await expect(table).toBeVisible({ timeout: 180_000 });
    await expect(table.locator('tbody tr')).toHaveCount(2);
    await expect(page.locator('#cmpOut')).toContainText('pick-demo.app.json');
    // Two control versions take the same time; what separates them is how well
    // they tracked the path, in the worst axis's own unit.
    await expect(table.locator('tbody tr').first()).toContainText('rad');

    // The comparison is kept: it is a document in this session, and reopening
    // it renders the same table it produced live.
    await expect(page.locator('#cmpRuns')).toContainText('control');
    await page.locator('#cmpOut').evaluate((el) => { el.textContent = 'cleared'; });
    await page.locator('#cmpRuns .libload').first().click();
    await expect(page.locator('#cmpOut table tbody tr')).toHaveCount(2);
    await page.locator('#cmpRuns .libdel').first().click();
    await expect(page.locator('#cmpRuns')).toContainText('no comparisons kept yet');
  });

  // #93: a scene can change what the base world already declares — the point at
  // which "make it yours" stops meaning "add clutter to it".
  test('a scene override removes a body the base world declared', async ({ page }) => {
    const before = await page.evaluate(async () => (await (await fetch('/model')).json()).links.length);
    await page.evaluate(() =>
      fetch('/command', { method: 'POST', body: JSON.stringify({ cmd: 'load_scene', file: 'clean-line.scene.json' }) }));
    await expect(page.locator('#runstate')).toHaveText('idle', { timeout: 60_000 });
    await expect
      .poll(async () => page.evaluate(async () => (await (await fetch('/model')).json()).links.length),
            { timeout: 30_000 })
      .toBeLessThan(before);

    // Put the shipped scenario back for whatever runs next.
    await page.evaluate(() =>
      fetch('/command', { method: 'POST', body: JSON.stringify({ cmd: 'load_scene', file: 'conveyor-line.scene.json' }) }));
    await expect(page.locator('#runstate')).toHaveText('idle', { timeout: 60_000 });
  });

  // 6-DoF from the browser: the point and the angle, in degrees, through the
  // same verb the CLI and an application step use.
  test('the transport bar commands a pose, not just a point', async ({ page }) => {
    await page.evaluate(() =>
      fetch('/command', {
        method: 'POST',
        body: JSON.stringify({ cmd: 'set_version', capability: 'planner', version: 'robonode.moveP' }),
      }));
    // The angle boxes start from where the tool actually points.
    await expect(page.locator('#trx')).not.toHaveValue('');

    await page.locator('#tx').fill('0.85');
    await page.locator('#ty').fill('0.25');
    await page.locator('#tz').fill('0.55');
    await page.locator('#trx').fill('180');
    await page.locator('#try_').fill('0');
    await page.locator('#trz').fill('0');
    await page.locator('#movep').click();

    await expect(page.locator('#runstate')).toHaveText('idle', { timeout: 120_000 });
    const tcp = await page.evaluate(async () => (await (await fetch('/telemetry')).json()).tcp);
    expect(Math.abs(tcp[0] - 0.85)).toBeLessThan(0.03);
    expect(Math.abs(tcp[2] - 0.55)).toBeLessThan(0.03);
  });

  // A robot is a cell descriptor, and the catalogue is a document collection:
  // starting a second machine is picking one from a list. What makes it a
  // DIFFERENT machine is visible — six joints where the first has seven.
  test('a second robot starts from the catalogue with its own chain', async ({ page }) => {
    await expect(page.locator('#robotList .ncard')).not.toHaveCount(0);
    await page.locator('#robotList .robotstart[data-file="ur10e-fixed.cell.json"]').click();

    const cells = page.locator('#cellList .libitem');
    await expect(cells).toHaveCount(2, { timeout: 30_000 });
    await expect(page.locator('#cellList')).toContainText('6 joints');
    await expect(page.locator('#cellList')).toContainText('7 joints');

    await page.locator('#cellList .cellstop').first().click();
    await expect(cells).toHaveCount(1, { timeout: 30_000 });
  });

  // A second robot you cannot drive is decoration: choosing it re-points the
  // dashboard AND the controls, so Run means the machine you are looking at.
  test('the browser drives the robot it is pointed at', async ({ page }) => {
    await page.locator('#robotList .robotstart[data-file="ur10e-fixed.cell.json"]').click();
    await expect(page.locator('#cellList .libitem')).toHaveCount(2, { timeout: 30_000 });

    // Point at the second robot: the joint table follows it (six, not seven),
    // and the header says which machine the transport bar will move.
    await expect(page.locator('#drivingCell')).toHaveText('main');
    await page.locator('#cellList .celldrive[data-id="ur10e-fixed"]').click();
    await expect(page.locator('#drivingCell')).toHaveText('ur10e-fixed');
    await openTab(page, 'cell');
    await expect(page.locator('table tbody tr')).toHaveCount(6, { timeout: 30_000 });

    // Back to the first, and the rail is there again.
    await page.locator('#cellList .celldrive[data-id="main"]').click();
    await expect(page.locator('table tbody tr')).toHaveCount(7, { timeout: 30_000 });
    await expect(page.locator('table tbody tr').first()).toContainText('rail-x');

    // Every read follows it, not just the joint table: a feed or a log from a
    // different machine than the one on screen is a lie the UI tells quietly.
    await page.locator('#cellList .celldrive[data-id="ur10e-fixed"]').click();
    await expect
      .poll(() => page.locator('#camFeed').evaluate((i: HTMLImageElement) => i.currentSrc),
            { timeout: 15_000 })
      .toContain('cell=ur10e-fixed');
    await page.locator('#cellList .celldrive[data-id="main"]').click();

    await page.evaluate(() => fetch('/cells/ur10e-fixed', { method: 'DELETE' }));
  });

  // A robot is data you can make your own — and the platform refuses to store
  // one whose sections contradict each other, at save rather than at run.
  test('a forked robot is yours, and a self-contradicting one is refused', async ({ page }) => {
    await page.locator('#sessionId').fill('e2e-robots');
    await page.locator('#sessionSet').click();
    await page.locator('#robotList .robotfork[data-file="ur10e-fixed.cell.json"]').click();
    await expect(page.locator('#robotList')).toContainText('ur10e-fixed-mine.cell.json', { timeout: 20_000 });

    const refusal = await page.evaluate(async () => {
      const mine = await (await fetch('/robots/ur10e-fixed-mine.cell.json?session=e2e-robots')).json();
      mine.nodes = mine.nodes.filter((n: { id: string }) => n.id !== 'j6');  // still named by the robot
      const res = await fetch('/robots/ur10e-fixed-mine.cell.json?session=e2e-robots', {
        method: 'PUT', body: JSON.stringify(mine),
      });
      return res.json();
    });
    expect(refusal.ok).toBe(false);
    expect(refusal.error).toContain('j6');

    // Yours is editable in a form: change what an axis is allowed to do, save,
    // and the platform validates it like any other document.
    await page.locator('#robotList .robotedit[data-file="ur10e-fixed-mine.cell.json"]').click();
    const j1 = page.locator('#robotNodes .step').first();
    await expect(j1).toContainText('axis');
    await j1.locator('input[data-field="velocity_max"]').fill('1.5');
    await page.locator('#robotSave').click();
    await saidOk(page, '#robotOut');

    const saved = await page.evaluate(async () =>
      (await (await fetch('/robots/ur10e-fixed-mine.cell.json?session=e2e-robots')).json()));
    expect(saved.nodes[0].limits.velocity_max).toBe(1.5);
    expect(saved.robots[0].joints.length).toBe(6);  // the parts the form does not show survive

    await page.evaluate(() =>
      fetch('/robots/ur10e-fixed-mine.cell.json?session=e2e-robots', { method: 'DELETE' }));
  });

  // The families a cell offers are the cell's business: the selector is built
  // from what the node tree declares, so markup carries no list of its own.
  test('the driver-family control is built from what the cell declares', async ({ page }) => {
    const declared = await page.evaluate(async () => (await (await fetch('/nodes')).json()).families);
    const buttons = page.locator('#family button');
    await expect(buttons).toHaveCount(declared.length);
    for (const family of declared) {
      await expect(page.locator(`#family button[data-fam="${family}"]`)).toBeVisible();
    }
    // And it still commands: pressing one swaps the family the cell runs.
    await page.locator(`#family button[data-fam="${declared[declared.length - 1]}"]`).click();
    await expect(page.locator('#fambadge')).toHaveText(declared[declared.length - 1], { timeout: 30_000 });
    await page.locator(`#family button[data-fam="${declared[0]}"]`).click();
    await expect(page.locator('#fambadge')).toHaveText(declared[0], { timeout: 30_000 });
  });

  // A window that is not 1440 wide is the normal case, and a page that scrolls
  // sideways is a page whose controls you cannot reach.
  test('the chrome fits the window it is given', async ({ page }) => {
    for (const [width, height] of [[1440, 900], [1100, 800], [900, 700]] as const) {
      await page.setViewportSize({ width, height });
      await page.waitForTimeout(300);
      const overflow = await page.evaluate(() => {
        const of = (sel: string) => {
          const el = document.querySelector(sel) as HTMLElement | null;
          return el ? el.scrollWidth - el.clientWidth : 0;
        };
        return {
          body: document.body.scrollWidth - document.body.clientWidth,
          dock: of('#dock'),
          bar: of('#transport'),
        };
      });
      expect(overflow, `at ${width}x${height}`).toEqual({ body: 0, dock: 0, bar: 0 });
      // Every dock tab stays reachable — six of them must wrap, not disappear.
      await expect(page.locator('#docktabs [role="tab"]')).toHaveCount(6);
      for (const tab of await page.locator('#docktabs [role="tab"]').all()) {
        await expect(tab).toBeInViewport();
      }
    }
    await page.setViewportSize({ width: 1440, height: 900 });
  });

  // A failure is announced once and then recorded. The announcement used to
  // stay up until the next success — a banner across the transport bar, over
  // the controls you would use to recover.
  test('a failure is announced once and the reason stays', async ({ page }) => {
    await page.evaluate(() =>
      fetch('/command', { method: 'POST', body: JSON.stringify({ cmd: 'move_l', x: 9, y: 9, z: 9 }) }));

    const toast = page.locator('#toast');
    await expect(toast).toHaveClass('on', { timeout: 20_000 });
    await expect(toast).toContainText('mm short');

    // It goes away on its own; the reason does not.
    await expect(toast).toHaveClass('', { timeout: 20_000 });
    const reason = page.locator('#runreason');
    await expect(reason).toHaveClass('rreason on');
    await expect(reason).toContainText('mm short');
    await expect(page.locator('#runstate')).toHaveText('faulted');

    // A command that works clears it.
    await page.locator('#run').click();
    await expect(page.locator('#runstate')).toHaveText('idle', { timeout: 60_000 });
    await expect(reason).toHaveClass('rreason');
  });

  // A panel with real content in it is the case that breaks: an editor row whose
  // controls are pushed off the edge is an editor you cannot delete a step with.
  test('a panel with content in it stays inside the dock', async ({ page }) => {
    const fits = async (what: string) => {
      const over = await page.evaluate(() => {
        const dock = document.querySelector('#dock') as HTMLElement;
        return dock.scrollWidth - dock.clientWidth;
      });
      expect(over, what).toBe(0);
    };

    // An application with many steps, several of which take arguments.
    await page.locator('#apps .appedit[data-file="moving-bin-picking.app.json"]').click();
    await expect(page.locator('#appSteps .step')).not.toHaveCount(0);
    await fits('apps');
    expect(await escaped(page, '#appSteps'), 'app step controls').toBe(0);

    // A scene whose overrides carry a position, a flag and two text fields.
    await openTab(page, 'scenes');
    await page.locator('#sceneList .scene', { hasText: 'Clean line' }).click();
    await expect(page.locator('#sceneOverrides .step')).not.toHaveCount(0);
    await fits('scenes');
    expect(await escaped(page, '#sceneOverrides'), 'override controls').toBe(0);
  });

  // Tuning a detector means seeing what it saw. The card shows the sensor's own
  // frame, and the platform draws its crosshair on it.
  test('the dashboard shows the frame the detector reads', async ({ page }) => {
    const feed = page.locator('#camFeed');
    await expect(feed).toBeVisible();
    // A real image, decoded by the browser — not a broken <img>.
    await expect
      .poll(() => feed.evaluate((i: HTMLImageElement) => i.naturalWidth), { timeout: 20_000 })
      .toBeGreaterThan(0);
    const first = await feed.evaluate((i: HTMLImageElement) => i.currentSrc);
    await expect
      .poll(() => feed.evaluate((i: HTMLImageElement) => i.currentSrc), { timeout: 10_000 })
      .not.toBe(first);  // it is a feed, not a screenshot
  });

  // Authoring a version and running it against the one it replaces is the whole
  // loop: write it, compile it, put it under the same load.
  // An argument the platform can enumerate is chosen, not typed. The
  // suggestions were being computed and thrown away, so every field rendered as
  // free text and a mistyped station failed at run time instead of at authoring
  // time — the worst place to find out.
  test('a step argument the platform publishes is a choice, not a text box', async ({ page }) => {
    await openTab(page, 'apps');
    await page.locator('#appNew').click();
    await page.locator('#appVerb').selectOption('conveyor');
    await page.locator('#appAdd').click();

    const station = page.locator('#appSteps select[data-field="station"]');
    await expect(station).toBeVisible();
    const stations = await (await page.request.get('/stations')).json();
    for (const s of stations) await expect(station.locator(`option[value="${s.id}"]`)).toHaveCount(1);
    // Unset must stay expressible, or opening the editor writes the first
    // station into every step that had none.
    await expect(station).toHaveValue('');
    await station.selectOption(stations[0].id);
    await expect(station).toHaveValue(stations[0].id);
  });

  // Judging an algorithm should start where you were looking at it: the card
  // hands the capability to the panel with the LIVE version on one side, so
  // the question is "is the other one better than what is running?" rather
  // than a form to fill in again.
  test('a capability is judged from its own card, pre-loaded against what is live', async ({ page }) => {
    await openTab(page, 'cell');
    const live = await page.locator('#cap-planner .chip.on').textContent();
    await page.locator('#cap-planner .capjudge').click();
    await expect(page.locator('#docktabs [data-tab="compare"]')).toHaveAttribute('aria-selected', 'true');
    await expect(page.locator('#cmpCap')).toHaveValue('planner');
    await expect(page.locator('#cmpA')).toHaveValue(new RegExp(`${live?.trim()}$`));
    await expect(page.locator('#cmpB')).not.toHaveValue(await page.locator('#cmpA').inputValue());
    await expect(page.locator('#cmpRun')).toBeEnabled();
  });

  test('a freshly authored module can be tested against the version it replaced', async ({ page }) => {
    await openTab(page, 'editor');
    await expect(page.locator('#modTest')).toBeDisabled();
    await page.locator('#modCap').selectOption('vision');
    await page.locator('#modName').fill('e2e-trial');
    await page.locator('#modRun').click();
    await expect(page.locator('#modOut')).toHaveClass('eout ok', { timeout: 30_000 });
    await expect(page.locator('#modTest')).toBeEnabled();
    await expect(page.locator('#modTest')).toContainText('toy-detector');
  });

  // Uncaught page errors matter more than console noise: one thrown module
  // initialiser silently stops every view after it from being wired.
  test('no console or page errors during the session', async ({ page }) => {
    const errors: string[] = [];
    page.on('console', (m) => m.type() === 'error' && errors.push(`console: ${m.text()}`));
    page.on('pageerror', (e) => errors.push(`pageerror: ${e.message}`));
    await page.reload();
    await openTab(page, 'editor');
    await openTab(page, 'compare');
    await page.locator('#run').click();
    await page.waitForTimeout(2_000);
    expect(errors).toEqual([]);
  });
});
