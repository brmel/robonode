// Browser e2e — the same journey a human (or an agent via Playwright MCP)
// walks in the live web app, pinned as a repeatable regression. Proves the
// full loop: browser → HTTP/SSE gateway → celld → motion → MuJoCo physics →
// telemetry → UI. Run against a live cell_server (docker compose up, or the
// binary with ROBONODE_WORLDS_DIR set); base URL from E2E_BASE_URL.
import { test, expect } from '@playwright/test';

test.describe('RoboNode live cell', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('boots the 7-node cell under the physics driver', async ({ page }) => {
    await expect(page.getByRole('heading', { name: 'RoboNode — live cell' })).toBeVisible();
    const rows = page.locator('table tbody tr');
    await expect(rows).toHaveCount(7);
    await expect(rows.first()).toContainText('rail-x');
    // Every node advertises the three swappable driver versions (forced interface).
    const firstDriver = rows.first().locator('select');
    await expect(firstDriver.locator('option')).toHaveText([
      'mujoco-axis',
      'sim-axis',
      'sim-axis-soft',
    ]);
  });

  test('Run streams a coordinated move — telemetry leaves home', async ({ page }) => {
    const railOut = page.locator('table tbody tr', { hasText: 'rail-x' }).locator('td').last();
    await expect(railOut).toHaveText('0 mm');
    await page.getByRole('button', { name: /Run coordinated move/ }).click();
    // Rail traverses to its final waypoint (400 mm) under real physics.
    await expect(railOut).toContainText('400 mm', { timeout: 20_000 });
  });

  test('per-node driver swap is live (try each version)', async ({ page }) => {
    const j3 = page.locator('table tbody tr', { hasText: 'j3' });
    await j3.locator('select').selectOption('sim-axis');
    // The published node tree reflects the swap (SSE round-trip).
    await expect(j3.locator('select')).toHaveValue('sim-axis');
  });

  test('no console errors during the session', async ({ page }) => {
    const errors: string[] = [];
    page.on('console', (m) => m.type() === 'error' && errors.push(m.text()));
    await page.getByRole('button', { name: /Run coordinated move/ }).click();
    await page.waitForTimeout(2_000);
    expect(errors).toEqual([]);
  });
});
