import { defineConfig, devices } from '@playwright/test';

// Base URL points at a running cell_server. In CI: `docker compose up -d`
// (or run the binary with ROBONODE_WORLDS_DIR set), then `npx playwright test`.
export default defineConfig({
  testDir: './e2e',
  timeout: 60_000,
  expect: { timeout: 10_000 },
  fullyParallel: false, // one shared cell/world at a time
  reporter: [['list'], ['html', { open: 'never', outputFolder: 'e2e/report' }]],
  use: {
    baseURL: process.env.E2E_BASE_URL || 'http://localhost:8080',
    trace: 'on-first-retry',
    screenshot: 'only-on-failure',
  },
  projects: [{ name: 'chromium', use: { ...devices['Desktop Chrome'] } }],
});
