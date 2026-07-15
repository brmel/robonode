# Test strategy — and how we see the system grow

> Goal: every iteration **adds** verifiable behaviour, and that growth is **visible**. The scenario matrix below is the ledger — rows flip ▶ → ✅ as capabilities land and stay green. A slice isn't done until it adds a test here.

## Layers (a pyramid, not a pile)

| Layer | What | Where | Runs in |
|---|---|---|---|
| **Unit / module** | Pure logic per module, through its public surface (no vendor internals) | `tests/*_tests.cpp` | `robonode` + `sanitizers` CI |
| **Integration** | Cross-module through a seam we own — the contract CLI + UI both bind to | `tests/gateway_integration_tests.cpp` | `mujoco` CI |
| **Service** | The Python kinematics service self-checks the mature library | `services/rtb-kinematics/test_service.py` | `rtb-service` CI |
| **E2E / browser** | The real user (or agent) journey in the live web app | `e2e/*.spec.ts` (Playwright) | `e2e` CI (+ Playwright MCP live) |
| **CLI e2e** | Same journeys headless, `--json` — agent parity with the UI | `robonode` CLI (#43) + robonode_cli_nodes ctest | `mujoco` CI |

**Discipline (ADR-8):** the browser e2e and the CLI e2e assert the **same journeys** — because both surfaces are thin clients of one facade, a journey that passes in one must pass in the other. That parity is the anti-divergence guard.

## Current inventory (baseline 2026-07-14)

- **8 C++ suites**: motion, profile, recorder, celld, mujoco, arm, kinematics, **gateway-integration** (new).
- **1 browser suite**: `e2e/cell.spec.ts` — 4 journeys (new).
- **1 service self-test**: rtb-kinematics FK/IK/moveL.
- Boundary lint + ASan/UBSan gates.

## Scenario matrix — the growth ledger

Each row is a user/agent journey. ✅ verified & guarded · ▶ planned (issue) · ⏸ later. **The count of ✅ rows is the headline progress metric** — it only goes up.

| # | Journey | Integration | Browser e2e | CLI e2e | Status |
|---|---|---|---|---|---|
| J1 | Boot cell → 7 nodes, driver versions listed | ✅ | ✅ | ✅ | **✅ live** |
| J2 | Run coordinated move → telemetry leaves home (physics) | ✅ | ✅ | ✅ | **✅ live** |
| J3 | Per-node driver swap, live (try each version; incl. clock-owner swap #50) | ✅ | ✅ | ✅ | **✅ live** |
| J4 | Bad command / unknown driver fails closed | ✅ | — | ✅ | **✅** |
| J5 | Physics vs Sim family toggle rebuilds the cell | ▶ | ▶ | ✅ | ▶ |
| J6 | Cartesian goal → IK → TCP arrives (in-process) | ▶#34 | ▶#22 | ▶#43 | ▶ |
| J7 | Bring-your-own node appears + drives | ✅ | ✅ | ▶#43 | **✅** |
| J8 | Node inspector: capability/limits + live in/out/Δfollow | — | ✅ | ▶#43 | **✅** |
| J9 | Vision → pose → target reached | ▶#38 | ▶#6 | ▶#43 | ▶ |
| J10 | Tier-B sandboxed algo drives a node | ▶#26 | ▶#26 | ▶#43 | ▶ |
| J11 | Real robot model (menagerie UR10e) renders + moves | ▶#20 | ▶#20 | ▶#43 | ▶ |
| J12 | Logs/traces/telemetry followable from one surface | ▶#44 | ▶#44 | ▶#44 | ▶ |
| J13 | Theme toggle flips light/dark and persists | — | ✅ | ▶#43 | **✅** |
| J14 | Dashboard overlay (robot 3D + live physics · env · camera) | — | ✅ | — | **✅** |
| J15 | Application library — deploy a ready app (Pick demo) | — | ✅ | ✅ | **✅** |

**Green: 11/15 journeys (J1–J4, J7, J8, J13, J14, J15, +CLI parity).** When you add a capability, add its integration + e2e cell here and flip its row.

## Running the layers

```sh
# Unit + integration (C++) — configure once, then:
cmake -B build && cmake --build build -j && ctest --test-dir build --output-on-failure

# Browser e2e — needs a live cell_server:
docker compose up -d                       # or: ROBONODE_WORLDS_DIR=sim-mujoco/worlds ./build/apps/cell_server/cell_server &
npm install && npx playwright install --with-deps chromium
E2E_BASE_URL=http://localhost:8080 npm run test:e2e
```

An agent can also drive the live app directly through the **Playwright MCP server** (navigate / snapshot / click / screenshot) for exploratory or ad-hoc verification — the checked-in `e2e/*.spec.ts` is the same journey pinned for CI.

## How growth stays honest

- **Every issue ships a test** in the layer it belongs to; PRs that add behaviour without a matrix row are incomplete.
- **CI is the gate**: unit+integration (`ctest`) and browser (`playwright`) must be green to merge.
- **The matrix is reviewed each iteration** — ▶ → ✅ is the unit of progress, and ✅ rows never regress (a red row blocks release).
