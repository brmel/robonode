# Modularity roadmap — try each node version, bring your own

> Direction: iterate toward a system where, in the browser, a user sees the robot + every node, **tries each version of a node**, and **implements their own** against a clean input/output contract. Each iteration ends with a browser check (robot + nodes visibly working).

The contract already exists and is enforced structurally: a node is an **`AxisAdapter`** (in: `write_setpoint`; out: `read` → position/velocity/safety; lifecycle: `configure/activate/deactivate`), built by name through the **`DriverRegistry`**. celld is vendor-blind; the boundary lint keeps it that way. The roadmap makes that contract *visible and swappable per node in the UI*, and easy to extend.

## Priorities (each = one browser-verified iteration)

| # | Iteration | Why | Browser check |
|---|---|---|---|
| **1** | **Per-node driver versions** — each node advertises its available drivers + current one; swap a single node live (not just the whole cell). Add a 2nd sim variant so there's a real choice. | The core "try each version of a node" ask, at node granularity. | Swap one joint's driver; robot keeps moving; table shows the new driver. |
| **2** | **Real physics model** — load a real robot MJCF (menagerie UR10e) so MuJoCo simulates the same robot RTB plans for; retire the primitive arm. | Makes the *simulation* a real robot; aligns physics with the RTB kinematics. | Real UR10e renders + moves. |
| **3** | **Cartesian goals via RTB in the UI** — click/enter a TCP target; `RtbPlanner` (real IK) plans; the cell executes; the robot reaches it in physics. | Real planning end-to-end, visible. | Set a target; TCP arrives there. |
| **4** | **Bring-your-own node** — a documented driver template + a registered "custom" example; show the I/O contract in the UI; a user-authored driver appears in every node's version list. | The "implement your own version with clean I/O" ask. | A third, user-written driver shows up and drives a node. |
| **5** | **Node inspector** — per node, live input (setpoint) vs output (actual, following error, safety), and the capability/limits it declares. | Makes the clean I/O contract legible. | Inspect a node; watch its in/out during a move. |
| **6** | **Tier-B algorithm slot in the browser** — load a small user algorithm (WASM/py) that reads a node's output and writes its setpoint, sandboxed. | The platform's "plug your algorithm" story, per REQUIREMENTS FR-3. | A user snippet drives a node live. |

## Working method

- Small vertical slices; keep every seam (`AxisAdapter`, `DriverRegistry`, `Kinematics`, `Planner`, `CellGateway`) intact — extend, never bypass.
- `bash scripts/check-boundaries.sh` + tests + a browser screenshot each iteration.
- Commit per iteration on `UsingRealRobot`; keep CI green.

Current: iteration 1.
