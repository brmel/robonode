# RoboNode (working name)

Platform where every robot and each of its modules is a **node** you can see, control, and plug your own algorithms into — end-to-end like Vention.io, open like nothing on the market: user code runs at every tier, from cloud analytics down to the certified real-time control loop.

## Documents

| Doc | What it is |
|---|---|
| [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md) | Product requirements: vision, personas, domain model, FR-1…FR-10 with priorities, NFRs, MVP cut, risks |
| [docs/SPEC.md](docs/SPEC.md) | System spec: three-plane architecture, node/capability model, motion core, algorithm tiers A/B/C, data plane, safety, milestones M0–M4 |
| [docs/TECH-LANDSCAPE.md](docs/TECH-LANDSCAPE.md) | Researched survey of usable systems (Viam, ROS 2, Zenoh, Ruckig, EtherCAT, WASM, Gazebo, Mender, …) + recommended stack |
| [docs/STACK.md](docs/STACK.md) | **Exact pins**: repo + tag + license per dependency, build-verified status, and the modularity seam each one hides behind |
| [docs/DECISIONS.md](docs/DECISIONS.md) | ADR-1…4: x86+PREEMPT_RT reference hardware, Zenoh-native data plane, dual Tier B sandbox, MVP safety posture |
| [docs/research/](docs/research/) | Competitive intel on Vention (API surface + verified gaps, product specs, customers, company) — the seams RoboNode targets |

Read order: REQUIREMENTS → SPEC → TECH-LANDSCAPE → DECISIONS.

## Real robots (branch `UsingRealRobot`)

Real robot kinematics from a mature library, behind our seams — no hand-rolled math. [petercorke/robotics-toolbox-python](https://github.com/petercorke/robotics-toolbox-python) (real models: UR3/5/10, Panda, …; validated FK/IK/Jacobian) runs as a small Python service ([services/rtb-kinematics/](services/rtb-kinematics/)); the C++ [bridges/rtb/](bridges/rtb/) implements the motion `Kinematics`/`Planner` seams against it. celld, the executive, and the adapters are unchanged. `scripts/rtb-verify.sh` plans a real UR10 Cartesian moveL end-to-end. Details + the real-physics-model plan: [docs/REAL-ROBOTS.md](docs/REAL-ROBOTS.md).

## Live web app (see it, drive it)

```sh
docker compose up --build          # → http://localhost:8080   (reproducible, no host toolchain)
# or locally:
cmake -B build && cmake --build build -j --target cell_server
./build/apps/cell_server/cell_server   # → http://localhost:8080
```

A 3D UR10e-on-a-rail you watch move in real time: the node table lists all 7 nodes with their live driver + position; **Run** streams a coordinated blended move from MuJoCo physics; the **Physics / Sim** toggle rebuilds every node through the `DriverRegistry` under a different driver — both satisfy `AxisAdapter`, so a node's implementation swaps live while the UI and motion code stay untouched (the forced interface). Transport is a thin HTTP+SSE gateway ([gateway/](gateway/), [apps/cell_server/](apps/cell_server/)); the roadmap Zenoh/gRPC surface swaps in behind the same `CellGateway` seam.

## The package (one build, MIL-style modules)

```sh
cmake -B build && cmake --build build -j && ctest --test-dir build
./build/apps/robonode_dev/robonode_dev          # sim demos + MCAP recording
bash scripts/check-boundaries.sh                # module-boundary lint (also in CI)
```

| Component | What it is |
|---|---|
| [core/](core/) → `robonode::core` | Type vocabulary every module speaks: `State`, `AxisLimits`/`MotionProfile`, `Status` (one error model), `Lifecycle` verbs, `TelemetryRow`. Depends on nothing |
| [motion/](motion/) → `robonode::motion` | Governor (NaN-proof, limits-as-data), plans (S-curve/trapezoid; **SyncBlendPlan** multi-axis pass-through blends), **OTG slot** (`SetpointSource`: Ruckig `Otg` retargetable mid-flight = FR-2.6, plans, later Tier C plugins — executive can't tell them apart), executives with per-cycle safety gate, `AxisAdapter` seam. trajlib + ruckig = private impl details |
| [recorder/](recorder/) → `robonode::recorder` | Telemetry → MCAP (Foxglove-openable); owns the mcap dependency; knows only core |
| [celld/](celld/) → `robonode::celld` | Coordination plane v0: descriptor JSON → node tree → lifecycle orchestration → cell-coherent runs. Vendor-blind (lint-enforced); owns the JSON dependency. `./build/apps/celld_dev/celld_dev` boots a cell purely from [robonode-idl/examples](robonode-idl/examples/) |
| [adapters/ur/](adapters/ur/) → `robonode::adapter_ur` | UR wrist SERVOJ @500 Hz behind the seam; owns the urcl dependency; lifecycle-verified via `configure()` |
| [robonode-idl/](robonode-idl/) | Wire contracts: capability protos, topics, descriptor examples — C++ core types mirror these |
| [tests/](tests/) | Per-module test binaries — each one's include set doubles as a dependency statement |

Boundaries are structural: [scripts/check-boundaries.sh](scripts/check-boundaries.sh) fails CI on any cross-module include (core depends on nothing; recorder never sees motion; trajlib never escapes motion/).

## Code seeds

- **[trajectory-lab/](trajectory-lab/)** — C++20 motion profiles (trapezoid, S-curve), blending, lock-free SPSC streaming + tests. Feeds motion-core's profile layer (SPEC §3). Rebuild: `cmake -B build && cmake --build build && ctest --test-dir build`.
- **[ur-stream-playground/](ur-stream-playground/)** — URSim + ur_client_library scaffold. Seeds the UR adapter (SPEC §3.1, milestone M1).

## Archive

`_archive/` holds the June 2026 interview-prep material this workspace grew from (guides, study notes, drafts, CV). Safe to delete once you've confirmed you need none of it.

## Status & next

M0 done: IDL v0 + motion-core skeleton verified (see [docs/audit/2026-07-12-journey-audit.md](docs/audit/2026-07-12-journey-audit.md) for the full entry-point audit). OQ-1/2/4/5 decided ([DECISIONS](docs/DECISIONS.md)); OQ-3 (license) open.

Next — **M1** (SPEC §12): UR adapter against URSim, EtherCAT bench axis, blended arm+axis sequences, flight recorder + Foxglove live.
