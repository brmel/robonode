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

## Components (M0 in progress)

- **[robonode-idl/](robonode-idl/)** — contract source of truth: capability protos (`MotionAxis@1`, descriptors, `CellClient` Tier B surface), topic scheme, canonical descriptor examples.
- **[motion-core/](motion-core/)** — RT-domain skeleton: 1 kHz executive with absolute deadlines, governor (limits-as-data), sim axis adapter, trajlib-backed motion plans. `cmake -B build && cmake --build build && ./build/robonode_dev`.

## Code seeds

- **[trajectory-lab/](trajectory-lab/)** — C++20 motion profiles (trapezoid, S-curve), blending, lock-free SPSC streaming + tests. Feeds motion-core's profile layer (SPEC §3). Rebuild: `cmake -B build && cmake --build build && ctest --test-dir build`.
- **[ur-stream-playground/](ur-stream-playground/)** — URSim + ur_client_library scaffold. Seeds the UR adapter (SPEC §3.1, milestone M1).

## Archive

`_archive/` holds the June 2026 interview-prep material this workspace grew from (guides, study notes, drafts, CV). Safe to delete once you've confirmed you need none of it.

## Status & next

M0 done: IDL v0 + motion-core skeleton verified (see [docs/audit/2026-07-12-journey-audit.md](docs/audit/2026-07-12-journey-audit.md) for the full entry-point audit). OQ-1/2/4/5 decided ([DECISIONS](docs/DECISIONS.md)); OQ-3 (license) open.

Next — **M1** (SPEC §12): UR adapter against URSim, EtherCAT bench axis, blended arm+axis sequences, flight recorder + Foxglove live.
