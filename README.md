# RoboNode (working name)

Platform where every robot and each of its modules is a **node** you can see, control, and plug your own algorithms into — end-to-end like Vention.io, open like nothing on the market: user code runs at every tier, from cloud analytics down to the certified real-time control loop.

## Documents

| Doc | What it is |
|---|---|
| [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md) | Product requirements: vision, personas, domain model, FR-1…FR-10 with priorities, NFRs, MVP cut, risks |
| [docs/SPEC.md](docs/SPEC.md) | System spec: three-plane architecture, node/capability model, motion core, algorithm tiers A/B/C, data plane, safety, milestones M0–M4 |
| [docs/TECH-LANDSCAPE.md](docs/TECH-LANDSCAPE.md) | Researched survey of usable systems (Viam, ROS 2, Zenoh, Ruckig, EtherCAT, WASM, Gazebo, Mender, …) + recommended stack |
| [docs/research/](docs/research/) | Competitive intel on Vention (API surface + verified gaps, product specs, customers, company) — the seams RoboNode targets |

Read order: REQUIREMENTS → SPEC → TECH-LANDSCAPE.

## Code assets (seeds for the platform)

- **[trajectory-lab/](trajectory-lab/)** — C++20 motion profiles (trapezoid, S-curve), blending, lock-free SPSC streaming + tests. Seeds the motion core's blend/profile layer (SPEC §3). Rebuild: `cmake -B build && cmake --build build && ctest --test-dir build`.
- **[ur-stream-playground/](ur-stream-playground/)** — URSim + ur_client_library scaffold. Seeds the UR adapter (SPEC §3.1, milestone M1).

## Archive

`_archive/` holds the June 2026 interview-prep material this workspace grew from (guides, study notes, drafts, CV). Safe to delete once you've confirmed you need none of it.

## Next actions

1. Resolve OQ-1…OQ-5 (REQUIREMENTS §9) — hardware target and middleware bet first.
2. `git init` + first commit (workspace is not yet a repo).
3. Start M0: `robonode-idl` capability schemas + motion-core skeleton against the sim adapter.
