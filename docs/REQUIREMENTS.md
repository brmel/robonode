# RoboNode — Product Requirements Document

> **Working name:** RoboNode (placeholder — rename freely).
> **Status:** Draft v0.1 — 2026-07-11.
> **One-liner:** An end-to-end platform where every robot and each of its modules is a **node** you can see, control, and — unlike anything on the market today — **plug your own algorithms into at any level of the stack**, from cloud analytics down to the real-time control loop.

---

## 1. Vision and positioning

### 1.1 The problem

Industrial automation platforms force a trade-off today:

- **Vention** (MachineBuilder → MachineLogic → MachineMotion AI) is the gold standard for hardware-to-deployment integration, but its motion and intelligence stack is **closed**: no public streaming API, no blending on actuators, no way to run your own planner, controller, or perception model in the loop. Servo tuning is literally a warranty-voiding backdoor. If Vention's algorithm doesn't fit your process, you are stuck.
- **Viam** nailed plug-and-play modularity (registry of drivers + services, JSON config, SDKs in five languages), but it is general-purpose/prosumer-oriented: no industrial motion pedigree, no jerk-limited coordinated motion, no fieldbus story, no functional-safety story.
- **Wandelbots NOVA** is robot-agnostic and developer-friendly (Python/JS APIs, Isaac Sim integration) but is a software layer only — no actuator/module ecosystem, and user code stays outside the real-time loop.
- **ROS 2** gives you everything and guarantees nothing: it is a toolkit, not a product. Integrators burn months assembling drivers, safety, UI, fleet management, and OTA around it.

**Gap: no platform lets a customer bring their own algorithm — perception, planning, trajectory generation, force control, quality inspection — and run it safely against industrial-grade hardware with a plug-and-play experience.**

### 1.2 The product

RoboNode is a platform in three layers:

1. **Node runtime (edge)** — every physical asset (robot arm, linear axis, gripper, camera, F/T sensor, conveyor, PLC I/O) is represented by a **node** with a typed capability descriptor. Nodes compose into a **cell**. A hard-real-time motion core coordinates them (blending, synchronized multi-axis, constant-TCP-speed paths — the things Vention's public API can't do).
2. **Algorithm plug-in system** — user code runs in one of three sandboxed tiers: cloud/offline (analytics, training), edge skill-rate (perception, task logic, 10–100 Hz), and certified real-time in-loop (custom OTG, force controllers, 1 kHz). Algorithms are versioned artifacts in a registry, testable in simulation before touching hardware.
3. **Control plane (cloud/on-prem)** — fleet view of all cells and nodes, declarative configuration, low-code program editor plus full-code SDKs, telemetry/recording/replay, OTA deployment, user/role management.

### 1.3 Differentiators (why we win)

| # | Differentiator | Nearest competitor's state |
|---|---|---|
| D1 | User algorithms in the **real-time loop** (signed, sandboxed, sim-validated) | Nobody offers this; Vention closed, Viam/NOVA non-RT only |
| D2 | **Unified motion tree**: arm + axes + conveyor blended and synchronized as one kinematic chain | Vention: robot-only blending, no arm+rail blend, single-threaded event loop |
| D3 | **Simulation gate**: every algorithm/program must pass in the digital twin before deploy; sim and real expose the *same node API* | Partial in NOVA (Isaac), absent in Vention SDK |
| D4 | **Open telemetry**: every node streams typed, recordable, replayable data (MCAP), Foxglove-compatible | Vention MachineCloud is closed; Viam good but cloud-tied |
| D5 | Registry of drivers **and** algorithms with semantic versioning and fleet rollout | Viam has drivers/ML; nobody has RT algorithm artifacts |

### 1.4 What RoboNode is not (v1)

- Not a CAD/hardware-design tool (no MachineBuilder clone; import URDF/STEP instead).
- Not a hardware company: we define a **driver SDK** and certify third-party hardware; we may ship a reference controller but do not manufacture actuators.
- Not a safety controller: we integrate with certified safety PLCs/relays; we do not replace them (see FR-8).
- Not an ML training platform: we host/serve models and export data; training happens in the customer's MLOps stack.

---

## 2. Personas

| Persona | Description | Primary needs |
|---|---|---|
| **P1 Automation engineer** (integrator/OEM) | Builds cells for end customers; PLC background, some Python | Fast commissioning, no-code programs, reliable diagnostics, doesn't want to see middleware |
| **P2 Robotics software developer** | C++/Python; builds product features on top of cells | Clean SDK/APIs, local dev loop with sim, logs/replay, CI integration |
| **P3 Algorithm engineer / researcher** | Controls, perception, or ML specialist | Bring-your-own algorithm with a stable contract, benchmark harness, data in/out, no robot-vendor lore required |
| **P4 Machine operator** | Runs production; no programming | One-screen cell status, start/stop/recover, guided fault recovery, e-stop confidence |
| **P5 Production/ops manager** | Owns OEE and uptime across sites | Fleet dashboard, alerts, utilization/quality analytics, audit trail |
| **P6 Hardware partner** | Actuator/gripper/sensor vendor | Driver SDK + certification path to appear "plug-and-play" on the platform |

---

## 3. Domain model (ubiquitous language)

- **Node** — the digital representation of one physical (or simulated) asset with identity, lifecycle state, capability descriptor, telemetry streams, and command interface. A robot arm is a node; so is each module.
- **Module** — a node attached beneath another node in the **node tree** (e.g., gripper on arm flange, camera on wrist, F/T sensor between flange and tool). Modules can be re-parented (hot-plug).
- **Capability** — a typed interface a node implements (`MotionAxis`, `ArmKinematics`, `Gripper`, `Camera`, `ForceTorque`, `DigitalIO`, `Conveyor`, …). One node may expose several.
- **Cell** — a set of nodes coordinated by one **cell controller** (one physical location, one safety domain).
- **Fleet** — all cells of an organization across sites.
- **Driver** — software that binds a vendor device to a node capability (runs in the node runtime).
- **Algorithm** — user-supplied code artifact with a declared tier, contract, resources, and version (see FR-3).
- **Program** — orchestration of nodes + algorithms into a task (behavior-tree or flow representation; Python escape hatch).
- **Recipe** — parameterization of a program for a specific product/variant.
- **Twin** — the simulated counterpart of a node/cell exposing the same API.

---

## 4. Functional requirements

Priorities: **[M]** must (MVP), **[S]** should (v1.x), **[C]** could (v2+).

### FR-1 Node & module management

- **FR-1.1 [M]** Declarative cell configuration: a versioned config (file + UI) declares nodes, their drivers, parameters, and tree topology. Applying a config converges the cell to it (create/configure/remove nodes) without reflashing.
- **FR-1.2 [M]** Node lifecycle: `unconfigured → configuring → inactive → active → degraded → fault`, with explicit transitions, timestamps, and reasons exposed on the API and UI.
- **FR-1.3 [M]** Capability descriptors: every node publishes a machine-readable descriptor (capabilities, units, limits: position/velocity/acceleration/jerk/force, command rates, supported blend types). Limits are **data, not constants** — e.g., belt vs ball-screw actuators differ; consumers (planners, UIs) must read them at runtime.
- **FR-1.4 [M]** Discovery: USB/Ethernet/EtherCAT scan proposes detected devices with matching drivers ("found UR10e at 192.168.1.5 — add as node?").
- **FR-1.5 [S]** Hot-plug modules: attaching a known gripper/camera while the cell is `inactive` auto-creates its node under the right parent; removal degrades gracefully with a clear fault.
- **FR-1.6 [M]** Node tree kinematics: parent-child transforms (static or from calibration) are first-class; the full chain (rail → arm → F/T → tool) is queryable as one kinematic model (URDF-compatible export).
- **FR-1.7 [S]** Calibration workflows: guided tool-center-point, hand-eye (AX=XB), and axis-to-arm frame calibration wizards storing results in the node tree with provenance.
- **FR-1.8 [C]** Multi-vendor parity matrix published per driver: which capabilities are full/partial/absent per device model.

### FR-2 Control & motion

- **FR-2.1 [M]** Manual control: per-node jog (joint and Cartesian for arms; velocity mode for axes/conveyors) from the web UI with speed override 0–100 %, deadman semantics for browsers (heartbeat; loss ⇒ controlled stop).
- **FR-2.2 [M]** Point-to-point moves with per-move motion profile (velocity, acceleration, **jerk honored, not decorative**) on every `MotionAxis` and arm node.
- **FR-2.3 [M]** Waypoint sequences with **blending on all node types** — arms (`movej/movel` + blend radius), actuators, and mixed chains. This closes Vention's most visible gap.
- **FR-2.4 [M]** Synchronized multi-node motion: N axes + arm coordinated by one clock master; time-parameterized so all reach waypoints simultaneously (gantry squaring, 7th-axis + arm).
- **FR-2.5 [S]** Constant-TCP-speed Cartesian paths across the full chain (arm + rail) with curvature- and limit-aware retiming (TOPP-style lookahead), for process work (dispensing, sanding, welding).
- **FR-2.6 [M]** Streaming setpoint interface: external or plugged-in code can retarget motion online at the node's command rate (e.g., 500 Hz UR servoj, 125 Hz+ generic), mediated by an online trajectory generator so streams are always jerk-limited and limit-respecting. **This is the "plug your algorithm" primitive.**
- **FR-2.7 [M]** Stop semantics: `pause` (decelerate on path, resumable), `stop` (decelerate, discard program state), `protective stop` (category 2), `e-stop` (category 0/1, hardware-backed). Path-stop preferred over trajectory-abort wherever supported. Every state reachable from every motion state.
- **FR-2.8 [S]** Force-controlled skills on supported hardware: admittance control wrapped around position interfaces (UR/Fanuc class), hybrid force/position with selection matrix; gravity compensation calibration for tools.
- **FR-2.9 [C]** Collision-aware planning service: PRM/RRT-Connect against the cell twin, with attach/detach of carried objects; speed scaling near humans per ISO/TS 15066 when safety sensors present.
- **FR-2.10 [M]** All motion commands idempotent + acknowledged with command IDs; absolute setpoints on the wire so one lost packet self-heals.

### FR-3 Algorithm plug-in system (the differentiator)

- **FR-3.1 [M]** Three execution tiers with explicit contracts:
  - **Tier A — Cloud/offline**: containers; consume recorded/streamed telemetry, produce reports, models, parameters. No latency guarantee.
  - **Tier B — Edge skill**: sandboxed (WASM component or container) on the cell controller; subscribe to node topics, call node commands; 10–100 Hz soft real-time; CPU/memory quotas.
  - **Tier C — Real-time in-loop**: native plugin (fixed C ABI / C++ interface) loaded by the motion core; runs in the 1 kHz control cycle; may implement `TrajectoryGenerator`, `SetpointFilter`, or `ForceController` interfaces. Watchdogged: overrun or NaN output ⇒ automatic fallback to platform OTG + degraded state.
- **FR-3.2 [M]** Algorithm artifact = code + manifest (tier, interfaces, node-capability requirements, parameters schema, resource limits, semver). Signed; registry stores provenance.
- **FR-3.3 [M]** **Simulation gate**: an algorithm cannot be deployed to a physical cell until its test suite passes against the cell's twin (same API, recorded-data replay + synthetic scenarios). Override requires an explicit, audited "I understand" by an admin.
- **FR-3.4 [M]** Local dev loop: `robonode dev` runs a twin locally, hot-reloads the algorithm, streams the same telemetry/UI as production. Time-to-first-moving-sim-robot from SDK install: **< 15 minutes**.
- **FR-3.5 [S]** Benchmark harness: standard scenarios per interface (e.g., OTG stress: retarget storms, limit sweeps; force control: stiffness ladder) with scored reports, comparable across versions.
- **FR-3.6 [S]** Registry & marketplace: publish privately (org) or publicly; consumers install by reference; fleet-wide staged rollout (canary cell → all) and rollback.
- **FR-3.7 [C]** Certification program for Tier C artifacts (static analysis, WCET budget evidence, fault-injection results) — badge in registry.
- **FR-3.8 [M]** Data access API for algorithms: typed access to any subscribed node stream, cell kinematic model, and parameter store; no raw device access ever (capability-mediated only).

### FR-4 Programming surface

- **FR-4.1 [M]** Low-code editor: behavior-tree/flow canvas (sequences, parallels, conditions, retries, error branches) whose nodes are node-commands, algorithm invocations, and operator prompts. Compiles to the same API the SDKs use — **no capability cliff between no-code and code**.
- **FR-4.2 [M]** Python SDK (first), then C++ [S] and TypeScript [S]; all generated from the same IDL, feature-complete vs the wire API.
- **FR-4.3 [M]** REST + gRPC + WebSocket north APIs; everything the UI does is doable via API (UI is just a client).
- **FR-4.4 [S]** Program versioning with diff view; run history linked to telemetry recordings (every production run is replayable).
- **FR-4.5 [C]** AI assistant: natural-language → program skeleton against the cell's actual capability descriptors (never invents capabilities).

### FR-5 Simulation & digital twin

- **FR-5.1 [M]** Every cell has a twin: physics + kinematics from the node tree (URDF/meshes), spawnable on the cell controller or a dev machine; exposes byte-identical node APIs.
- **FR-5.2 [M]** Sim/real parity tests: platform CI runs the same motion test suite against twin and reference rigs; parity report per release.
- **FR-5.3 [S]** Sensor simulation adequate for Tier B algorithm testing: cameras (RGB/depth, basic noise), F/T signals, conveyor motion.
- **FR-5.4 [C]** Photoreal/synthetic-data pipeline via Isaac Sim connector for perception training.
- **FR-5.5 [S]** "Shadow mode": run a candidate algorithm against live telemetry in parallel without actuating; divergence report vs incumbent.

### FR-6 Telemetry, observability & data

- **FR-6.1 [M]** Every node publishes typed telemetry (state, setpoints vs actuals, following error, temperatures, faults) at capability-defined rates; uniform timestamping (PTP-synced on the cell; one clock domain).
- **FR-6.2 [M]** Ring-buffer flight recorder on the cell: last N minutes of full-rate data always recorded; fault ⇒ automatic incident bundle (data + config + versions).
- **FR-6.3 [M]** Recording/replay in MCAP; Foxglove-compatible live streams and files.
- **FR-6.4 [S]** Cloud sync with store-and-forward (offline-first; bandwidth budgets; downsampling policies per stream).
- **FR-6.5 [S]** Dashboards: OEE, cycle time, per-node health trends, alert rules (threshold + anomaly [C]).
- **FR-6.6 [M]** Structured audit log: who deployed/changed/ran what, when, on which cell.

### FR-7 Fleet & deployment

- **FR-7.1 [M]** Cloud (or on-prem) control plane: org → sites → cells → nodes hierarchy; live status; remote shell disabled by default (support-mode with consent + audit).
- **FR-7.2 [M]** OTA updates: A/B atomic updates of the cell controller OS/runtime; per-component container updates for drivers/algorithms; staged rollouts; automatic rollback on boot/health failure.
- **FR-7.3 [S]** Config fragments: reusable config blocks (e.g., "UR10e on 7th axis") instantiated with per-cell variables — Viam-style fragments for industrial cells.
- **FR-7.4 [S]** Cell provisioning: flash reference image → device claims into org via one-time token → config applied → cell live. Target < 30 min from unboxing.
- **FR-7.5 [C]** Multi-cell orchestration (line-level coordination, inter-cell handoff) — explicitly out of MVP.

### FR-8 Safety (constraint on everything above)

- **FR-8.1 [M]** Hardware safety chain is authoritative: e-stop, safeguard stops, and safety-rated monitored stop run through a certified safety controller (integrated relay/PLC, e.g., Cat 3 / PL d–e per ISO 13849); the platform *observes* safety state, never implements the safety function in software.
- **FR-8.2 [M]** Safety state is first-class on every node (`NORMAL / REDUCED / PROTECTIVE_STOP / ESTOP / FAULT`), propagated to all consumers within one control cycle on-cell and ≤ 250 ms to UIs; UI shows cell-wide safety banner.
- **FR-8.3 [M]** Software stop categories mapped per IEC 60204 (Cat 0/1/2) and exercised in tests; e-stop reaction (command to drive disable signal) ≤ 10 ms on the RT path.
- **FR-8.4 [M]** Algorithm tiers constrained by safety: Tier B/C outputs pass through limit governors (position/velocity/torque envelopes, workspace fences) enforced below the plugin boundary; governors are platform code, not user code.
- **FR-8.5 [S]** Risk-assessment artifacts: per-cell safety configuration report (limits, fences, stop categories, test dates) exportable for the integrator's ISO 10218 file.
- **FR-8.6 [M]** Simulation and reduced-speed commissioning modes (T1-style ≤ 250 mm/s TCP) enforced platform-side.

### FR-9 Enterprise integration (north-side)

- **FR-9.1 [S]** OPC UA server per cell (address space mirrors the node tree; state/telemetry/recipe tags) for MES/SCADA.
- **FR-9.2 [S]** MQTT Sparkplug B publisher option for UNS-style factories.
- **FR-9.3 [C]** Webhooks + event bus for line events (job done, fault, part result).
- **FR-9.4 [M]** Everything above read-mostly; writes (start/stop/recipe select) require mapped, authenticated roles.

### FR-10 Identity, tenancy & administration

- **FR-10.1 [M]** Multi-tenant org model; SSO (OIDC) [S]; local accounts fallback [M].
- **FR-10.2 [M]** RBAC: at minimum Operator / Engineer / Admin / Viewer, scoped per site/cell; jog+deploy privileges separated from view.
- **FR-10.3 [M]** Device identity: per-controller certificates (mTLS); no shared secrets; offline grace.
- **FR-10.4 [S]** API tokens with scopes + expiry for CI and integrations.

---

## 5. Non-functional requirements

| ID | Requirement | Target |
|---|---|---|
| NFR-1 | RT control cycle on cell controller | 1 kHz base rate; cycle jitter p99.9 < 100 µs (PREEMPT_RT) |
| NFR-2 | E-stop software observation → drive-disable command | ≤ 10 ms (hardware chain independently faster) |
| NFR-3 | Streaming command path (Tier B → governor → drive setpoint) | ≤ 20 ms p99 |
| NFR-4 | UI live telemetry latency (cell → browser, LAN) | ≤ 150 ms p95 |
| NFR-5 | Cell autonomy | Fully operational with cloud unreachable indefinitely; cloud is management, never in the control path |
| NFR-6 | Availability of control plane | 99.9 % monthly; cell keeps running regardless |
| NFR-7 | Scale | v1: 50 nodes/cell, 200 cells/org; UI fleet views paginate accordingly |
| NFR-8 | Security | Signed OS/driver/algorithm artifacts (SBOM per release); sandbox escape = P0; TLS everywhere; CIS-hardened controller image |
| NFR-9 | Portability | Controller runtime on arm64 + x86_64 Linux (Jetson-class and industrial PC) |
| NFR-10 | Data integrity | Flight recorder survives power loss (journaled); incident bundles complete or clearly marked partial |
| NFR-11 | Docs | Every public API/interface documented with runnable example; docs CI-tested |
| NFR-12 | Licensing posture | Core protocols + SDKs + driver interface open (Apache-2.0) to drive ecosystem; motion core and cloud proprietary (open-core) — *decision pending, see OQ-3* |

---

## 6. MVP definition (v0 — "one cell, one arm, one axis, one algorithm")

**Scenario to demo end-to-end:** a UR arm (URSim acceptable) on a simulated 7th axis; user writes a Python Tier B algorithm that visually servos a target (fake detector acceptable), streams retargets through the OTG, with blended arm+axis waypoint moves, jog UI, flight recorder, replay in Foxglove, deployed via registry after passing the sim gate.

Included: FR-1.1–1.4, 1.6; FR-2.1–2.4, 2.6, 2.7, 2.10; FR-3.1 (A: stub, B: full, C: interface defined + reference impl only), 3.2–3.4, 3.8; FR-4.1 (minimal), 4.2 (Python), 4.3; FR-5.1, 5.2; FR-6.1–6.3, 6.6; FR-7.1, 7.2 (single-cell); FR-8.1–8.4, 8.6; FR-10.1–10.3.

Explicitly deferred: force control, collision-aware planning, marketplace, OPC UA/Sparkplug, multi-cell, Isaac connector, certification program.

## 7. Success metrics

- TTFM (time to first motion, sim): < 15 min from SDK install (P3).
- TTFC (time to first cell commissioned, hardware): < 1 day for P1 with certified hardware.
- Algorithm iteration loop (edit → sim-validated → on hardware): < 10 min.
- ≥ 3 external Tier B algorithms and ≥ 1 external driver by end of v1 pilot.
- Zero safety-governor bypasses in the field (audited).

## 8. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| Tier C (user code @ 1 kHz) is hard to make safe | Flagship feature slips | Ship interface + first-party reference plugins first; governors + watchdog architecture from day 0; certification later |
| Vendor protocol breadth (UR/Fanuc/ABB/EtherCAT…) eats all engineering | MVP death by drivers | MVP = UR + one EtherCAT axis only; driver SDK so partners carry breadth |
| Safety perception ("user code near robots?!") | Sales blocker | Hardware chain authoritative + governors below plugin boundary + sim gate; publish the architecture |
| Viam/NOVA move down/up into this niche | Differentiation erodes | Speed on D1/D2 (RT plug-in + unified motion tree) — hardest to copy |
| Open-core boundary wrong | Ecosystem stillborn or IP given away | OQ-3 decision with counsel before public launch |

## 9. Open questions

Decisions recorded in [DECISIONS.md](DECISIONS.md).

- **OQ-1**: ~~Reference controller hardware~~ → **resolved, ADR-1**: industrial x86_64 + PREEMPT_RT first; Jetson variant v1.x.
- **OQ-2**: ~~ROS 2 core vs Zenoh-native~~ → **resolved, ADR-2**: Zenoh-native, ROS 2 bridged.
- **OQ-3**: Open-core license boundary (NFR-12) — **open**, needs counsel; nothing published until decided.
- **OQ-4**: ~~Tier B sandbox packaging~~ → **resolved, ADR-3**: WASM components + OCI, one contract.
- **OQ-5**: ~~Safety-PLC certification in MVP?~~ → **resolved, ADR-4**: reference wiring + report generator in MVP; partner program v1.x.

## 10. Glossary

See §3. Additional: **OTG** = online trajectory generation (Ruckig-style, per-cycle retargetable); **TOPP** = time-optimal path parameterization; **governor** = platform-enforced limit envelope below the plugin boundary; **MCAP** = container file format for multimodal robotics logs; **UNS** = unified namespace.
