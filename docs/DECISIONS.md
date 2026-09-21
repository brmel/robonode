# Architecture Decision Records

Format: one entry per decision; status Accepted unless noted. Context/options live in [ARCHITECTURE.md](ARCHITECTURE.md).

## ADR-1 — Reference controller = industrial x86_64 + PREEMPT_RT (resolves OQ-1)

**Decision (2026-07-11):** v0 reference hardware is an industrial x86_64 PC running Linux PREEMPT_RT. Jetson Orin becomes a supported variant when GPU perception tiers land (v1.x).

**Why:** NFR-1 (1 kHz, p99.9 jitter < 100 µs) is the hardest constraint in the system; x86 + PREEMPT_RT is the best-documented, least-surprising path to it (IgH EtherCAT master is battle-tested there; LinuxCNC lineage). GPU is not on the MVP critical path — Tier C plugins are CPU-bound control code, and MVP perception is a fake detector (REQUIREMENTS §6). Choosing Jetson now would couple the RT budget to GPU memory-traffic interference (the exact concern raised about Vention's MMAI) before we have the tooling to measure it.

**Consequences:** motion-core CI gains an x86 PREEMPT_RT latency rig early; descriptor/limits code stays arch-neutral (arm64 build kept green per NFR-9); Isaac/cuMotion connectors remain [C]-priority.

## ADR-2 — Zenoh-native data plane, ROS 2 bridged (resolves OQ-2)

**Decision (2026-07-11):** the platform's on-cell and cell↔cloud data plane is Zenoh. ROS 2 ecosystems attach via `zenoh-plugin-ros2dds` / `rmw_zenoh` as external nodes; we do not build the product core on ROS 2.

**Why:** one protocol from RT-adjacent edge to cloud (routers, store-and-forward, queries) vs DDS-on-LAN + custom uplink glue; decouples product API from ROS distro cadence; official rmw_zenoh keeps ROS interop first-class rather than forked.

**Consequences:** we own IDL + capability schemas (`robonode-idl`) instead of inheriting ROS msgs; ROS-native pilot customers get a documented bridge recipe; revisit trigger = a pilot where >50 % of integration surface is existing ROS 2 nodes; commercial support option = ZettaScale (OQ-8 stays open).

## ADR-3 — Tier B sandbox = WASM components *and* OCI containers, one contract (resolves OQ-4)

**Decision (2026-07-11):** both packagings ship in v0 behind the identical `CellClient` gRPC surface and manifest; WASM (wasmtime, component model) is the default for logic-weight algorithms, OCI for heavy/GPU workloads.

**Why:** WASM gives capability-grant security + ms cold-start + typed interfaces (right default for "plug in my controller"); containers are unavoidable for perception stacks; making the manifest, not the packaging, the contract prevents ecosystem fragmentation.

**Consequences:** sim gate and registry treat both artifact kinds uniformly; resource quotas enforced by wasmtime limits and cgroups respectively.

## ADR-4 — Safety: reference wiring documented, no certified-partner program in MVP (resolves OQ-5)

**Decision (2026-07-11):** MVP ships a reference safety-chain design (dual-channel e-stop → safety relay Cat 3 / PL d → drive STO + UR safety inputs) with commissioning checklist and exportable safety-config report; formal partner-certification program deferred to v1.x.

**Why:** integrators (P1) already own the risk assessment under ISO 10218-2; our MVP obligation is to *observe* the chain correctly (FR-8.1–8.3) and prove governors below the plugin boundary. A certification program before pilot feedback would freeze the wrong interface.

**Consequences:** pilot contracts state the integrator holds safety responsibility; FR-8.5 report generator is MVP-scoped; safety-PLC vendor conversations (REER/Pilz/SICK class) start during pilots.

## ADR-5 — RT kinematics run in-process (Pinocchio); the Python RTB service is offline-only

**Decision (2026-07-14, from external review):** forward/inverse kinematics and Jacobians on the 1 kHz path are computed **in-process by Pinocchio (C++)** behind the existing `Kinematics` seam. The `rtb-kinematics` Python service (Robotics Toolbox) is demoted to an **offline** role: model/URDF source, one-shot IK at configuration time, and Tier-C batch planning. It is never called from the RT loop.

**Why:** two independent reviews flagged the `motion --1kHz--> rtb-kinematics(Python)` hop as the system's critical flaw — IPC round-trip (50–500 µs) plus Python's GIL and interpreter overhead cannot fit a 1 ms budget deterministically, so it breaks NFR-1. Pinocchio is the industry-standard C++ rigid-body library (Eigen-backed, compile-time-unrolled, analytic derivatives), resolving FK/IK/Jacobian in microseconds inside the motion module's memory space. The seam we already own makes this a swap, not a rewrite: RTB and Pinocchio both satisfy `Kinematics`.

**Consequences:** new dependency `pinocchio` owned by a C++ kinematics impl behind the seam (issue, P0); `engines/rtb` + `services/rtb-kinematics` stay for offline use and the real-robot model catalogue; analytic derivatives unlock gradient-based Tier-C planning (Crocoddyl). The `UsingRealRobot` thesis is unchanged — RTB still supplies validated real-robot models; Pinocchio just does the math the loop needs.

## ADR-6 — Non-RT↔RT hand-off is a lock-free SPSC queue; the 1 kHz path never locks or allocates

**Decision (2026-07-14, from external review):** all data crossing between the non-real-time `CellGateway` (commands in, telemetry out) and the real-time executive goes through **lock-free single-producer/single-consumer queues** (pre-allocated ring, `memory_order_release`/`acquire`). The RT path performs no heap allocation, no mutex, no blocking I/O. We vendor a vetted implementation (`boost::lockfree::spsc_queue`) rather than hand-rolling.

**Why:** a mutex shared between the gateway thread and the motion thread admits priority inversion — if the OS pre-empts the gateway while it holds the lock, the 1 kHz loop blocks and misses its deadline. The reviews call SPSC the standard remedy and explicitly warn against hand-rolling (cache-coherence, false sharing, memory-reordering hazards).

**Consequences:** `CellGateway` gains a command-ingress and a telemetry-egress SPSC (issue, P0); the executive polls wait-free (empty queue → keep last setpoint); `boost::lockfree` becomes a gateway-private dependency behind the transport seam. Supersedes any ad-hoc copy/lock in the current HTTP+SSE gateway.

## ADR-7 — Seam error model is `std::expected`, not exceptions

**Decision (2026-07-14, from external review):** value-returning calls across the Module/Capability seams return `std::expected<T, core::Error>` (via `tl::expected` until the toolchain is C++23). Exceptions are not thrown across a seam boundary. The RT step path stays `noexcept` with latched safety state (already the case).

**Why:** exceptions unwinding across an ABI/plugin boundary are neither RT-safe (unbounded) nor stable across independently-compiled modules — a problem the moment Tier-A `.so` plugins and hot-swapped drivers cross the seam. `expected` makes the error path explicit, allocation-free, and ABI-stable. Replaces throwing factories such as `MotionPlan::move`'s `std::invalid_argument`.

**Consequences:** `core::Status` converges with `std::expected`; configuration verbs return `expected`; the boundary lint gains an exceptions-across-seams check. Small mechanical refactor across the existing factories.

## ADR-8 — CLI is a first-class, agent-complete surface; CLI and UI share one contract

**Decision (2026-07-14):** the `robonode` CLI is a primary surface, not an afterthought. It and the web UI are **both thin clients of the Platform facade** — the same command + telemetry contract, the same code path, no logic living in one surface and missing from the other. The CLI is **agent-complete**: every action a human performs in the UI (execute programs, swap/replace modules, drive lifecycle, monitor state, tail logs/traces/telemetry, fetch feedback) is doable headless from the CLI, with `--json` machine-readable output for agents. Command parsing uses a mature library (**CLI11**), never hand-rolled `argv` walking.

**Why:** the platform is meant to be driven by agents as much as humans; a headless, scriptable, machine-parseable surface is the substrate for that. Forcing CLI and UI through the *same* facade guarantees they never diverge — a demo that works in the UI works identically from the CLI because it is literally the same call. This is the "one clean interface" (MIL) principle applied to the client edge: the facade defines *what the system can do* once, and every surface projects it.

**Consequences:** the facade becomes the hard contract both surfaces bind to; a "UI-only" or "CLI-only" capability is treated as a bug; issue builds the CLI over the facade with CLI11 + JSON output + stream-follow (`logs -f`, `trace`, `telemetry`); the SDK is a third client of the same contract. A parity check (every facade verb reachable from CLI) guards against drift.

## ADR-9 — Logging = spdlog + fmt, structured/async; logs, traces, telemetry are one shared, followable surface

**Decision (2026-07-14):** logging uses **spdlog + fmt** (mature, professional) — structured (JSON sink), levelled, per-module, **async/lock-free** so it never blocks the control loop (the RT-safety mechanic is planned). No `printf`/`iostream` in product code. Logs, traces, and telemetry are exposed as **one queryable/followable observability surface** through the recorder seam, consumed identically by CLI (`logs -f`, `trace`, `telemetry`) and UI.

**Why:** ad-hoc logging is unusable for agents and unsafe in a 1 kHz loop. A mature async logger gives structured, machine-parseable records without I/O on the hot path; unifying logs/traces/telemetry behind one seam means every surface (and every agent) observes the system the same way, and the flight recorder (MCAP) and live stream are the same data at rest vs in motion.

**Consequences:** `spdlog`/`fmt` vendored behind a thin logging seam (module-private, boundary-lint clean); the recorder seam gains a log/trace channel alongside telemetry; CLI + UI subscribe to the same stream; is the RT-no-block piece, is the structured-logging + shared-surface piece.

## ADR-10 — Autonomy is guarded by executable gates, not trust; the design is enforced by code

**Decision (2026-07-14):** the repo is built to be worked by an **autonomous agent, start to finish, unsupervised**. Autonomy rests on three executable artifacts, not on the agent's good judgement:
1. **`scripts/verify.sh`** — the Definition of Done: design invariants + build + every C++ suite (+ `--e2e` browser journeys). Exit 0 = mergeable. Nothing is "done" until it passes.
2. **`scripts/check-design.sh`** — the design-drift gate (a CI job): the ADR decisions expressed as greppable rules (seams exist, RT-loop purity, descriptor-driven, no throw across seams). A violation fails CI, so the design **cannot** rot silently. New invariants are added as their issue closes — the guard hardens over time, like the scenario matrix.
3. **`AGENTS.md`** — the operating constitution: the loop (pick → build → test → verify → record → repeat), how to pick the next issue (tracker + `agent-ready`/`blocked` labels), the roadmap-as-single-source rule (update tracker + matrix as part of Done, no drift), and explicit stop-and-ask conditions.

**Why:** an agent left to "follow the plan" drifts — it hardcodes, bypasses seams, skips tests, and lets the roadmap and reality diverge. Encoding the plan as *gates the agent must pass* and a *constitution it must follow* makes the design self-enforcing: the human launches the loop and reviews closed issues, rather than supervising each step. The tools the agent needs are all headless — `gh` (issues), `ctest` (unit/integration), Playwright (e2e, plus the MCP server for live checks), the `robonode` CLI `--json`, and the observability stream.

**Consequences:** every issue is a tracer slice with executable acceptance criteria + `blocked-by`; the tracker and scenario matrix (`docs/TEST-STRATEGY.md`) are the single source of truth the agent updates as Done; CI runs `check-design.sh` (`design` job) + `verify.sh`'s suites; the harness itself is tracked by (label automation, next-issue helper, roadmap-consistency check, and enabling the pending `check-design` gates as ///// land).

## ADR-11 — Every algorithm capability is one shape: interface + ModuleRegistry + sandbox

**Decision (2026-07-18):** the platform's purpose is to **try different algorithms on different parts of the system** — trajectory, vision, control — each with a **basic version** the user can **replace with their own code, written and run from the web app**. To make that real without coupling exploding, every algorithm-bearing capability has the **exact same shape**, and no other pattern is introduced:

```
Capability<I> = interface I  +  ModuleRegistry<I, Ctx>  +  a selected version
```

- **The seam is an interface, nothing else.** `Planner` (trajectory), `Detector` (vision), `Controller` (control) — alongside the existing `AxisAdapter`, `Kinematics`. Product logic depends only on the interface; a concrete impl is never named above the seam.
- **One registry mechanism** — `ModuleRegistry<T,Ctx>` — lists versions, builds one, swaps live, for *every* capability. `DriverRegistry` was the first; vision/planner/control reuse it verbatim. No bespoke registry per type.
- **Capabilities never call each other.** The program/executive *composes* them (vision → planner → controller → axes) through their interfaces. A `Detector` does not know a `Controller` exists.
- **Selection is data.** `Platform.set_version(capability, version)` — the same verb `set_driver` already is for axes. An App records the chosen version per capability; the wiring is not code.

**Untrusted code runs behind a sandbox boundary.** A user version implements a capability interface but its body runs in **WebAssembly (Wasmtime, WASI off)**: no filesystem, network, or syscalls; memory + wall-clock limits. It sees only a **narrow typed ABI** (`frame → detections`, `poses → waypoints`) and **cannot touch the cell, driver, or hardware**. The **1 kHz path never calls user code** — perception/planning run async off the RT loop, and only a **host-validated** trajectory (bounds/limits checked) crosses into control. A sandbox breach/timeout/limit trip trips the existing cell-coherent safety hold. So the worst a malicious algorithm does is get its output rejected — never slam a joint.

**Why:** the alternative — each capability growing its own classes, wiring, and lifecycle — is exactly the coupling the platform can't afford. One repeated shape means adding a capability (or a user algorithm) changes nothing else, and the dangerous part (running user code) is isolated at one owned seam rather than sprinkled through product logic.

**Consequences:** `Detector`/`Planner`/`Controller` seams + their `register_*` helpers land as tracer slices (vision first — it already has a basic version); each capability leaves the gateway as it gains its seam (this is also the de-godding); a `SandboxedModule<I>` wraps any interface over Wasmtime; `check-design.sh` gains a rule that product code above a seam never names a concrete version. The web editor (write → compile-to-WASM → register as a new version → run → compare) is the surface this ADR exists to enable.

**Sandbox engine — staged (2026-07-20).** The isolation boundary is `robonode::sandbox::Program` + `sandbox::run` (fuel + stack bound, host-validated output): user source is **compiled** (infix → bytecode) and **run by a pure stack VM with no host surface** — no memory, no syscalls, no loops, so it cannot express I/O or non-termination. This Tier-B engine builds on every platform and proves the whole path end-to-end (`SandboxedDetector`: pose in → validated pose out; a fault or out-of-bounds answer yields no detection, never an unsafe move). **Wasmtime (WASI-off) is the same seam with a heavier engine** — it swaps behind `Program`/`sandbox::run` without touching the capability wrappers, the editor, or product code — and stays deferred until a WASM toolchain is vendored (it needs a compiled guest, which the dev box's Apple clang cannot emit). The seam is the durable decision; the engine is an implementation swap.

## ADR-12 — Commands have identity; completion is observed, never polled for

**Decision (2026-07-25):** every command submitted to the platform is acknowledged with an **id** (`{ok, id, queued}`), and that acknowledgement means *accepted*, not done. The telemetry frame carries `accepted_id`, `applied_id`, the cell `state`, and `last_error`, so a caller learns that its command finished by watching one number. `Platform::await_settled` is the single completion predicate the whole platform shares; the UI's `awaitApplied` is its browser twin.

**Why:** before this, the reply was `{"ok":true}` meaning "queued", and every surface invented its own notion of done — the CLI polled snapshots with per-verb predicates (including one that knew the demo's final rail position), the web app guessed with `setTimeout(300)`, and Compare slept between steps. Three incompatible workarounds for one missing protocol feature, each of them a source of flake and each coupling a client to the *content* of the motion it asked for.

**Consequences:** the queue is bounded and coalescing (a repeated idempotent verb replaces the pending one rather than stacking); progress is published the moment a command is accepted, so a caller never reads a snapshot that predates it; the three workarounds were deleted; `contracts/command-ack.schema.json` and `contracts/telemetry.schema.json` pin the shape, validated against a live server.

## ADR-13 — A robot is a node; axes are bound by name, and the carrier is part of the reach

**Decision (2026-07-25):** the cell descriptor declares `robots`, each naming its `carrier` axes and its `joints` **by id**. `RobotNode` binds those names to the live cell, owns the unit conversion between descriptor units (mm) and model units (m), and composes joint waypoints into full-cell waypoint lists. The kinematic chain solved by IK is `carrier ++ joints`, with the carrier given a low mobility weight so it is the solver's last resort.

**Why:** the gateway used to index the flat node list positionally (`nodes[0]` = rail, `nodes[1..6]` = arm), which baked a single 7-axis cell into product logic and silently no-opped on anything else. Worse, the rail sat *outside* the kinematic chain, so the 7th axis could never contribute reach and the reported TCP was in the wrong frame. Naming is what makes a second robot, a different arm, or a cell without a rail a descriptor change.

**Consequences:** a design gate fails the build on positional node indexing in `gateway/`; weighted damped-least-squares keeps the carrier parked unless the arm cannot reach (which improved Cartesian accuracy from ~35 mm to ~3 mm); IK is joint-limit-projected, and `TrajectoryValidator` refuses out-of-travel, non-finite or absurdly long trajectories *before* anything moves, instead of letting the governor clamp mid-motion and silently change the path.

## ADR-14 — Tuning is data; no heuristic is a literal

**Decision (2026-07-25):** every tunable lives in `config/robonode.settings.json` — IK damping, tolerance, iteration budget and step clamp, the carrier weight, interpolation steps, control rate, settle and on-path stop windows, the trajectory-duration ceiling, the soft-limit tolerance, pick/place approach clearance, queue capacity, stream period, telemetry decimation, sandbox workspace bounds, and run recording. `Settings` is a plain struct in `core` (which depends on nothing); the JSON loader lives in `celld`, which already owns that dependency. `ROBONODE_SETTINGS` overrides the path, and a partial file overrides only the keys it names.

**Why:** these numbers are the ones an operator most wants to change and most often gets wrong, and every one of them was a literal scattered through the code that used it. A heuristic you cannot change without a rebuild is a heuristic nobody will improve. This is the anti-hardcoding rule applied to behaviour, not just to identifiers and paths.

**Consequences:** a design gate requires the settings file to exist; compose mounts it read-only so a deployment can tune without a rebuild; defaults live in one struct, so an absent file is a valid configuration rather than a crash.

## ADR-15 — The software stop is a stop, not a safety function

**Decision (2026-07-25):** `stop` ramps the executive's **path clock** to zero over `motion.stop_time_s`, decelerating the axes along the planned path (stop category 2) rather than stepping to a hold. `estop` does the same and **latches** the cell so no motion is accepted until `resume`. All three act on the calling thread, because a queue the worker is draining is exactly the situation they exist for. Every surface, log line and document calls this a *software* stop.

**Why:** a robotics platform that cannot stop a moving robot is not a robotics platform — before this there was no abort, pause or e-stop verb anywhere, and a long program held the worker for its entire duration. Equally, presenting a software latch as a safety function would be a dangerous lie: the safety function is hardware's (SPEC I4, ADR-4). The honest position is to provide the stop, make it behave correctly on-path, and be explicit about what it is not.

**Consequences:** `CancelToken` is read once per cycle by the executive (one relaxed atomic load, RT-safe); a run that ends early reports `stopped_early` rather than claiming success; `CellSupervisor` owns the latch and refuses new work while it is held; the existing cell-coherent safety gate (any axis non-NORMAL holds every axis) is unchanged and remains the mechanism a real safety input would drive.

## Open

- **OQ-3 — open-core license boundary** (REQUIREMENTS NFR-12): needs counsel + business input before anything is published publicly. Interim rule: nothing leaves the private repo, so no boundary is being created implicitly.
- **OQ-6/7/8** (SPEC §13): BT exposure in UI, GPU inference API neutrality, Zenoh support contract.

## ADR-16 — Contacts are on, and a grasp is a constraint

**Status:** accepted (2026-07-26)

**Context.** The world shipped with every collision geom at
`contype="0" conaffinity="0"` and a grasp implemented as a pose the gateway
remembered. The arm passed through parts and fixtures, and a "pick" moved
nothing. For a platform whose promise is *testing algorithms in real physics*,
that is the one thing that must not be pretend.

**Decision.** Contacts are enabled and grouped by what a thing is (robot ·
structure · parts · work surface · tool tip — see ARCHITECTURE). A grasp closes
a weld the model declares, captured on the live relative pose. The workpiece is
a free body carried by belt friction. Two exceptions are deliberate and narrow:
the arm takes no contact from itself (a control twin must not hold its own
joints off target), and the suction tip takes none from parts (a cup that
shoved its target away could not pick anything up).

**Consequences.** Poses that intersect the cell now fail, so the shipped
motions, stations and apps had to become physically sensible: the robot has a
real tool, the pallet sits where the arm can reach it without leaning on it, and
the line delivers a part per cycle. IK retries from mid-travel before calling a
target unreachable. The cell explains itself through `contacts`, which is now
part of telemetry, the log, the dashboard and the CLI.

**One owner.** `mjData` is not thread-safe, and enabling contacts made that
visible: any surface that read the live world while the worker stepped it
corrupted the solver's stack. The thread that drives the physics samples frames
and contacts once per published cycle, per command, and whenever a station or
the gripper moves something; everyone else reads that snapshot. No lock is
added to the 1 kHz path (ADR-5/6/7 hold).

**Alternatives rejected.** Leaving contacts off and modelling "held" in the
gateway (the status quo — it makes every collision-aware algorithm untestable).
Welding to a part that still had its transport joint (over-constrained: the
solver fights itself). Making the belt a contactless transport (then a part
could not rest, stack or be knocked over).

## ADR-17 — One context, one result, one place a setting comes from

**Status.** Accepted (2026-08-01).

**Context.** The platform is described as MIL-style — one clean interface, the
complexity behind it — but "MIL-style" was a design intent, not something the
code could be checked against. Three questions had no written answer: what owns
a thing's lifetime, what a call gives back when it fails, and where a tunable
comes from. Each had drifted somewhere.

**Decision.**

*Context owns lifetime.* `Platform` is the context a surface allocates; a
`CellGateway` is allocated in it, and everything a cell owns — its world, its
capabilities, its snapshot, its stores — is reached through it. Nothing that
belongs to a cell is a global. The log was the exception this ADR first recorded
as an open one: `Log::instance` is a process-wide sink, so `GET /logs?cell=x`
answered with every cell's records. It is closed — a record carries the cell
whose thread produced it, and one made outside any cell belongs to the platform
and shows in every cell's view, because it is equally true of all of them. The
sink stays shared (one ring, one stderr stream); the *scope* is no longer a
claim the platform cannot back.

*One result.* `Status` when a call returns nothing but can fail; `Result<T>`
(`std::expected<T, Status>`) when a value comes back. Never `Status f(…, T&
out)`, never an exception across a seam. A failure carries a reason a person can
read: not "IK failed" but how far short it was, after how many iterations.

*Stateless or stateful, decided per type, never both.* A capability version
(detector, planner, controller) is a pure function of its inputs and is
swappable at runtime. A gateway, a scene view and a store hold state, are
non-copyable, and say which lock guards what. A type that is neither is the
thing to split — `SceneView` came out of `CellGateway` exactly this way.

*One place a setting comes from.* `config/robonode.settings.json` is resolved
once, at the boundary, into `Settings`, and injected. A default argument that
reads `Settings{}.motion.stop_time_s` is a second source of truth wearing a
different hat: the caller that forgot to pass it compiles into something that
looks deliberate. Those defaults are gone; the tunables are parameters the
caller must supply, and product code supplies them from `cfg_`.

**Consequences.** `check-design.sh` fails the build on a compiled-in settings
default and on a raw `Scene` outside its owner. Tests pass their own values,
which is the point — a test may want a different settle time, but it now says
so. Removing the defaults immediately showed that no product caller had been
relying on them, which is the answer worth having.

**Alternatives rejected.** A MIL-shaped C handle API (`RN_ID` + `rnGetResult`)
— the ergonomics are worse in C++23 and `std::expected` already gives the
result half. A global settings singleton (fast, and exactly the drift ADR-14
exists to prevent). Leaving the defaults in place with a comment (a comment does
not fail a build).
