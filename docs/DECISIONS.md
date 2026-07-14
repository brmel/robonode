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

**Consequences:** new dependency `pinocchio` owned by a C++ kinematics impl behind the seam (issue #34, P0); `bridges/rtb` + `services/rtb-kinematics` stay for offline use and the real-robot model catalogue; analytic derivatives unlock gradient-based Tier-C planning (Crocoddyl, #41). The `UsingRealRobot` thesis is unchanged — RTB still supplies validated real-robot models; Pinocchio just does the math the loop needs.

## ADR-6 — Non-RT↔RT hand-off is a lock-free SPSC queue; the 1 kHz path never locks or allocates

**Decision (2026-07-14, from external review):** all data crossing between the non-real-time `CellGateway` (commands in, telemetry out) and the real-time executive goes through **lock-free single-producer/single-consumer queues** (pre-allocated ring, `memory_order_release`/`acquire`). The RT path performs no heap allocation, no mutex, no blocking I/O. We vendor a vetted implementation (`boost::lockfree::spsc_queue`) rather than hand-rolling.

**Why:** a mutex shared between the gateway thread and the motion thread admits priority inversion — if the OS pre-empts the gateway while it holds the lock, the 1 kHz loop blocks and misses its deadline. The reviews call SPSC the standard remedy and explicitly warn against hand-rolling (cache-coherence, false sharing, memory-reordering hazards).

**Consequences:** `CellGateway` gains a command-ingress and a telemetry-egress SPSC (issue #35, P0); the executive polls wait-free (empty queue → keep last setpoint); `boost::lockfree` becomes a gateway-private dependency behind the transport seam. Supersedes any ad-hoc copy/lock in the current HTTP+SSE gateway.

## ADR-7 — Seam error model is `std::expected`, not exceptions

**Decision (2026-07-14, from external review):** value-returning calls across the Module/Capability seams return `std::expected<T, core::Error>` (via `tl::expected` until the toolchain is C++23). Exceptions are not thrown across a seam boundary. The RT step path stays `noexcept` with latched safety state (already the case).

**Why:** exceptions unwinding across an ABI/plugin boundary are neither RT-safe (unbounded) nor stable across independently-compiled modules — a problem the moment Tier-A `.so` plugins and hot-swapped drivers cross the seam. `expected` makes the error path explicit, allocation-free, and ABI-stable. Replaces throwing factories such as `MotionPlan::move`'s `std::invalid_argument`.

**Consequences:** `core::Status` converges with `std::expected` (issue #36); configuration verbs return `expected`; the boundary lint gains an exceptions-across-seams check. Small mechanical refactor across the existing factories.

## Open

- **OQ-3 — open-core license boundary** (REQUIREMENTS NFR-12): needs counsel + business input before anything is published publicly. Interim rule: nothing leaves the private repo, so no boundary is being created implicitly.
- **OQ-6/7/8** (SPEC §13): BT exposure in UI, GPU inference API neutrality, Zenoh support contract.
