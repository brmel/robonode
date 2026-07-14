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

## Open

- **OQ-3 — open-core license boundary** (REQUIREMENTS NFR-12): needs counsel + business input before anything is published publicly. Interim rule: nothing leaves the private repo, so no boundary is being created implicitly.
- **OQ-6/7/8** (SPEC §13): BT exposure in UI, GPU inference API neutrality, Zenoh support contract.
