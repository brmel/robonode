# RoboNode — System Specification

> **Status:** Draft v0.1 — 2026-07-11. Companion to [REQUIREMENTS.md](REQUIREMENTS.md); FR-x references point there. Technology choices are justified in [ARCHITECTURE.md](ARCHITECTURE.md).

---

## 1. Architecture overview

Three planes, strict downward-only trust:

```
┌──────────────────────── CLOUD / ON-PREM CONTROL PLANE ────────────────────────┐
│  Fleet API (gRPC/REST) · Registry (drivers, algorithms, configs) · Identity   │
│  Telemetry lake (MCAP) · Dashboards · OTA orchestrator · Web app              │
└───────────────▲───────────────────────────────▲───────────────────────────────┘
                │ mTLS, store-and-forward        │ (never in control path)
┌───────────────┴───────────────────────────────┴───────────────────────────────┐
│                       CELL CONTROLLER (edge, Linux PREEMPT_RT)                 │
│  ┌───────────── coordination plane (non-RT) ─────────────────────────────────┐ │
│  │ celld: config convergence · node supervision · program engine (BT)        │ │
│  │ Tier B algorithm sandboxes (WASM/containers) · twin runner (sim gate)     │ │
│  │ zenohd router: topics/queries · flight recorder (MCAP ring)               │ │
│  │ north gateways: WebSocket/gRPC for UI · OPC UA/Sparkplug [v1.x]           │ │
│  └───────────────▲───────────────────────────────────────────────────────────┘ │
│                  │ SPSC queues / shared-mem (lock-free, absolute setpoints)    │
│  ┌───────────────┴────────────── motion core (RT, 1 kHz) ────────────────────┐ │
│  │ motion tree · interpolation/blending · OTG (Ruckig-class) · governors     │ │
│  │ Tier C plugin slots (watchdogged) · safety-state observer                 │ │
│  │ vendor adapters: EtherCAT CSP @1kHz · UR RTDE/servoj @500Hz ·             │ │
│  │                  Fanuc StreamMotion @8ms · ABB EGM @4ms · sim adapter     │ │
│  └───────────────▲───────────────────────────────────────────────────────────┘ │
└──────────────────│─────────────────────────────────────────────────────────────┘
                   │ fieldbus / vendor RT protocols
        ┌──────────┴──────────┐        ┌────────────────────────────┐
        │ drives · arms · I/O │◄──────►│ certified safety chain     │
        │ grippers · cameras  │        │ (e-stop, relays, PL d/e)   │
        └─────────────────────┘        └────────────────────────────┘
```

Key invariants:

- **I1** Cloud is management-only; a cell runs forever without it (NFR-5).
- **I2** Nothing crosses into the RT domain except through lock-free queues; the RT thread never allocates, blocks, or logs synchronously.
- **I3** User code (any tier) can only affect actuators through capability APIs, and every actuator command passes a platform **governor** (limits, fences, safety state) below the plugin boundary (FR-8.4).
- **I4** Safety function lives in the certified hardware chain; software observes and constrains, never implements it (FR-8.1).
- **I5** Sim twin and real cell expose byte-identical APIs (FR-5.1); artifact deploys are gated on twin tests (FR-3.3).

## 2. Node model

### 2.1 Identity & tree

```
cell/<cell-id>/node/<node-id>            e.g. cell/mtl-line1-c3/node/ur10e-1
                                              cell/mtl-line1-c3/node/rail-x
parent relationship + transform:              ur10e-1.parent = rail-x (carriage frame)
```

Node descriptor (served at `GET /nodes/{id}`, published on change):

```jsonc
{
  "id": "rail-x",
  "driver": { "name": "robonode.ethercat-cia402", "version": "1.4.2" },
  "device": { "vendor": "generic", "model": "belt-1500", "serial": "A123" },
  "parent": { "node": null, "transform": { "xyz": [0,0,0], "rpy": [0,0,0] } },
  "capabilities": [{
    "type": "MotionAxis@1",
    "units": "mm",
    "limits": { "pos": [0, 1450], "vel": 1200, "acc": 8000, "jerk": 120000,
                "force": 120 },
    "command_rate_hz": 1000,
    "modes": ["position", "velocity", "torque"],
    "blending": true, "streaming": true
  }],
  "state": "active",
  "safety": "NORMAL"
}
```

Capability schemas are versioned protobuf/JSON-schema pairs in the open `robonode-idl` repo; **limits are data** (FR-1.3).

### 2.2 Core capability interfaces (v0 set)

| Capability | Commands (abridged) | Telemetry |
|---|---|---|
| `MotionAxis@1` | `home, move_to(pos, profile, blend), move_seq([...]), jog(vel), stream_open(rate) → stream_write(setpoint), stop(kind)` | pos/vel/acc actual+target, following-err, torque, temp @ ≤1 kHz |
| `ArmKinematics@1` | `movej/movel(seq w/ blend), jog(joint|cart), stream_open(joint|cart), fk/ik, set_tcp, set_payload, freedrive` | joint states, TCP pose, safety_state @ ≥125 Hz |
| `Gripper@1` | `grip(width,force), release, get_state` | width, force, object-detected |
| `Camera@1` | `capture, stream(profile)` | frames (shm-backed refs on-cell), intrinsics |
| `ForceTorque@1` | `zero, get_wrench, stream` | wrench @ ≥500 Hz |
| `DigitalIO@1` | `read/write/subscribe` | edges, values |
| `Conveyor@1` | `run(vel), stop, index(dist)` | vel, position tick |

`stream_*` is the FR-2.6 primitive: opened streams route through the OTG + governor, so arbitrary user setpoints become jerk-limited, limit-respecting motion or are rejected with a typed reason.

### 2.3 Lifecycle

`unconfigured → configuring → inactive ⇄ active → degraded → fault` (FR-1.2), modeled after ros2_lifecycle semantics; transitions are commands with acks; every state carries `reason` + timestamp. `celld` converges declared config → actual (create/configure/activate order respects tree dependencies; teardown reverse).

## 3. Motion core (RT domain)

### 3.1 Pipeline

```
program engine / SDK / Tier B          (non-RT, command batches, blend specs)
        │
        ▼
motion tree manager     — groups nodes into synchronized chains (FR-2.4);
        │                 one clock master per cell (PTP)
        ▼
path & blend layer      — waypoint sequences → geometric path with blends
        │                 (parabolic/spline corners; per-node blend capability)
        ▼
retiming (TOPP-lite)    — limit-aware time law incl. constant-TCP-speed [S]
        │
        ▼
OTG (Ruckig-class)      — per-cycle state → target, jerk-limited, retargetable;
        │                 Tier C TrajectoryGenerator plugin may REPLACE this slot
        ▼
governors               — position/vel/acc/jerk/torque envelopes, workspace
        │                 fences, safety-state scaling; platform-owned, always-on
        ▼
vendor adapters         — EtherCAT CiA402 CSP @1 kHz · UR servoj/RTDE @500 Hz ·
                          Fanuc Stream Motion @8 ms · ABB EGM · sim adapter
```

- RT executive: single 1 kHz cycle (`clock_nanosleep` absolute), SCHED_FIFO, pinned core, `mlockall`; per-adapter subcycles derived from the master clock.
- Adapters translate the platform setpoint (joint or Cartesian per capability) into vendor protocol each vendor cycle; absolute setpoints, sequence-numbered (FR-2.10).
- E-stop/protective-stop observation propagates within one cycle; motion states per FR-2.7 (`Idle/Jogging/Executing/Pausing/Paused/Stopping/ProtectiveStop/EStop/Fault`), e-stop reachable from all.

### 3.2 Tier C plugin contract (sketch)

```cpp
// robonode/rt_plugin.hpp — fixed ABI, no exceptions across boundary, no allocation in step()
struct RtContext {            // read-only snapshot, provided each cycle
  double t;                   // cell clock, s
  std::span<const JointState> chain_state;
  SafetyState safety;
};
class TrajectoryGenerator {   // one of: TrajectoryGenerator | SetpointFilter | ForceController
public:
  virtual Status configure(const KinematicChain&, const Limits&, ParamView) = 0;
  virtual Status step(const RtContext&, const Target&, SetpointOut&) = 0;  // must return < budget_us
  virtual void   reset() noexcept = 0;
};
```

Runtime protections: WCET budget per plugin (measured at load in sim, enforced by watchdog); overrun / NaN / limit-violating output ⇒ same-cycle fallback to platform OTG, node → `degraded`, incident bundle recorded (FR-3.1). Plugins are signed `.so` artifacts; loading unsigned requires commissioning mode.

## 4. Algorithm tiers (non-RT contracts)

### 4.1 Tier B — edge skill

- Packaging: WASM component (wasmtime, WASI + `robonode:cell` world) for logic-weight algorithms; OCI container for heavy ones (perception, GPU). Both get the **same gRPC surface** (`CellClient`): subscribe topics, call capability commands, open setpoint streams, read kinematics/params.
- Scheduling: best-effort 10–100 Hz; quotas (CPU/mem/stream-rate) from manifest; no device or network access beyond the cell API socket.
- Manifest (registry artifact, FR-3.2):

```yaml
name: acme/visual-servo
version: 1.2.0
tier: B
runtime: wasm-component            # or oci
requires:
  - capability: ArmKinematics@1    # bound at deploy time to concrete nodes
  - capability: Camera@1
params_schema: ./params.schema.json
resources: { cpu: "500m", mem: "256Mi" }
tests: { sim_suite: ./tests/sim, min_score: pass }   # FR-3.3 gate input
signature: sigstore
```

### 4.2 Tier A — cloud/offline

OCI containers against the telemetry lake + fleet API (read) and registry (write parameters/models). No cell access. Standard K8s jobs; out of RT scope entirely.

## 5. Data plane

- **On-cell + cell↔cloud transport: Zenoh** (router on cell, peers for runtimes); ROS 2 interop via `zenoh-plugin-ros2dds`/`rmw_zenoh` bridge so existing ROS nodes appear as external nodes (OQ-2 resolution: Zenoh-native core, ROS 2 as bridged ecosystem, revisit if pilot customers are ROS-heavy).
- Topic scheme: `rn/<cell>/<node>/<capability>/{state|telemetry|cmd|evt}`; typed by the same IDL as capabilities; queries for descriptors/params (Zenoh queryables).
- Large payloads (camera frames): on-cell shared-memory with topic-carried references; cloud sync sends downsampled derivatives per policy (FR-6.4).
- **Recording: MCAP** — 1 kHz flight-recorder ring (journaled, power-loss safe, FR-6.2/NFR-10), incident bundles = MCAP + config snapshot + artifact versions; **Foxglove-compatible** live (WebSocket bridge) and offline (FR-6.3).
- Time: PTP (or PTP-over-TSN later) on-cell; all telemetry stamped from the cell clock domain (FR-6.1).

## 6. North APIs & security

- gRPC (canonical, from `robonode-idl`) + REST transcoding + WebSocket event/telemetry push (FR-4.3). UI consumes only these.
- AuthN: OIDC for humans (local accounts fallback), mTLS device certs for cells (provisioned at claim, FR-10.3), scoped PATs for CI (FR-10.4).
- AuthZ: RBAC roles Operator/Engineer/Admin/Viewer scoped org→site→cell (FR-10.2); jog/deploy/support-shell privileges separate; every mutating call audit-logged (FR-6.6).
- Browser jog safety: jog session = server-issued lease + 10 Hz heartbeat; loss ⇒ controlled stop (FR-2.1). Speed override server-side.
- Supply chain: all artifacts (OS image, drivers, algorithms, configs) signed (sigstore), SBOM per release (NFR-8).

## 7. Web application (v0 pages)

1. **Fleet** — cells grid, status/safety badges, versions, alerts.
2. **Cell** — node tree + live 3D twin (three.js from URDF), safety banner, program run controls.
3. **Node** — descriptor, live telemetry charts, jog panel (lease UI), fault history.
4. **Programs** — BT/flow canvas editor (FR-4.1), run history → recordings.
5. **Algorithms** — registry browser, versions, sim-gate results, deploy/rollback.
6. **Data** — recordings list, open-in-Foxglove, incident bundles.
7. **Admin** — users/roles, device claiming, OTA channels.

## 8. Simulation subsystem

- Physics/kinematics twin: **Gazebo (Harmonic-class)** headless on cell or dev machine; sim vendor-adapter implements the same adapter interface as EtherCAT/UR adapters, so the motion core is unchanged (I5).
- `robonode dev`: CLI spins up celld + motion core + twin + local web UI from a cell config; hot-reload of Tier B artifacts (FR-3.4).
- Sim gate runner: deterministic scenario suites (limit sweeps, retarget storms, packet-loss injection on adapters, e-stop mid-blend) + algorithm's own tests; results stored with the artifact version (FR-3.3, FR-5.2).
- Isaac Sim connector deferred [C] (FR-5.4) — descriptor→USD export keeps the door open.

## 9. Cell controller platform & OTA

- Reference hardware v0: **industrial x86_64 + PREEMPT_RT** (simplest path to NFR-1); Jetson Orin variant [S] when GPU perception tiers land (OQ-1: x86 first).
- OS: immutable A/B image (Yocto or Ubuntu Core decision at M1) + container runtime for coordination-plane services; motion core native.
- OTA: image A/B with health-gated auto-rollback (Mender-style); drivers/Tier B via per-component container/wasm updates; staged rollout channels (canary→stable) (FR-7.2).
- Provisioning: flash → boot → claim token → org enrollment (mTLS cert issued) → config converge (FR-7.4).

## 10. Safety architecture (I4 detail)

- Certified chain: e-stop buttons, guard switches, safety relay/PLC (Cat 3, PL d/e) wired to drive STO and arm safety inputs; platform reads chain state via safe inputs.
- Software mapping: chain events → stop categories (IEC 60204 Cat 0/1) at adapters; Cat 2 (SS2-style hold) platform-initiated for protective stops; reaction budget ≤ 10 ms software-side (NFR-2) — chain acts faster independently.
- Governors (I3): joint/Cartesian envelopes, workspace fences, reduced-speed modes (T1 ≤ 250 mm/s TCP, FR-8.6), safety-state speed scaling; compiled into the RT path, configuration signed with the cell config.
- Commissioning artifacts: exportable safety-config report (FR-8.5).

## 11. Technology choices (summary — rationale + exact verified pins in [ARCHITECTURE.md](ARCHITECTURE.md) Part 2)

| Slot | Choice (v0) | Runner-up |
|---|---|---|
| RT OS | Linux + PREEMPT_RT | Xenomai |
| RT kinematics | **Pinocchio (C++, in-process)** — ADR-5 | RTB offline / KDL |
| Motion OTG | Ruckig (community) wrapped behind Tier C slot | in-house OTG |
| Global planning | cuRobo (GPU) | OMPL (CPU) |
| RT hand-off | **lock-free SPSC (boost::lockfree)** — ADR-6 | — |
| Seam error model | **`std::expected`** — ADR-7 | `core::Status` |
| Fieldbus master | IgH EtherCAT (kernel, CiA 402) | SOEM / Acontis (commercial fallback) |
| Arm protocols | UR RTDE+servoj first; Fanuc J519, ABB EGM next | vendor ROS 2 drivers via bridge |
| Data plane | Zenoh (+ zenoh-shm for vision) | DDS (CycloneDDS) / eCAL |
| ROS 2 stance | Bridge, not foundation (review-confirmed) | full ROS 2 core |
| Tier B sandbox | wasmtime (AOT/Cranelift, WASI-off) + OCI; Tier-A native C-ABI `.so` | gVisor-only containers |
| Recording | MCAP + Foxglove compat | rosbag2 |
| CLI surface | **CLI11 — agent-complete, same facade as UI** (ADR-8) | hand-rolled argv (rejected) |
| Logging | **spdlog + fmt — structured, async/lock-free** (ADR-9) | glog / iostream |
| Sim | **MuJoCo (local twin, in-process)** | Gazebo (Linux/CI) / Isaac (synthetic data) |
| Orchestration | BehaviorTree.CPP engine under flow UI | Node-RED embed |
| OTA | Mender-style A/B + container updates | balena |
| North integration | OPC UA server + Sparkplug B option | REST-only |
| Cloud | K8s, Postgres, object store (MCAP), gRPC gateway | — |

*RT-loop discipline (ADR-5/6/7): no Python, no heap alloc, no locks, no blocking I/O on the 1 kHz path. In-loop descriptor reads use FlatBuffers (JSON for authoring); perception is async-decoupled off the loop.*

## 12. Milestones

- **M0 (weeks 1–4)** — `robonode-idl` v0 (capabilities, descriptors, topics), motion core skeleton @1 kHz with sim adapter, `robonode dev` boots a twin axis; profile math (trapezoid/S-curve) folded into motion as the OTG-slot seed.
- **M1 (weeks 5–10)** — UR adapter (URSim), MotionAxis via EtherCAT on bench, blended arm+axis sequences (D2 demo), flight recorder + Foxglove live.
- **M2 (weeks 11–16)** — Tier B runtime (wasmtime + CellClient gRPC), registry v0 + sim gate, Python SDK, jog UI + cell page.
- **M3 (weeks 17–22)** — MVP scenario end-to-end (REQUIREMENTS §6), OTA A/B, incident bundles, RBAC v0. Pilot-ready.
- **M4** — Tier C slot hardening (watchdog/WCET), Fanuc or ABB adapter, force-control skill [S] items.

## 13. Open items

OQ-1/2/4/5 resolved — see [DECISIONS.md](DECISIONS.md) (ADR-1…4); OQ-3 open. Additions:
- OQ-6: BT engine exposure — do users see trees or only the flow abstraction?
- OQ-7: on-cell GPU inference API for Tier B containers (NVIDIA-only vs ONNX-runtime neutral)?
- OQ-8: Zenoh licensing/support contract (Eclipse project; ZettaScale commercial) for pilots.
