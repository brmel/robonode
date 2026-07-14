# RoboNode — system map

> High-level map of the platform: every component (web, app, backend, services, domain) and every node/module type with the versions a user can swap or author. ✅ implemented · ▶ planned (issue #). Vision: an open-source, modular platform where users see and manipulate robots + stations in real physics, and replace any module (path/trajectory, control, vision, …) with their own, tested quickly and safely — one clean interface hiding the complexity (Matrox-Imaging-Library-style).

## Components (layers)

```mermaid
flowchart TB
  subgraph FE["Frontend · web (apps/cell_server/web)"]
    UI["3D viewer (three.js) ✅ · node table + per-node version dropdowns ✅<br/>clean I/O (setpoint→actual→Δfollow) ✅ · Cartesian goal input ▶#22 · node inspector ▶#24 · algo editor ▶#26"]
  end
  subgraph APP["App · servers (apps/*)"]
    SRV["cell_server (HTTP+SSE) ✅ · robonode_dev / arm_dev / mujoco_dev / rtb_dev ✅"]
  end
  subgraph BE["Backend · C++ modules (one package, boundary-lint enforced)"]
    GW["gateway · CellGateway (transport seam) ✅ (Zenoh/gRPC swap-in ▶)"]
    FACADE["platform facade (one MIL-style entry) ▶#33"]
    CELLD["celld · Cell: node tree, lifecycle, coherent run ✅ · stations ▶#31"]
    MOTION["motion · governor ✅ · OTG/Ruckig ✅ · blend/SyncBlendPlan ✅<br/>executives ✅ · Kinematics/Planner seams ✅ · Module/Capability ontology ▶#30"]
    REG["DriverRegistry (version list + live swap) ✅ → ModuleRegistry ▶#30"]
    REC["recorder · MCAP/Foxglove ✅"]
  end
  subgraph SVC["Services / sidecars"]
    RTB["rtb-kinematics (Python · Robotics Toolbox) ✅ real models FK/IK/plan"]
    SAND["Tier-B sandbox (wasmtime / py) ▶#26"]
    VIS["vision service (OpenCV / DL) ▶#32"]
  end
  subgraph DEP["Vendored (not reinvented)"]
    MJ["MuJoCo physics ✅"] ; RUCK["Ruckig OTG ✅"] ; MCAP["MCAP/Foxglove ✅"] ; URCL["ur_client_library ✅"]
  end
  subgraph DOMAIN["Domain / business (data, not code)"]
    IDL["robonode-idl · capabilities, topics, descriptors ✅"]
    DESC["cell descriptors: tree + nodes + stations + programs ▶#28/#29 (single source of truth, no hardcoding)"]
  end

  UI <-->|JSON/SSE| SRV
  SRV --> GW --> FACADE --> CELLD
  CELLD --> MOTION --> REG
  CELLD --> REC
  MOTION -->|Kinematics/Planner seam| RTB
  REG -.->|loads user modules| SAND
  CELLD -.-> VIS
  MOTION --> MJ ; MOTION --> RUCK ; REC --> MCAP
  DESC --> CELLD ; IDL --> DESC
```

## Nodes / modules and their versions (the registry)

Every node is a **Module** exposing a typed **Capability** with a clean I/O contract (in: setpoint/command · out: state/telemetry · lifecycle: configure/activate/deactivate). A user picks a version per node in the UI, or **authors their own** version satisfying the interface — it appears in the same dropdown.

```mermaid
flowchart LR
  subgraph MotionAxis["Capability: MotionAxis@1 ✅ (rail + each joint)"]
    A1["robonode.sim-axis (filter) ✅"]
    A2["robonode.sim-axis-soft (sluggish) ✅"]
    A3["robonode.mujoco-axis (physics) ✅"]
    A4["robonode.ur-wrist (real UR RTDE) ✅"]
    A5["your driver ▶#23 (BYO template)"]
  end
  subgraph Arm["Capability: ArmKinematics@1 ✅"]
    K1["MujocoKinematics (FK/Jac) ✅"]
    K2["RtbKinematics (real robot models) ✅"]
    K3["URDF/DH your model ▶#25 (FANUC, …)"]
  end
  subgraph Plan["Capability: Planner ✅"]
    P1["JointPlanner ✅"]
    P2["CartesianLinePlanner (moveL) ✅"]
    P3["RtbPlanner (real IK) ✅"]
    P4["OMPL / collision-aware ▶"]
    P5["your planner ▶ (same seam)"]
  end
  subgraph Vision["Capability: Vision ▶#32"]
    V1["MuJoCo camera → frames ▶"]
    V2["OpenCV detector ▶ (not reinvented)"]
    V3["DL model (ONNX/…) ▶"]
    V4["your detector ▶"]
  end
  subgraph Station["Capability: Station ▶#31"]
    S1["conveyor / moving deck ▶"]
    S2["pallet ▶"]
    S3["your station ▶"]
  end
  subgraph Algo["Tier-B user algorithm ▶#26 (sandboxed)"]
    Z1["reads a node's output, writes its setpoint ▶"]
  end

  REG["Module/Capability registry ✅→▶#30<br/>(version list · live swap · lifecycle)"]
  REG --- MotionAxis ; REG --- Arm ; REG --- Plan ; REG --- Vision ; REG --- Station ; REG --- Algo
```

## Control/telemetry flow (browser → physics → back)

```mermaid
sequenceDiagram
  participant U as Browser
  participant G as gateway/CellGateway ✅
  participant C as celld/Cell ✅
  participant M as motion (governor+OTG+exec) ✅
  participant P as MuJoCo physics ✅
  U->>G: POST /command (run · set_driver · move_l ▶#22)
  G->>C: replace_node / run program
  C->>M: SyncBlendPlan → executive (1 kHz)
  M->>P: setpoints (governed, safety-gated)
  P-->>M: joint state
  M-->>G: CycleHook telemetry (target·actual·Δfollow) ✅
  G-->>U: SSE (node tree + live I/O) ✅
  Note over U,P: Cartesian goals via RtbPlanner (real IK) ▶#22 · algo in the loop ▶#26
```

## Principles (enforced)

- **One clean interface** hiding complexity — the Module/Capability seam (#30) + Platform facade (#33). MIL-style single ontology.
- **No hardcoded values / no duplication** — everything (tree, nodes, stations, limits, programs, ports, robot) is descriptor data (#28/#29), guarded by anti-hardcoding lint (#32).
- **Don't reinvent** — physics (MuJoCo), kinematics/models (Robotics Toolbox), OTG (Ruckig), vision (OpenCV/DL), telemetry (MCAP/Foxglove). Full catalog of reused/candidate open-source repos (incl. ROS 2, MoveIt, ros2_control, OpenCV, ONNX, vendor drivers): **[OSS-STACK.md](OSS-STACK.md)**.
- **Bring your own, safely** — any capability has a user-version slot; Tier-B runs user code sandboxed (#26); the governor is always the last safety net.
- **Structural modularity** — module boundaries enforced by `scripts/check-boundaries.sh` in CI; each module owns its dependency.
