# Tech Landscape & System Survey

> Research snapshot 2026-07-11 (web + prior Vention intel in [research/](research/)). Feeds the choices in [SPEC.md §11](SPEC.md). Each section: what it is → what we learned → verdict for RoboNode.

---

## 1. Direct analogs (build-vs-differ map)

### Vention (MachineBuilder / MachineLogic / MachineMotion AI)
Full-stack hardware-to-cloud; CAD model as single source of truth; MMAI controller = Jetson Orin + EtherCAT master (up to ~30 drives) running NVIDIA cuMotion + FoundationPose on-controller. **Verified gaps (our opening, detail in [research/vention-motion-api.md](research/vention-motion-api.md)):** blending exists only on robot move sequences (none on actuators, none arm+rail); no public streaming/servo API; v2 controller admitted single-threaded event loop; servo tuning via warranty-voiding backdoor; jerk parameter optional/possibly unhonored; external surface limited to MQTT/HTTP/EtherNet-IP. Closed algorithm stack end-to-end.
**Verdict:** benchmark for hardware UX; anti-pattern for openness. Our D1–D5 differentiators target exactly these seams.

### Viam
`viam-agent` supervises `viam-server`; machines declared in cloud JSON config; **components/services/modules** model with a semver'd **modular registry**; SDKs in Python/Go/TS/C++/Flutter; fragments for reusable configs; offline-queueing data sync. The best plug-and-play developer experience in robotics today.
**Verdict:** adopt the shape (registry, declarative config, fragments, capability APIs); differ on substance — industrial motion (RT, fieldbus, coordinated chains), functional-safety posture, and RT-tier plugins, none of which Viam addresses. Sources: [What is Viam](https://docs.viam.com/what-is-viam/), [modular resources](https://docs.viam.com/registry/modular-resources/), [platform overview](https://www.viam.com/platform/overview).

### Wandelbots NOVA
Robot-agnostic "OS": brand-independent control + Python/JS APIs; Omniverse/Isaac Sim digital-twin integration; Plan-Build-Operate lifecycle; VW Sachsen case study. Software-only, arms-focused, user code outside the servo loop.
**Verdict:** validates robot-agnostic APIs + sim-first; our motion tree (axes+arm as one chain) and RT plugin tier go deeper. Sources: [NOVA product](https://www.wandelbots.com/wandelbots-nova), [docs](https://docs.wandelbots.io/26.4), [launch](https://www.businesswire.com/news/home/20241106616593/en/Wandelbots-Presents-NOVA-The-World%E2%80%99s-First-Agnostic-Operating-System-for-Robots), [VW + Isaac Sim](https://www.nvidia.com/en-us/case-studies/volkswagen-with-wandelbots-nova-and-isaac-sim/).

### Others
Intrinsic (Google) Flowstate — upstream platform play, opaque GA status; Standard Bots / RobCo — vertically integrated arm OEMs. Watch, don't chase.

## 2. Middleware / data plane

- **ROS 2 (2026)**: healthy — Lyrical Luth (May 2026, Ubuntu 26.04), Kilted/Jazzy current LTS line; ros2_control mature (lifecycle + hardware abstraction, chained controllers), MoveIt 2 maintained. But: distro cadence coupling, DDS config pain at scale, and a full ROS core would couple our product API to theirs. Sources: [distros](https://docs.ros.org/en/kilted/Releases.html), [ros2_control release notes](https://control.ros.org/rolling/doc/ros2_control/doc/release_notes.html), [ecosystem 2026](https://robocloud-dashboard.vercel.app/learn/blog/ros2-distributions-2026).
- **Zenoh**: data-centric pub/sub + queries + storage; router topology spans LAN/WAN/constrained links; official `rmw_zenoh` (since Jazzy) and `zenoh-plugin-ros2dds` give first-class ROS 2 interop; production users incl. Woven by Toyota, PX4 support. Sources: [rmw_zenoh](https://github.com/ros2/rmw_zenoh), [ros2dds plugin](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds), [use cases](https://zenoh.io/usecases/), [ROS 2 docs](https://docs.ros.org/en/rolling/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.html), [DDS vs Zenoh 2026](https://iotdigitaltwinplm.com/ros2-dds-vs-zenoh-robotics-middleware-comparison-2026/).
- **Verdict:** Zenoh-native data plane; ROS 2 as bridged ecosystem (drivers, RViz users) not foundation. Revisit at pilot if customers are ROS-native (OQ-2). DDS (CycloneDDS) remains the conservative fallback.

## 3. Motion generation

- **Ruckig**: online, jerk-constrained, time-optimal OTG from arbitrary state; ≤1 ms compute, cycles down to 250 µs; C++17, community version open source (Pro adds waypoint following). The de-facto standard shock absorber between commands and RT streaming. Sources: [repo](https://github.com/pantor/ruckig), [ruckig.com](https://ruckig.com/), [paper](https://arxiv.org/pdf/2105.04830).
- **NVIDIA cuMotion / Isaac ROS**: GPU collision-free optimal-time planning, MoveIt 2 integration, ROS 2 actions/services; needs CUDA (Jetson/x86+GPU). Sources: [isaac_ros_cumotion docs](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion/index.html), [repo](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion).
- **MoveIt 2 / TOPP-RA**: planning + retiming references; heavier stack.
- **Verdict:** Ruckig behind the Tier C slot as default OTG; our value-add = blending/retiming across the mixed node chain + governor layer (Vention's exact gaps). cuMotion later as optional planner on GPU hardware. `trajectory-lab` (in-repo) seeds the blend/profile layer and tests.

## 4. Fieldbus & vendor protocols

- **EtherCAT masters**: IgH/EtherLab (kernel module, PREEMPT_RT/Xenomai, the LinuxCNC path), SOEM (user-space lib, permissive license, CoE/FoE/SoE + distributed clocks), Acontis EC-Master (commercial, support + tooling). CiA 402 = the drive profile either way. Sources: [SOEM](https://github.com/openethercatsociety/soem), [IgH via Intel ECI](https://eci.intel.com/docs/3.3/components/ethercat.html), [comparison](https://www.acontis.com/en/ethercat-master-options-ec-master-vs-open-source-etherlab-SOEM.html), [open CiA402 impl](https://github.com/kubabuda/ecat_servo).
- **Arm streaming interfaces** (from prior research, verified): UR RTDE 500 Hz state + `servoj` streaming; Fanuc Stream Motion (J519) UDP 8 ms reply-or-fault; ABB EGM ~4 ms UDP/protobuf. These set our adapter subcycle requirements.
- **Verdict:** IgH for v0 bench axis (deterministic, proven); SOEM acceptable for user-space prototyping; Acontis = commercial de-risk option. UR adapter first (URSim testable without hardware — `ur-stream-playground` in-repo is the seed).

## 5. User-algorithm sandboxing (Tier B)

- **WASM 2026**: production-normal for plugin systems (67 % production use in 2026 survey; Envoy-style plugins, edge FaaS). Wasmtime = leading runtime, capability-based security (explicit grants only), Component Model gives typed, composable interfaces — exactly a plugin contract. WasmEdge notable for edge/AI extensions. Sources: [state of Wasm 2026](https://devstarsj.github.io/webdev/2026/02/02/WebAssembly-Wasm-2026-Guide/), [ecosystem tools](https://reintech.io/blog/webassembly-ecosystem-2026-tools-frameworks-runtimes), [server-side + WASI 0.2](https://zeonedge.com/blog/webassembly-server-side-wasm-wasi-component-model-2026).
- **Containers**: needed anyway for heavy/GPU perception; gVisor/kata if isolation must harden.
- **Verdict:** both — WASM components (wasmtime) for logic-weight algorithms (fast cold start, tight capability grants), OCI for heavy ones; identical gRPC `CellClient` surface so the manifest, not the packaging, is the contract (resolves OQ-4).

## 6. Simulation

- **Gazebo (Harmonic/Ionic line)**: open-source baseline, best ROS 2 integration, multiple physics engines, zero licensing, CPU-friendly — right for CI sim gates and twins.
- **Isaac Sim**: photoreal + synthetic data + RL standard in 2026, but heavyweight (Omniverse + big GPU) — wrong default for a per-cell twin, right later for perception data.
- Sources: [2026 perspective](https://www.blackcoffeerobotics.com/blog/which-robot-simulation-software-to-use), [MuJoCo/Isaac/Gazebo comparisons](https://www.trossenrobotics.com/post/robot-arm-simulation-mujoco-isaac-sim-gazebo), [VnRobo overview](https://vnrobo.com/en/blog/sim-series-1-overview).
- **Verdict:** Gazebo-class twin as the sim-gate workhorse (headless, deterministic-enough, CI-friendly); Isaac connector deferred [C].

## 7. Observability / fleet tooling

- **Foxglove**: de-facto robotics visualization + MCAP ecosystem; remote viz/teleop in private beta 2026 → they own the *screen*, not the platform. **Formant**: SaaS fleet ops + teleop, enterprise on-prem only by contract. **Open-RMF + Foxglove ≈ 60 % of Formant** per comparisons. **Transitive**: thin public footprint. Sources: [Formant alternatives 2026](https://vnrobo.com/en/blog/formant-alternatives), [RViz/Foxglove/Rerun](https://www.reduct.store/blog/comparison-rviz-foxglove-rerun), [Foxglove teleop beta](https://foxglove.dev/blog/announcing-remote-visualization-teleoperation-private-beta), [fleet tools top-10](https://www.scmgalaxy.com/tutorials/top-10-robotics-fleet-management-tools-features-pros-cons-comparison/).
- **Verdict:** don't compete with Foxglove — emit MCAP + Foxglove-compatible streams natively (D4); build our own thin fleet/ops UI (it's our product surface), no Formant dependency.

## 8. OS / OTA / device fleet

- **balena**: container-first fleet platform (balenaOS + Supervisor), fastest DX, robotics-marketed; lock-in + hidden limits noted by buyers. **Mender**: open-source A/B OTA with atomic install, deltas, phased rollouts — update-focused, bring-your-own-OS. **Torizon**: Toradex-hardware-centric. Sources: [balena](https://www.balena.io/), [robotics page](https://www.balena.io/industries/robotics), [Torizon vs balena vs Mender](https://www.ics.com/blog/iot-fleet-management-system-torizon-balena-mender), [buyer caveats](https://www.rfp.wiki/cloud-computing/edge-computing-platforms/balena), [fleet strategies](https://docs.roxautomation.com/linux/remote_management/).
- **Verdict:** Mender-style A/B (or Mender itself) under our own control plane; balena pattern (supervisor + per-service containers) informs the coordination-plane layout. RT constraint (PREEMPT_RT kernel, pinned cores) rules out fully managed OSes we can't tune.

## 9. Orchestration / low-code

- **BehaviorTree.CPP** (v4): production-standard reactive task trees in C++, async-friendly, Groot2 editor; ROS-adjacent but standalone. Sources: [behaviortree.dev](https://www.behaviortree.dev/), [repo](https://github.com/behaviortree/behaviortree.cpp), [BTs in industrial automation](https://arxiv.org/html/2404.14030v1).
- **Node-RED**: beloved IIoT low-code glue; single-threaded JS runtime — fine for integration flows, wrong engine for cell programs. Source: [nodered.org](https://nodered.org/), [low-code in automation](https://control.com/technical-articles/node-red-and-a-low-code-approach-to-automation/).
- **Verdict:** BT.CPP as the program engine; our flow UI compiles to it (FR-4.1). Node-RED-style integration flows possible later at the north gateway, not in the control path.

## 10. Enterprise integration

2026 consensus: **OPC UA organizes the data, MQTT (Sparkplug B) moves it** — complementary, both now natively supported by AWS/Azure/GCP/Siemens/Beckhoff/Rockwell. Sources: [FlowFuse 2026](https://flowfuse.com/blog/2026/01/opcua-vs-mqtt/), [Sparkplug vs OPC UA](https://www.iotforall.com/a-comparison-of-iiot-protocols-mqtt-sparkplug-vs-opc-ua), [manufacturing view](https://workcell.ai/blog/opc-ua-mqtt-manufacturing).
**Verdict:** per-cell OPC UA server (address space = node tree) + optional Sparkplug B publisher, both read-mostly (FR-9).

---

## 11. Recommended stack (one screen)

| Layer | Pick | Why (short) |
|---|---|---|
| Edge OS | Linux PREEMPT_RT, immutable A/B image | NFR-1 latency + safe OTA |
| Fieldbus | IgH EtherCAT + CiA 402 | kernel-grade determinism, proven |
| Arm adapters | UR RTDE/servoj → Fanuc J519 → ABB EGM | testable via URSim first |
| OTG | Ruckig (community) in Tier C slot | SOTA, replaceable = dogfoods plugin API |
| Blending/retiming | in-house (seeded by trajectory-lab) | the differentiator Vention lacks |
| Data plane | Zenoh (+ ROS 2 bridge) | edge-to-cloud one protocol, ROS interop kept |
| Recording | MCAP + Foxglove compat | ecosystem leverage, D4 |
| Tier B sandbox | wasmtime components + OCI | typed contracts + heavy-workload escape |
| Program engine | BehaviorTree.CPP under flow UI | reactive, debuggable, no capability cliff |
| Twin | Gazebo-class headless | CI-able sim gate |
| OTA | Mender-style A/B + container channels | atomic, staged, self-hostable |
| North | OPC UA + Sparkplug B | 2026 factory consensus |
| Cloud | K8s + Postgres + object store + gRPC/REST | boring on purpose |

**Biggest open bets:** Zenoh-vs-ROS-core (OQ-2), Tier C safety story (certification path), x86-vs-Jetson reference hardware (OQ-1).
