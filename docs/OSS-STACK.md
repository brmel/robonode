# Open-source building blocks — reuse, don't reinvent

> The platform is the **clean, modular glue + swap/test experience**, not a from-scratch physics/kinematics/vision engine. This catalogs the mature open-source repos we reuse (✅ in the code now), plan to reuse (▶ candidate, has an issue), or evaluated and set aside (⏸ with the trade-off). Written for external review — critique the choices (e.g. "why not ROS 2 / MoveIt?").

## Physics / simulation
| Repo | Role | Status |
|---|---|---|
| [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco) | Physics engine (the twin) | ✅ in `sim-mujoco` |
| [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) | Real robot MJCF models (UR, Franka, Kuka, …) | ▶ #20 |
| [gazebosim/gz-sim](https://github.com/gazebosim/gz-sim) | Alt sim (Linux/CI twin) | ⏸ behind the same `AxisAdapter` seam |
| [bulletphysics/bullet3](https://github.com/bulletphysics/bullet3) (PyBullet), [isaac-sim](https://developer.nvidia.com/isaac/sim) | Alt physics / GPU sim + synthetic data | ⏸ later (perception RL) |

## Kinematics / dynamics / robotics toolboxes
| Repo | Role | Status |
|---|---|---|
| [petercorke/robotics-toolbox-python](https://github.com/petercorke/robotics-toolbox-python) | Real robot models + FK/IK/Jacobian/trajectories | ✅ behind Kinematics/Planner seams (`rtb-kinematics`) |
| [stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio) | Fast rigid-body dynamics/kinematics (C++) | ▶ candidate for a C++-native Kinematics impl |
| [google-deepmind/dm_control](https://github.com/google-deepmind/dm_control) | MuJoCo Python control/PyMJCF | ⏸ reference |
| [robot-descriptions/robot_descriptions.py](https://github.com/robot-descriptions/robot_descriptions.py) | Fetch 185+ robot models (URDF/MJCF) ready | ▶ #21/#25 |

## Motion planning / trajectory
| Repo | Role | Status |
|---|---|---|
| [pantor/ruckig](https://github.com/pantor/ruckig) | Online jerk-limited trajectory (OTG) | ✅ in `motion` |
| [ompl/ompl](https://github.com/ompl/ompl) | Sampling-based motion planning (RRT/PRM) | ▶ collision-aware Planner impl (Phase 4) |
| [moveit/moveit2](https://github.com/moveit/moveit2) | Full manipulation planning stack (ROS 2) | ⏸ **candidate — feedback wanted.** Heavy/ROS-coupled; we chose RTB + a Planner seam for a lean core. OMPL (its planner core) is the lighter path. |
| [hungpham2511/toppra](https://github.com/hungpham2511/toppra) | Time-optimal path parameterization | ▶ retiming reference |

## Robot control / drivers (real hardware)
| Repo | Role | Status |
|---|---|---|
| [UniversalRobots/Universal_Robots_Client_Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) | UR RTDE/servoj control | ✅ `adapters/ur` |
| [ros-controls/ros2_control](https://github.com/ros-controls/ros2_control) | Real-time controller framework | ⏸ **candidate — feedback wanted.** Our `AxisAdapter`+governor+executive cover this without ROS; ros2_control is the standard alternative. |
| [FANUC-CORPORATION/fanuc_description](https://github.com/FANUC-CORPORATION/fanuc_description), [ros-industrial/fanuc](https://github.com/ros-industrial/fanuc) | FANUC URDF + meshes | ▶ #25 (via RTB URDF) |
| [frankaemika/libfranka](https://github.com/frankaemika/libfranka), [doosan-robotics/doosan-robot](https://github.com/doosan-robotics/doosan-robot), [ros-industrial/abb](https://github.com/ros-industrial/abb) | Vendor drivers/descriptions (Franka/Doosan/ABB) | ▶ per-vendor adapters behind the seam |

## Vision / perception / learning (don't reinvent)
| Repo | Role | Status |
|---|---|---|
| [opencv/opencv](https://github.com/opencv/opencv) | Classical vision (detection, calib, tracking) | ▶ #6/#32 (Vision capability) |
| [microsoft/onnxruntime](https://github.com/microsoft/onnxruntime) | Run trained DL models (portable) | ▶ #32 (DL detector slot) |
| [pytorch/pytorch](https://github.com/pytorch/pytorch), [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) (YOLO) | Training / detection models | ▶ bring-your-own model |
| [NVlabs/FoundationPose](https://github.com/NVlabs/FoundationPose), [google/mediapipe](https://github.com/google-ai-edge/mediapipe) | 6-DoF pose / perception building blocks | ⏸ candidate detectors |

## Middleware / comms (node ↔ node, cloud)
| Repo | Role | Status |
|---|---|---|
| [yhirose/cpp-httplib](https://github.com/yhirose/cpp-httplib) | HTTP+SSE gateway (v0 transport) | ✅ `gateway` |
| [eclipse-zenoh/zenoh](https://github.com/eclipse-zenoh/zenoh) | Edge↔cloud pub/sub data plane | ▶ #7 |
| [ros2/ros2](https://github.com/ros2/ros2) | Full robotics middleware + ecosystem | ⏸ **candidate — feedback wanted.** We chose Zenoh-native + our own IDL for a lean, un-coupled core; ROS 2 is bridged (rmw_zenoh) rather than the foundation. |
| [grpc/grpc](https://github.com/grpc/grpc) | Typed RPC (SDKs/UI/Tier-B) | ⏸ #8 deferred (HTTP/SSE covers v0) |

## Telemetry / visualization
| Repo | Role | Status |
|---|---|---|
| [foxglove/mcap](https://github.com/foxglove/mcap) | Recording (flight recorder) | ✅ `recorder` |
| [foxglove/foxglove-sdk](https://github.com/foxglove/foxglove-sdk) | Live viz + MCAP | ▶ #11 |
| [rerun-io/rerun](https://github.com/rerun-io/rerun) | Multimodal viz | ⏸ alt |
| [mrdoob/three.js](https://github.com/mrdoob/three.js), [gkjohnson/urdf-loaders](https://github.com/gkjohnson/urdf-loaders) | Web 3D twin | ✅ web app / ▶ real meshes |

## User-module sandbox (bring your own, safely)
| Repo | Role | Status |
|---|---|---|
| [bytecodealliance/wasmtime](https://github.com/bytecodealliance/wasmtime) | WASM runtime for user algorithms | ▶ #26 |
| OCI/containerd, [gvisor](https://github.com/google/gvisor) | Heavier/GPU user modules, isolation | ▶ Tier-B (containers) |

## Fleet / deploy / OS
| Repo | Role | Status |
|---|---|---|
| [mendersoftware/mender](https://github.com/mendersoftware/mender) | A/B OTA updates | ▶ later |
| [balena-os](https://github.com/balena-os) | Container fleet pattern | ⏸ reference |

---

### Stance summary (for critique)
We deliberately built a **lean C++ core with clean seams** and reuse best-in-class engines behind them, rather than adopting the full **ROS 2 / MoveIt / ros2_control** stack up front. Rationale: avoid coupling the product API + node model to ROS distro cadence, keep the RT loop small and auditable, and let ROS be **bridged** (rmw_zenoh) for teams that want it. Open question for reviewers: **is the lean-core-with-bridges bet right, or should MoveIt/ros2_control/ROS 2 be first-class from the start?** Exact pinned versions of what's already in the build: [STACK.md](STACK.md).
