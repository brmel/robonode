# Vention Motion API Surface (verified from docs/PyPI/GitHub, June 2026)

THE highest-value research file. Their public SDK tells you what the motion stack does today — and therefore what THIS ROLE builds next.

## machine-logic-sdk 3.x (MachineMotion AI, Python)

- PyPI `machine-logic-sdk` v3.0.1. **Dependencies leak the architecture: `paho-mqtt`, `roslibpy` (ROS bridge!), `vention-firmware-grpc-client` (gRPC — JD requirement confirmed in-product), `python-statemachine` (JD: state machines), numpy/scipy.** Python 3.10 on MMAI devices.
- `Machine` → `get_actuator/get_robot/get_ac_motor/get_scene`, `on_mqtt_event`, `publish_mqtt_event`.
- **Actuator**: `home`, `move_absolute(position, motion_profile)`, `move_relative`, `move_continuous_async` (velocity mode), `stop`, `wait_for_move_completion`; state has `position, speed, output_torque`.
- **`MotionProfile(velocity, acceleration, jerk=optional)`** — **jerk-limited profiles exist in the API** (mm/s³). Trapezoid default.
- **`ActuatorGroup(*actuators)`** — synchronized multi-axis moves (`move_absolute(position_tuple, profile)`).
- **`Robot`** (MMAI only): `movej/movel(±async)`, `compute_forward_kinematics`, `compute_inverse_kinematics(cartesian, joint_constraints, seed)`, `set_active_tcp`, `set_payload(payload, com, inertia)`, freedrive; `RobotState` incl. `safety_state` (NORMAL/ESTOP/REDUCED_SPEED/RECOVERABLE_FAULT); joint data "timestamped the exact moment recorded **in ROS**".
- **`RobotMoveSequence`**: `append_movej/append_movel(target, velocity, acceleration, blend_radius)` → **blending exists for robot moves only. Actuator moves have NO blend parameter.**
- Conveyors: servo conveyors as actuators (`move_continuous_async`); AC/VFD via `ACMotor.move_forward/reverse/stop`.
- Exceptions per subsystem; `Scene` for reference/calibration frames + targets.

## Legacy mm-python-api (MachineMotion v1/v2, GitHub VentionCo)

- `moveToPosition(Combined)`, `moveRelative(Combined)`, `moveContinuous`, global `setSpeed/setAcceleration`, per-axis max speed/accel (v2).
- **G-code path following on v2**: `configPathMode(axisMap, *tools)`, `startPath(path, maxAcceleration, speedOverride, rapidSpeed)`, `getPathStatus`, `stopPath` — CNC-style execution exists in legacy. Raw `emitgCode()` on v1.
- E-stop API: `triggerEstop/releaseEstop/resetSystem/bindeStopEvent`.
- Gantry/multi-motor: `configActuator(config, parentDrive, *childDrives)`; MMAI sync modes "Cyclic Synchronous Torque" (same chain) / "Common Position Target" (cross-chain).
- Actuator families (MECH_GAIN verbatim): timing_belt_150mm_turn, enclosed_timing_belt, ballscrew_10mm_turn, enclosed_ballscrew_16mm, rack_pinion(_v2), indexer(_v2) rotary, roller/belt conveyor, electric_cylinder.

## UR integration today (URCap)

- **UR controller is master**: URCap on the pendant commands MachineMotion over TCP sockets (UR 192.168.5.3 → MM 192.168.5.2). Nodes: Position Move, Move Continuous, Stop, Home, Wait For Completion, IO reads/writes.
- MachineMotion AI inverts this: Vention controller commands the UR via `Robot` class.
- **Coordinated Motion (FABTECH 2023, with UR)**: synchronizes **all 6 UR joints + Vention 7th axis** for multi-waypoint constant-TCP-speed trajectories (pitched at welding). [Robotics 24/7](https://www.robotics247.com/article/vention_and_universal_robots_show_coordinated_motion_technology_at_fabtech)

## What this means for THE ROLE (interview synthesis — memorize)

1. **JD "constant-speed toolpath" = generalizing Coordinated Motion** (2023 UR-only demo) into the MMAI-native stack across brands.
2. **JD "optimal blending waypoint trajectories" = the gap is visible in their API**: robots get `blend_radius`, actuators get nothing, and arm+rail blending doesn't exist publicly. You can SAY this: "Your SDK exposes blend_radius on robot sequences but not on actuator or coordinated arm+rail moves — is unified blending across the kinematic chain part of this role?"
3. **No public servo-streaming API** → UR Client Library / Fanuc Stream Motion integration (JD nice-to-have) = the internal layer that makes coordinated motion possible. Exactly where the role sits.
4. **gRPC + MQTT + ROS + state machines** all confirmed in the product (SDK deps) — every JD networking/software keyword maps to a real component. ROS on controller = ask whether motion core is ROS-based (MoveIt? ros_control?) or ROS is just the bridge.
5. **Jerk field optional and possibly unhonored on v2 hardware** → profile-generation ownership likely moving into the new firmware = this role's code.

## Actuator performance classes (selection guide)

| Actuator | Max speed | Force (typ / w 5:1 gearbox) |
|---|---|---|
| Timing belt | 1250 mm/s | ~125 N / 700 N @250mm/s |
| Rack & pinion | 1250 mm/s | ~150 N / 600 N @300mm/s (modular rack → long strokes) |
| Belt rack | 1250 mm/s | ~125 N / 500 N |
| Enclosed timing belt | 500 mm/s | ~200 N |
| Enclosed ball screw | 75 mm/s | ~2000 N (high force, low speed) |

Belt compliance vs screw stiffness → trajectory limits differ per actuator → limits must be config data, not constants. ([guide](https://docs.vention.io/docs/en/selecting-a-linear-axis-actuator))

## Forum intel (= user-visible motion gaps = team backlog)

Deep forum sweep (Discourse JSON, ~165 topics total, heavily staff-seeded):

**Architecture admissions (staff-authored, quote-able):**
- **Single-threaded, non-blocking event architecture** on the controller — Max Windisch explains actuator "concurrency" as event-loop interleaving; "long active CPU loops will prevent going back to the event-handling state" ([368](https://forum.vention.io/t/concurrency-in-no-code-machinelogic/368)). True synchronized multi-axis interpolation/gantry squaring never discussed.
- External control surface = "MQTT, HTTP, and (more recently) Ethernet/IP" only; EtherNet/IP positioned as "the real-time option" ([451](https://forum.vention.io/t/message-passing-and-interop-with-machinelogic/451), [771](https://forum.vention.io/t/5-types-of-3rd-party-communications-supported-by-vention/771)).
- **Servo PID tuning = warranty-voiding Cloud9 backdoor** (`setTuningV1()`, defaults PKp=4500, VKp=60000) ([770](https://forum.vention.io/t/how-to-change-pid-gain-values/770)). Stall-detection false positives disabled via hidden config edit ([782](https://forum.vention.io/t/how-to-turn-off-stall-detection/782)).
- Firmware v2.13 added low-level **torque-mode REST route** `:8000/smartDrives/motion/torque` + revised actuator max speeds DOWN "to safer values" ([573](https://forum.vention.io/t/firmware-release-of-v2-13-on-mmv2-pendant-release-of-v3-2/573)).

**User pain points:**
- Homing speed barely configurable, version-fragmented workarounds ([578](https://forum.vention.io/t/configuring-homing-speed/578)).
- Jerky motion near actuator performance limits — official fix "reduce speed/accel" → no limit-aware retiming; your TOPP/jerk knowledge is the fix.
- Robot **joint limits** requested by Vention's own engineer, no roadmap ([483](https://forum.vention.io/t/machinelogic-robot-joint-limits/483)).
- Operator mode / boot-launch / e-stop program persistence gaps ([499](https://forum.vention.io/t/put-the-machinemotion-in-operator-mode/499)).
- 2025 user threads going unanswered (MQTT topic mismatch [910], ServiceNow integration [903]).

**The headline absence**: ZERO forum threads on trajectory blending, jerk control, streaming, ROS, EtherCAT, multi-robot sync. Advanced motion is invisible publicly → being built internally → this role. Interview line: "Your public motion surface stops at MoveJ/MoveL + per-move profiles; the JD describes the layer that doesn't exist publicly yet — that's exciting."

## Pendant/stack facts

MMAI = Orin Nano 8GB (AI) / Orin NX 16GB (AI Pro), 2/4 EtherCAT chain ports (max 10 motors/port, datasheet "up to 20", marketing "30"), integrated safety PLC **REER M1S RV, Cat 3, PL e**, RS485 IO to 128 DI+DO, PoE camera ports. Python 3.10 onboard.

Sources: [Python API v3](https://docs.vention.io/docs/vention-python-api) · [PyPI](https://pypi.org/project/machine-logic-sdk/) · [mm-python-api](https://github.com/VentionCo/mm-python-api) · [URCap guide](https://docs.vention.io/docs/machinelogic-for-machinemotion-with-universal-robots-urcap) · [MMAI datasheet](https://docs.vention.io/docs/machinemotion-ai-controller-datasheet) · [Coordinated Motion PR](https://www.prnewswire.com/news-releases/vention-unveils-coordination-motion-technology-at-fabtech-in-collaboration-with-universal-robots-301923595.html)
