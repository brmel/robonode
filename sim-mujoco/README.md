# robonode::sim_mujoco

The physics twin ([issue #2](https://github.com/brmel/robonode/issues/2)). Turns the `AxisAdapter` seam into real dynamics: a MuJoCo world driven by the same governor/executive/celld that drive `SimAxis`, so following error, servo tracking, and blend behaviour become physical instead of a first-order filter.

```
MujocoWorld (owns mjModel+mjData, one mj_step per cycle)
   └─ MujocoAxisAdapter  (one joint+actuator; live read from mjData)
         registered as "robonode.mujoco-axis" → celld builds it like any node
```

## Why MuJoCo (and where Gazebo stays)

Local dev is Apple-silicon, where Gazebo-on-macOS-arm64 is effectively unsupported and we already lived the URSim-under-qemu pain (STACK.md). MuJoCo runs native arm64 and headless (physics needs no GL), so it is the **local dev twin**. Gazebo remains the Linux/CI twin — both sit behind the same `AxisAdapter` seam, so the motion core never learns which is underneath. That is the modularity paying rent.

## Build

Gated by `ROBONODE_BUILD_MUJOCO` (default ON) because MuJoCo is FetchContent-built from source (`3.10.0`, shallow). First configure clones MuJoCo + a few small deps; cached in `_deps` afterwards. Sanitizer CI turns it off (sanitizing MuJoCo is not our job, same as urcl).

```sh
cmake -B build && cmake --build build -j
./build/apps/mujoco_dev/mujoco_dev      # S-curve on a physical rail, MCAP out
ctest --test-dir build -R mujoco_tests
```

## Units & stepping

- The descriptor speaks its own units (mm for a rail, rad for a joint); MuJoCo speaks SI. `units_per_m` in the driver config is the descriptor-unit-per-metre scale (1000 for mm). The adapter is the unit boundary.
- One world may back many joints (an arm, #3). Exactly one adapter is the `clock_owner` and calls `mj_step`; `SyncExecutive` phases write→step→read so every setpoint is in place before that single step. `MujocoWorld::step` accumulates a residual so sim time tracks the control clock whatever the timestep/rate ratio.

## Worlds

- `rail.xml` — single linear axis (#2), gravity real; the 65 mm physical following error comes from inertia.
- `rail_ur10e.xml` — rail + UR10e-parameterised 6R arm (#3), 7 DOF sharing one world. UR10e link lengths (`d1,a2,a3,d4,d5,d6`), primitive capsule geometry, gravity-compensated links, self-collision off (a control/coordination twin; collision-aware planning uses a separate collision world, #5). **Servo gains are nominal** — the platform drives every commanded (governed) setpoint to the exact target on one clock; the joints' physical tracking to within ~0.1 rad is a realistic following error, not a defect. Tightening it is inner-loop control tuning, orthogonal to coordination, and #4 cross-checks kinematics against MuJoCo's FK regardless.

## Arm: shared world

`MujocoWorldPool` shares one `MujocoWorld` across every adapter that names the same MJCF path, so the arm's 7 descriptors (one `world`) drive a single physics body; the first node is the clock owner (steps physics). celld keeps its per-node model and stays vendor-blind. `apps/arm_dev` runs the 7-DOF coordinated blended program; `tests/arm_tests` proves one-clock + governed-exact + a protective stop on one joint holding all 7 (cell-coherent safety).

## Kinematics (#4)

`MujocoKinematics` implements the motion `Kinematics` seam (`fk`, `position_jacobian`) on a private scratch world, so FK/IK queries never disturb the live sim. The Cartesian layer (`motion/cartesian.hpp`: DLS `ik_position`, `plan_move_l`, `cartesian_jog_velocity`) speaks only that interface — no MuJoCo — and emits joint waypoints that feed the existing SyncBlendPlan → governor pipeline. Cross-checks are kinematic (servo-independent): the Jacobian matches finite-difference to 1e-4, IK converges on reachable targets, moveL traces a straight TCP line within 2 mm.

## Boundary

`mujoco.h` is private to this module (lint: no `#include <mujoco/...>` anywhere else). Everything above sees only `AxisAdapter` + core types.
