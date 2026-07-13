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

## Boundary

`mujoco.h` is private to this module (lint: no `#include <mujoco/...>` anywhere else). Everything above sees only `AxisAdapter` + core types.
