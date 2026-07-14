cmake_minimum_required(VERSION 3.20)
project(robonode CXX)

# One package, MIL-style modules. Consumers link components:
#   robonode::core      — types every module speaks (State, Limits, Status,
#                         Lifecycle, Telemetry); depends on nothing
#   robonode::motion    — governor, plans, executives, OTG slot (ruckig),
#                         AxisAdapter seam
#   robonode::recorder  — telemetry → MCAP (owns the mcap dependency)
#   robonode::celld     — descriptors → node tree → lifecycle → coherent
#                         execution (owns the JSON dependency; vendor-blind)
#   robonode::sim_mujoco — physics twin (owns the MuJoCo dependency)
#   robonode::adapter_ur — UR vendor adapter (owns the urcl dependency)
# Module boundaries are enforced structurally: scripts/check-boundaries.sh
# runs in CI and locally.

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Our warnings apply to OUR code only, never to vendored deps — a global
# add_compile_options() leaks -Wextra into MuJoCo/urcl/etc, and MuJoCo's own
# -Werror then rejects it. Link robonode::warnings into targets that compile
# our .cpp; header-only modules are checked via the app/test that includes them.
add_library(robonode_warnings INTERFACE)
add_library(robonode::warnings ALIAS robonode_warnings)
target_compile_options(robonode_warnings INTERFACE -Wall -Wextra)

option(ROBONODE_BUILD_UR_ADAPTER "Build the UR vendor adapter (fetches ur_client_library)" ON)
option(ROBONODE_BUILD_MUJOCO "Build the MuJoCo physics twin (fetches + builds MuJoCo, heavy)" ON)

enable_testing()

add_subdirectory(core)
add_subdirectory(motion)
add_subdirectory(recorder)
add_subdirectory(celld)
# sim-mujoco before apps/tests that link it (also exports ROBONODE_WORLDS).
if(ROBONODE_BUILD_MUJOCO)
  add_subdirectory(sim-mujoco)
endif()
add_subdirectory(apps/robonode_dev)
add_subdirectory(apps/celld_dev)
if(ROBONODE_BUILD_MUJOCO)
  add_subdirectory(apps/mujoco_dev)
  add_subdirectory(apps/arm_dev)
endif()
add_subdirectory(tests)
if(ROBONODE_BUILD_UR_ADAPTER)
  add_subdirectory(adapters/ur)
endif()
