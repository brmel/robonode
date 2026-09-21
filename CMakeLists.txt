cmake_minimum_required(VERSION 3.20)
project(robonode VERSION 0.8.0 LANGUAGES CXX)

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

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Our warnings apply to OUR code only, never to vendored deps — a global
# add_compile_options() leaks -Wextra into MuJoCo/urcl/etc, and MuJoCo's own
# -Werror then rejects it. Link robonode::warnings into targets that compile
# our .cpp; header-only modules are checked via the app/test that includes them.
add_library(robonode_warnings INTERFACE)
add_library(robonode::warnings ALIAS robonode_warnings)
# A warning nobody reads is a bug nobody reads. `Status` is [[nodiscard]] and a
# wrapper still managed to discard one for a whole commit — the compiler said
# so, into a log the build filter dropped. Warnings are errors in our own code;
# vendored deps keep their own flags because this target is never linked into
# them.
target_compile_options(robonode_warnings INTERFACE -Wall -Wextra -Werror)

option(ROBONODE_BUILD_UR_ADAPTER "Build the UR vendor adapter (fetches ur_client_library)" ON)
option(ROBONODE_BUILD_MUJOCO "Build the MuJoCo physics twin (fetches + builds MuJoCo, heavy)" ON)
option(ROBONODE_BUILD_RTB "Build the Robotics Toolbox bridge (real robot kinematics via a Python service)" OFF)
option(ROBONODE_BUILD_WASM "Build the Wasmtime sandbox engine (downloads the C-API, real untrusted-code)" OFF)
option(ROBONODE_BUILD_OPENCV "Build the OpenCV vision node (needs a local OpenCV)" ON)

# A contributor working on the UI, the CLI or the motion core should not have to
# install OpenCV to build the tree. If it is not there, say so once and carry on
# without the real detector — the toy versions still exercise the seam.
if(ROBONODE_BUILD_OPENCV)
  find_package(OpenCV QUIET COMPONENTS core imgproc)
  if(NOT OpenCV_FOUND)
    message(STATUS "OpenCV not found — building without the OpenCV vision node. "
                   "Install it (brew install opencv / apt install libopencv-dev) "
                   "to get robonode.opencv, or configure with -DROBONODE_BUILD_OPENCV=OFF "
                   "to silence this.")
    set(ROBONODE_BUILD_OPENCV OFF)
  endif()
endif()

enable_testing()

# What this configuration actually builds. A contributor's first question is
# "why is the thing I wanted missing" — answer it at configure time.
message(STATUS "RoboNode ${PROJECT_VERSION} — physics twin (MuJoCo): ${ROBONODE_BUILD_MUJOCO}")
message(STATUS "                            OpenCV vision node:      ${ROBONODE_BUILD_OPENCV}")
message(STATUS "                            UR vendor adapter:       ${ROBONODE_BUILD_UR_ADAPTER}")
message(STATUS "                            Robotics Toolbox bridge: ${ROBONODE_BUILD_RTB}")
message(STATUS "                            Wasmtime sandbox:        ${ROBONODE_BUILD_WASM}")

add_subdirectory(core)
add_subdirectory(motion)
add_subdirectory(recorder)
add_subdirectory(celld)
add_subdirectory(sandbox)
add_subdirectory(vision)
if(ROBONODE_BUILD_WASM)
  add_subdirectory(engines/wasm)
endif()
if(ROBONODE_BUILD_OPENCV)
  add_subdirectory(engines/opencv)
endif()
# sim-mujoco before apps/tests that link it (also exports ROBONODE_WORLDS).
if(ROBONODE_BUILD_MUJOCO)
  add_subdirectory(engines/sim-mujoco)
endif()
if(ROBONODE_BUILD_UR_ADAPTER)
  add_subdirectory(adapters/ur)
endif()
if(ROBONODE_BUILD_MUJOCO)
  add_subdirectory(gateway)
  add_subdirectory(apps/support)
  add_subdirectory(apps/cell_server)
  add_subdirectory(apps/robonode_cli)
endif()
if(ROBONODE_BUILD_RTB)
  add_subdirectory(engines/rtb)
endif()
add_subdirectory(tests)
add_subdirectory(examples)
