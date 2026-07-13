cmake_minimum_required(VERSION 3.20)
project(robonode CXX)

# One package, MIL-style modules. Consumers link components:
#   robonode::core      — types every module speaks (State, Limits, Status,
#                         Lifecycle, Telemetry); depends on nothing
#   robonode::motion    — governor, plans, executives, AxisAdapter seam
#   robonode::recorder  — telemetry → MCAP (owns the mcap dependency)
#   robonode::adapter_ur — UR vendor adapter (owns the urcl dependency)
# Module boundaries are enforced structurally: scripts/check-boundaries.sh
# runs in CI and locally.

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

option(ROBONODE_BUILD_UR_ADAPTER "Build the UR vendor adapter (fetches ur_client_library)" ON)

enable_testing()

add_subdirectory(core)
add_subdirectory(motion)
add_subdirectory(recorder)
add_subdirectory(apps/robonode_dev)
add_subdirectory(tests)
if(ROBONODE_BUILD_UR_ADAPTER)
  add_subdirectory(adapters/ur)
endif()
