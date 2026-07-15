# Per-module test binaries — a test binary's include set doubles as a
# living statement of its module's dependencies. robonode::warnings applies
# our -Wall -Wextra to the test TU (which compiles the header-only modules).
add_executable(motion_tests motion_tests.cpp)
target_link_libraries(motion_tests PRIVATE robonode::motion robonode::warnings)
add_test(NAME motion_tests COMMAND motion_tests)

# Profile math (trapezoid/S-curve) through motion's public MotionPlan surface.
add_executable(profile_tests profile_tests.cpp)
target_link_libraries(profile_tests PRIVATE robonode::motion robonode::warnings)
add_test(NAME profile_tests COMMAND profile_tests)

add_executable(recorder_tests recorder_tests.cpp)
target_link_libraries(recorder_tests PRIVATE robonode::recorder robonode::warnings)
add_test(NAME recorder_tests COMMAND recorder_tests)

add_executable(celld_tests celld_tests.cpp)
target_link_libraries(celld_tests PRIVATE robonode::celld robonode::warnings)
target_compile_definitions(celld_tests PRIVATE
    ROBONODE_IDL_EXAMPLES="${CMAKE_SOURCE_DIR}/robonode-idl/examples"
    ROBONODE_CELL="${CMAKE_SOURCE_DIR}/apps/cell_server/cells/ur10e.cell.json")
add_test(NAME celld_tests COMMAND celld_tests)

if(ROBONODE_BUILD_MUJOCO)
  add_executable(mujoco_tests mujoco_tests.cpp)
  target_link_libraries(mujoco_tests PRIVATE robonode::sim_mujoco robonode::warnings)
  target_compile_definitions(mujoco_tests PRIVATE ROBONODE_WORLDS="${ROBONODE_WORLDS}")
  add_test(NAME mujoco_tests COMMAND mujoco_tests)

  # 7-DOF arm: celld + sim_mujoco integration (one shared world).
  add_executable(arm_tests arm_tests.cpp)
  target_link_libraries(arm_tests PRIVATE robonode::celld robonode::sim_mujoco robonode::warnings)
  target_compile_definitions(arm_tests PRIVATE ROBONODE_WORLDS="${ROBONODE_WORLDS}")
  add_test(NAME arm_tests COMMAND arm_tests)

  # Cartesian layer: FK/IK/moveL on the MuJoCo model (kinematic, servo-free).
  add_executable(kinematics_tests kinematics_tests.cpp)
  target_link_libraries(kinematics_tests PRIVATE robonode::sim_mujoco robonode::warnings)
  target_compile_definitions(kinematics_tests PRIVATE ROBONODE_WORLDS="${ROBONODE_WORLDS}")
  add_test(NAME kinematics_tests COMMAND kinematics_tests)

  # Integration: the CellGateway seam (command JSON in → telemetry JSON out),
  # the same contract the web UI and the CLI (#43) bind to. gateway needs
  # MuJoCo (it is only added under the same guard in the root CMakeLists).
  if(TARGET robonode::gateway)
    add_executable(gateway_integration_tests gateway_integration_tests.cpp)
    target_link_libraries(gateway_integration_tests PRIVATE robonode::gateway robonode::warnings)
    target_compile_definitions(gateway_integration_tests PRIVATE
        ROBONODE_WORLDS="${ROBONODE_WORLDS}"
        ROBONODE_CELL="${CMAKE_SOURCE_DIR}/apps/cell_server/cells/ur10e.cell.json")
    add_test(NAME gateway_integration_tests COMMAND gateway_integration_tests)
  endif()
endif()
