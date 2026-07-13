# robonode::sim_mujoco — the physics twin. Owns the MuJoCo dependency
# (boundary lint: mujoco.h private to this module). Built from source via
# FetchContent so it is portable across macOS-arm64 (local dev twin) and
# linux (CI); gated by ROBONODE_BUILD_MUJOCO because the source build is
# heavy. Worlds live in worlds/; ROBONODE_WORLDS points apps at them.
include(FetchContent)
FetchContent_Declare(mujoco
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
  GIT_TAG 3.10.0
  GIT_SHALLOW TRUE)
set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(mujoco)

add_library(robonode_sim_mujoco INTERFACE)
add_library(robonode::sim_mujoco ALIAS robonode_sim_mujoco)
target_include_directories(robonode_sim_mujoco INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robonode_sim_mujoco INTERFACE robonode::core robonode::motion mujoco::mujoco)

set(ROBONODE_WORLDS "${CMAKE_CURRENT_SOURCE_DIR}/worlds" CACHE INTERNAL "MuJoCo worlds dir")
