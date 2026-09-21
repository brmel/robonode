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

# MuJoCo 3.10 compiles as strict C11, but its sanitizer shim (mjsan.h, active
# only under -fsanitize) spells the barrier `asm` — a GNU extension strict C11
# does not define. Without this the whole ASan/UBSan build is unavailable to us
# on any suite that links physics, which is most of them. Vendored code, their
# bug, one property: turn the extensions their own header needs back on.
if(TARGET mujoco)
  set_target_properties(mujoco PROPERTIES C_EXTENSIONS ON)
    # PRIVATE for their own sources; INTERFACE because anything that includes
  # mujoco.h under a sanitizer pulls the same header, and our -Werror is for
  # our code, not theirs.
  target_compile_options(mujoco PRIVATE -Wno-gcc-compat -Wno-language-extension-token
                                INTERFACE -Wno-gcc-compat -Wno-language-extension-token)
endif()

add_library(robonode_sim_mujoco INTERFACE)
add_library(robonode::sim_mujoco ALIAS robonode_sim_mujoco)
target_include_directories(robonode_sim_mujoco INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robonode_sim_mujoco INTERFACE robonode::core robonode::motion mujoco::mujoco
                      tinyxml2::tinyxml2)

set(ROBONODE_WORLDS "${CMAKE_CURRENT_SOURCE_DIR}/worlds" CACHE INTERNAL "MuJoCo worlds dir")
