# robonode::adapter_ur — UR vendor adapter behind the AxisAdapter seam.
# Owns the ur_client_library dependency (STACK.md pin 2.13.0). Built as part
# of the robonode package (root CMakeLists), gated by
# ROBONODE_BUILD_UR_ADAPTER.
include(FetchContent)
FetchContent_Declare(
  ur_client_library
  GIT_REPOSITORY https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
  # 1.6.0 included Linux-only <endian.h> and cannot build on macOS; 2.x has
  # a portable socket layer.
  GIT_TAG 2.13.0
)
FetchContent_MakeAvailable(ur_client_library)

# In-tree (FetchContent) alias is ur_client_library::urcl; the installed
# package exports urcl::urcl.
add_library(robonode_adapter_ur INTERFACE)
add_library(robonode::adapter_ur ALIAS robonode_adapter_ur)
target_include_directories(robonode_adapter_ur INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robonode_adapter_ur INTERFACE robonode::motion ur_client_library::urcl)

add_executable(ur_governed_move src/ur_governed_move.cpp)
target_link_libraries(ur_governed_move PRIVATE robonode::adapter_ur robonode::recorder)
target_compile_definitions(ur_governed_move PRIVATE
    URCL_RESOURCES="${ur_client_library_SOURCE_DIR}")
