# robonode::motion — RT-domain motion module: governor, plans, executives,
# OTG slot, and the AxisAdapter seam. trajlib (trajectory-lab) and ruckig
# (STACK.md pin v0.17.3) are this module's implementation details: their
# include paths leak (header-only module) but the boundary lint forbids
# anything outside motion/ from including them directly, and neither
# appears in motion's public signatures.
include(FetchContent)
FetchContent_Declare(ruckig
  GIT_REPOSITORY https://github.com/pantor/ruckig.git
  GIT_TAG v0.17.3)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(ruckig)

add_library(robonode_motion INTERFACE)
add_library(robonode::motion ALIAS robonode_motion)
target_include_directories(robonode_motion INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory-lab/include
)
target_link_libraries(robonode_motion INTERFACE robonode::core ruckig::ruckig)
