# robonode::motion — RT-domain motion module: governor, plans, executives,
# OTG slot, and the AxisAdapter seam. trajlib (trajectory-lab) and ruckig
# (ARCHITECTURE.md pin v0.17.3) are this module's implementation details: their
# include paths leak (header-only module) but the boundary lint forbids
# anything outside motion/ from including them directly, and neither
# appears in motion's public signatures.
include(FetchContent)
FetchContent_Declare(ruckig
  GIT_REPOSITORY https://github.com/pantor/ruckig.git
  GIT_TAG v0.17.3)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
# Upstream's own test binary trips ASan in CI (their code, their tests);
# our sanitizer job covers OUR use of ruckig through motion_tests.
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
# Cloud client = Ruckig Pro remote waypoints — never wanted on a controller.
set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(ruckig)

add_library(robonode_motion INTERFACE)
add_library(robonode::motion ALIAS robonode_motion)
target_include_directories(robonode_motion INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory-lab/include
)
target_link_libraries(robonode_motion INTERFACE robonode::core ruckig::ruckig)
