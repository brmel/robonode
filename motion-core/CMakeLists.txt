cmake_minimum_required(VERSION 3.20)
project(robonode-motion-core CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

# Header-only core; trajlib (trajectory-lab) provides the profile math seed.
add_library(motion_core INTERFACE)
target_include_directories(motion_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory-lab/include
)

# mcap (flight recorder, STACK.md pin). Tarball, not git: the repo carries
# git-lfs assets plain clones can't fetch. Header-only; one TU defines
# MCAP_IMPLEMENTATION; compression disabled = zero further deps.
include(FetchContent)
FetchContent_Declare(mcap
  URL https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v2.1.3.tar.gz)
FetchContent_Populate(mcap)

add_library(mcap_impl STATIC src/mcap_impl.cpp)
target_include_directories(mcap_impl PUBLIC ${mcap_SOURCE_DIR}/cpp/mcap/include)
target_compile_definitions(mcap_impl PUBLIC MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD)

add_executable(robonode_dev src/robonode_dev.cpp)
target_link_libraries(robonode_dev PRIVATE motion_core mcap_impl)

enable_testing()
add_executable(motion_core_tests test/test_motion_core.cpp)
target_link_libraries(motion_core_tests PRIVATE motion_core mcap_impl)
add_test(NAME motion_core_tests COMMAND motion_core_tests)
