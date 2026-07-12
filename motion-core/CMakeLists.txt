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

add_executable(robonode_dev src/robonode_dev.cpp)
target_link_libraries(robonode_dev PRIVATE motion_core)

enable_testing()
add_executable(motion_core_tests test/test_motion_core.cpp)
target_link_libraries(motion_core_tests PRIVATE motion_core)
add_test(NAME motion_core_tests COMMAND motion_core_tests)
