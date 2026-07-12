cmake_minimum_required(VERSION 3.20)
project(trajectory_lab CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

add_library(trajlib INTERFACE)
target_include_directories(trajlib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(profile_demo src/profile_demo.cpp)
target_link_libraries(profile_demo PRIVATE trajlib)

add_executable(stream_demo src/stream_demo.cpp)
target_link_libraries(stream_demo PRIVATE trajlib)
find_package(Threads REQUIRED)
target_link_libraries(stream_demo PRIVATE Threads::Threads)

include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.5.2
)
FetchContent_MakeAvailable(Catch2)

add_executable(tests
  test/test_trapezoidal.cpp
  test/test_scurve.cpp
  test/test_blend.cpp
  test/test_spsc_ring.cpp
)
target_link_libraries(tests PRIVATE trajlib Catch2::Catch2WithMain Threads::Threads)

enable_testing()
add_test(NAME unit COMMAND tests)
