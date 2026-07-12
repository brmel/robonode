cmake_minimum_required(VERSION 3.20)
project(ur_stream_playground CXX)

set(CMAKE_CXX_STANDARD 17)  # ur_client_library targets C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  ur_client_library
  GIT_REPOSITORY https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
  GIT_TAG 1.6.0
)
FetchContent_MakeAvailable(ur_client_library)

# jerk-limited profiles from the sibling project (header-only)
add_library(trajlib INTERFACE)
target_include_directories(trajlib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory-lab/include)

add_executable(read_state src/read_state.cpp)
target_link_libraries(read_state PRIVATE urcl::urcl)

add_executable(stream_joint src/stream_joint.cpp)
target_link_libraries(stream_joint PRIVATE urcl::urcl trajlib)
