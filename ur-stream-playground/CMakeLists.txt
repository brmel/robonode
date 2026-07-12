cmake_minimum_required(VERSION 3.20)
project(ur_stream_playground CXX)

set(CMAKE_CXX_STANDARD 17)  # ur_client_library targets C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  ur_client_library
  GIT_REPOSITORY https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
  # 1.6.0 included Linux-only <endian.h> and cannot build on macOS; 2.x has
  # a portable socket layer.
  GIT_TAG 2.13.0
)
FetchContent_MakeAvailable(ur_client_library)

# jerk-limited profiles from the sibling project (header-only)
add_library(trajlib INTERFACE)
target_include_directories(trajlib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory-lab/include)

# In-tree (FetchContent) alias is ur_client_library::urcl; the installed
# package exports urcl::urcl.
add_executable(read_state src/read_state.cpp)
target_link_libraries(read_state PRIVATE ur_client_library::urcl)

add_executable(stream_joint src/stream_joint.cpp)
target_link_libraries(stream_joint PRIVATE ur_client_library::urcl trajlib)
