cmake_minimum_required(VERSION 3.20)
project(robonode-adapter-ur CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

include(FetchContent)
FetchContent_Declare(
  ur_client_library
  GIT_REPOSITORY https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
  GIT_TAG 2.13.0
)
FetchContent_MakeAvailable(ur_client_library)

FetchContent_Declare(mcap
  URL https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v2.1.3.tar.gz)
FetchContent_Populate(mcap)

# motion-core headers + trajlib (both header-only)
add_library(motion_core INTERFACE)
target_include_directories(motion_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../motion-core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../trajectory-lab/include
)

add_library(mcap_impl STATIC ../../motion-core/src/mcap_impl.cpp)
target_include_directories(mcap_impl PUBLIC ${mcap_SOURCE_DIR}/cpp/mcap/include)
target_compile_definitions(mcap_impl PUBLIC MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD)

add_executable(ur_governed_move src/ur_governed_move.cpp)
target_include_directories(ur_governed_move PRIVATE include)
target_link_libraries(ur_governed_move PRIVATE motion_core mcap_impl ur_client_library::urcl)
target_compile_definitions(ur_governed_move PRIVATE
    URCL_RESOURCES="${ur_client_library_SOURCE_DIR}")
