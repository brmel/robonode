# robonode::gateway — the live web-facing bridge (SSE/HTTP). Owns the
# cpp-httplib dependency (boundary: httplib private to gateway + its server
# app). The CellGateway bridge itself is transport-free (celld + sim_mujoco +
# json); the embedded HTTP server lives in the cell_server app.
include(FetchContent)
FetchContent_Declare(httplib
  GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
  GIT_TAG v0.19.0
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(httplib)

# spdlog — structured async logging (mature, not reinvented). Header-only so
# it stays a private impl detail of the gateway/apps (the non-RT surfaces).
FetchContent_Declare(spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG v1.15.1
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(spdlog)

add_library(robonode_gateway INTERFACE)
add_library(robonode::gateway ALIAS robonode_gateway)
target_include_directories(robonode_gateway INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robonode_gateway INTERFACE
    robonode::celld robonode::sim_mujoco robonode::vision robonode::recorder
    httplib::httplib spdlog::spdlog_header_only)
