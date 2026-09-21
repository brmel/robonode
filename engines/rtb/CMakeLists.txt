# robonode::rtb — the Robotics Toolbox bridge. Implements the motion
# Kinematics/Planner seams against the Python rtb-kinematics service (real
# robot models: UR3/5/10, Panda, …). Owns the httplib client + json deps
# (both header-only; deduped with the gateway/celld declarations). No MuJoCo,
# no vendor comms — just the seam impls + an HTTP client.
include(FetchContent)
FetchContent_Declare(httplib
  GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
  GIT_TAG v0.19.0
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(httplib)
if(NOT TARGET nlohmann_json::nlohmann_json)
  FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
  FetchContent_MakeAvailable(nlohmann_json)
endif()

add_library(robonode_rtb INTERFACE)
add_library(robonode::rtb ALIAS robonode_rtb)
target_include_directories(robonode_rtb INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robonode_rtb INTERFACE
    robonode::motion httplib::httplib nlohmann_json::nlohmann_json)
