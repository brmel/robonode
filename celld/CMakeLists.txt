# robonode::celld — coordination-plane skeleton: descriptors → node tree →
# lifecycle orchestration → cell-coherent execution. Owns the JSON
# dependency; depends on core + motion (never on vendor adapters — drivers
# arrive through a factory seam so celld stays vendor-blind).
include(FetchContent)
FetchContent_Declare(nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(nlohmann_json)

add_library(robonode_celld INTERFACE)
add_library(robonode::celld ALIAS robonode_celld)
target_include_directories(robonode_celld INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robonode_celld INTERFACE robonode::core robonode::motion nlohmann_json::nlohmann_json)
