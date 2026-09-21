# robonode::recorder — telemetry → MCAP. Owns the mcap dependency; depends
# on robonode::core only (TelemetryRow is the cross-module currency — the
# recorder knows nothing of adapters, plans, or executives).
#
# mcap is fetched as a tarball, not a git clone: the repo carries git-lfs
# assets plain clones can't fetch (ARCHITECTURE.md). Header-only; this module's
# single TU defines MCAP_IMPLEMENTATION; compression off = zero extra deps.
include(FetchContent)
FetchContent_Declare(mcap
  URL https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v2.1.3.tar.gz)
FetchContent_Populate(mcap)

add_library(robonode_recorder STATIC src/mcap_impl.cpp)
add_library(robonode::recorder ALIAS robonode_recorder)
target_include_directories(robonode_recorder PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${mcap_SOURCE_DIR}/cpp/mcap/include
)
target_compile_definitions(robonode_recorder PUBLIC MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD)
target_link_libraries(robonode_recorder PUBLIC robonode::core)
