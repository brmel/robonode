# robonode::core — the type vocabulary every module speaks. Depends on
# nothing (stdlib only); everything depends on it. Boundary lint enforces
# that core headers include only core headers.
add_library(robonode_core INTERFACE)
add_library(robonode::core ALIAS robonode_core)
target_include_directories(robonode_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
