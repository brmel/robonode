add_library(robonode_sandbox INTERFACE)
add_library(robonode::sandbox ALIAS robonode_sandbox)
target_include_directories(robonode_sandbox INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robonode_sandbox INTERFACE robonode::core)
