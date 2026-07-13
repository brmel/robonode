add_executable(robonode_dev main.cpp)
target_link_libraries(robonode_dev PRIVATE robonode::motion robonode::recorder robonode::warnings)
