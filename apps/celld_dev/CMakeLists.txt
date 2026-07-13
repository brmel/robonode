add_executable(celld_dev main.cpp)
target_link_libraries(celld_dev PRIVATE robonode::celld robonode::recorder robonode::warnings)
