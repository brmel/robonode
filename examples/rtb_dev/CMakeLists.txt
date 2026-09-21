add_executable(rtb_dev main.cpp)
target_link_libraries(rtb_dev PRIVATE robonode::rtb robonode::warnings)
