add_executable(arm_dev main.cpp)
target_link_libraries(arm_dev PRIVATE robonode::celld robonode::sim_mujoco robonode::recorder robonode::warnings)
target_compile_definitions(arm_dev PRIVATE ROBONODE_WORLDS="${ROBONODE_WORLDS}")
