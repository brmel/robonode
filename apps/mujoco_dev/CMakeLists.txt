add_executable(mujoco_dev main.cpp)
target_link_libraries(mujoco_dev PRIVATE robonode::sim_mujoco robonode::recorder robonode::warnings)
target_compile_definitions(mujoco_dev PRIVATE ROBONODE_WORLDS="${ROBONODE_WORLDS}")
