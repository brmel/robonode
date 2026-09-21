add_executable(ur_governed_move main.cpp)
target_link_libraries(ur_governed_move PRIVATE robonode::adapter_ur robonode::recorder robonode::warnings)
target_compile_definitions(ur_governed_move PRIVATE
    URCL_RESOURCES="${ur_client_library_SOURCE_DIR}")
