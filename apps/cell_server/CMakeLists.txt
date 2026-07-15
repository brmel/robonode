add_executable(cell_server main.cpp)
target_link_libraries(cell_server PRIVATE robonode::gateway robonode::warnings)
target_compile_definitions(cell_server PRIVATE
    ROBONODE_WORLDS="${ROBONODE_WORLDS}"
    ROBONODE_WEB="${CMAKE_CURRENT_SOURCE_DIR}/web"
    ROBONODE_CELL="${CMAKE_CURRENT_SOURCE_DIR}/cells/ur10e.cell.json"
    ROBONODE_APPS="${CMAKE_CURRENT_SOURCE_DIR}/apps")
