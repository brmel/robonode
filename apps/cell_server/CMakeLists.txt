add_executable(cell_server main.cpp)
target_link_libraries(cell_server PRIVATE robonode::app_support robonode::gateway robonode::warnings)
target_compile_definitions(cell_server PRIVATE
    ROBONODE_WORLDS="${ROBONODE_WORLDS}"
    ROBONODE_WEB="${CMAKE_CURRENT_SOURCE_DIR}/web"
    ROBONODE_CELL="${CMAKE_CURRENT_SOURCE_DIR}/cells/ur10e.cell.json"
    ROBONODE_ROBOTS="${CMAKE_CURRENT_SOURCE_DIR}/cells"
    ROBONODE_APPS="${CMAKE_CURRENT_SOURCE_DIR}/apps"
    ROBONODE_MODULES="${CMAKE_CURRENT_SOURCE_DIR}/modules"
    ROBONODE_SCENES="${CMAKE_SOURCE_DIR}/apps/cell_server/scenes"
    ROBONODE_SESSIONS="${CMAKE_SOURCE_DIR}/apps/cell_server/sessions"
    ROBONODE_SETTINGS="${CMAKE_SOURCE_DIR}/config/robonode.settings.json")

