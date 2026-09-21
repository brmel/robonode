# robonode CLI — headless cell control over the Platform facade (#43). CLI11 is
# header-only (arg/subcommand parsing); the CLI binds the same robonode::gateway
# (Platform) the web server does, so UI and CLI never diverge (ADR-8).
include(FetchContent)
FetchContent_Declare(CLI11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v2.4.2
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(CLI11)

add_executable(robonode_cli main.cpp)
target_link_libraries(robonode_cli PRIVATE robonode::app_support robonode::gateway robonode::warnings CLI11::CLI11)
target_include_directories(robonode_cli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(robonode_cli PRIVATE
    ROBONODE_WORLDS="${ROBONODE_WORLDS}"
    ROBONODE_CELL="${CMAKE_SOURCE_DIR}/apps/cell_server/cells/ur10e.cell.json"
    ROBONODE_APPS="${CMAKE_SOURCE_DIR}/apps/cell_server/apps"
    ROBONODE_MODULES="${CMAKE_SOURCE_DIR}/apps/cell_server/modules"
    ROBONODE_SCENES="${CMAKE_SOURCE_DIR}/apps/cell_server/scenes"
    ROBONODE_SESSIONS="${CMAKE_SOURCE_DIR}/apps/cell_server/sessions"
    ROBONODE_SETTINGS="${CMAKE_SOURCE_DIR}/config/robonode.settings.json"
    ROBONODE_ROBOTS="${CMAKE_SOURCE_DIR}/apps/cell_server/cells")

# Agent-parity smoke: `robonode nodes` boots the cell and prints the tree.
add_test(NAME robonode_cli_nodes COMMAND robonode_cli nodes)
set_tests_properties(robonode_cli_nodes PROPERTIES PASS_REGULAR_EXPRESSION "rail-x")

# App runner (#56): `robonode app <file>` runs a program to completion (exit 0).
add_test(NAME robonode_cli_app
  COMMAND robonode_cli app "${CMAKE_SOURCE_DIR}/apps/cell_server/apps/pick-demo.app.json")

# Logs surface (#42/#44): `robonode logs` prints the log ring (cell-built at least).
add_test(NAME robonode_cli_logs COMMAND robonode_cli logs)
set_tests_properties(robonode_cli_logs PROPERTIES PASS_REGULAR_EXPRESSION "cell built")

# --server drives a RUNNING cell instead of booting one. With nothing listening
# it must fail loudly rather than quietly answering about a private cell.
add_test(NAME robonode_cli_server_unreachable
  COMMAND robonode_cli --server http://127.0.0.1:9 telemetry)
set_tests_properties(robonode_cli_server_unreachable PROPERTIES WILL_FAIL TRUE)

# ADR-8 capability parity: swapping an algorithm version works headless (#43).
add_test(NAME robonode_cli_version COMMAND robonode_cli version planner robonode.moveJ)
set_tests_properties(robonode_cli_version PROPERTIES PASS_REGULAR_EXPRESSION "robonode.moveJ")

# Authoring a sandboxed version works headless and registers it.
add_test(NAME robonode_cli_define COMMAND robonode_cli define vision cli-test "x; y; z + 0.03")
set_tests_properties(robonode_cli_define PROPERTIES PASS_REGULAR_EXPRESSION "user.cli-test")

