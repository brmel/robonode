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
target_link_libraries(robonode_cli PRIVATE robonode::gateway robonode::warnings CLI11::CLI11)
target_compile_definitions(robonode_cli PRIVATE
    ROBONODE_WORLDS="${ROBONODE_WORLDS}"
    ROBONODE_CELL="${CMAKE_SOURCE_DIR}/apps/cell_server/cells/ur10e.cell.json")

# Agent-parity smoke: `robonode nodes` boots the cell and prints the tree.
add_test(NAME robonode_cli_nodes COMMAND robonode_cli nodes)
set_tests_properties(robonode_cli_nodes PROPERTIES PASS_REGULAR_EXPRESSION "rail-x")

# App runner (#56): `robonode app <file>` runs a program to completion (exit 0).
add_test(NAME robonode_cli_app
  COMMAND robonode_cli app "${CMAKE_SOURCE_DIR}/apps/cell_server/apps/pick-demo.app.json")
