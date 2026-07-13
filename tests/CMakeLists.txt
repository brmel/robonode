# Per-module test binaries — a test binary's include set doubles as a
# living statement of its module's dependencies.
add_executable(motion_tests motion_tests.cpp)
target_link_libraries(motion_tests PRIVATE robonode::motion)
add_test(NAME motion_tests COMMAND motion_tests)

add_executable(recorder_tests recorder_tests.cpp)
target_link_libraries(recorder_tests PRIVATE robonode::recorder)
add_test(NAME recorder_tests COMMAND recorder_tests)

add_executable(celld_tests celld_tests.cpp)
target_link_libraries(celld_tests PRIVATE robonode::celld)
target_compile_definitions(celld_tests PRIVATE
    ROBONODE_IDL_EXAMPLES="${CMAKE_SOURCE_DIR}/robonode-idl/examples")
add_test(NAME celld_tests COMMAND celld_tests)
