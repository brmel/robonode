# robonode::motion — RT-domain motion module: governor, plans, executives,
# and the AxisAdapter seam. trajlib (trajectory-lab) is this module's
# implementation detail: the include path leaks (header-only library) but
# the boundary lint forbids anything outside motion/ from including
# trajlib/* directly, and no trajlib type appears in motion's public
# signatures.
add_library(robonode_motion INTERFACE)
add_library(robonode::motion ALIAS robonode_motion)
target_include_directories(robonode_motion INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory-lab/include
)
target_link_libraries(robonode_motion INTERFACE robonode::core)
