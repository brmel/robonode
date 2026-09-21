# robonode::app_support — what a composition root needs and a library must not
# have: the choice of which vendor drivers this build registers. Kept out of
# robonode::gateway so the gateway stays vendor-free (boundary lint).
add_library(robonode_app_support INTERFACE)
add_library(robonode::app_support ALIAS robonode_app_support)
target_include_directories(robonode_app_support INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robonode_app_support INTERFACE robonode::gateway)
if(ROBONODE_BUILD_OPENCV)
  target_link_libraries(robonode_app_support INTERFACE robonode::vision_cv)
  target_compile_definitions(robonode_app_support INTERFACE ROBONODE_WITH_OPENCV=1)
endif()
if(ROBONODE_BUILD_UR_ADAPTER)
  target_link_libraries(robonode_app_support INTERFACE robonode::adapter_ur)
  target_compile_definitions(robonode_app_support INTERFACE
      ROBONODE_WITH_UR=1
      ROBONODE_UR_RESOURCES="${ur_client_library_SOURCE_DIR}")
endif()
