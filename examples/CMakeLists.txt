# Examples — runnable demonstrations, not product surface. Each shows one seam
# doing its job; none is linked into cell_server or the CLI.
if(ROBONODE_BUILD_RTB)
  add_subdirectory(rtb_dev)
endif()
if(ROBONODE_BUILD_UR_ADAPTER)
  add_subdirectory(ur_governed_move)
endif()
