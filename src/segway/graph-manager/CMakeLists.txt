# EXTERNAL DEPS
perlsx_eigen3 ()
perlsx_isam ()
perlsx_suitesparse ()

# BUILD EXE
perls_add_executable (graph-manager main.cpp seg_graph_manager.h)
perls_target_link_libraries (graph-manager perls-lcmtypes perls-common perls-math
  ${PERLSX_EIGEN3}
  ${PERLSX_ISAM}
  ${PERLSX_SUITESPARSE}
  )
