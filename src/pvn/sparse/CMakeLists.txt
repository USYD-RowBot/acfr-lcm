# EXTERNAL DEPS
perlsx_gsl ()
perlsx_eigen3 ()
perlsx_isam ()
perlsx_suitesparse ()

# hack till jeff addes eigen matrix slice to lib
include_directories (".")

# BUILD EXE
perls_add_executable (sparse main.cpp sparse.h
                      baseline_marginalization.cpp
                      compound_marginalization.cpp
                      lifting_marginalization.cpp
                      lifting_marginalization_glc.cpp
                      gen_simple_graph.cpp
                                      eigen_utils.h
                                      matrix_slice.h
                                      matrixbase_addons.h
                      chowliutree.cpp chowliutree.h)

perls_target_link_libraries (sparse perls-lcmtypes perls-common perls-math perls-pvn perls-isam
  ${PERLSX_GSL}
  ${PERLSX_EIGEN3}
  ${PERLSX_ISAM}
  ${PERLSX_SUITESPARSE}
  )