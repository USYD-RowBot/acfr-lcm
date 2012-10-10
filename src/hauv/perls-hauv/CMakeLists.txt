# SOURCE FILES
set (SRC 
  lm.c                    lm.h
  lmlec.c 
  lmbc.c 
  lmblec.c
  Axb.c 
  misc.c                  misc.h
  compiler.h
  vehiclestate.cpp        vehiclestate.h
  vehicletrajectory.cpp   vehicletrajectory.h
  hauv_timing.cpp         hauv_timing.h
  twoview_reg.c           twoview_reg.h
)

# EXTERNAL DEPS
perlsx_lapack ()
perlsx_blas ()
perlsx_f2c ()
perlsx_eigen3 ()

# BUILD LIBRARY
perls_add_library (perls-hauv STATIC ${SRC})
perls_target_link_libraries (perls-hauv perls-common perls-lcmtypes perls-math perls-vision
  ${PERLSX_LAPACK}
  ${PERLSX_BLAS}
  ${PERLSX_F2C}
  ${PERLSX_EIGEN3}
  )
