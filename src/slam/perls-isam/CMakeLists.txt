set(CMAKE_BUILD_TYPE Release)

# SOURCE FILES
set (SRC
  gparser.cpp           gparser.h
  isam_util.cpp         isam_util.h
  user_factors.h
  user_nodes.h
  glc_factors.h
)

# EXTERNAL DEPS
perlsx_eigen3 ()
perlsx_glib ()
perlsx_isam ()
perlsx_suitesparse ()

# BUILD EXE
perls_add_library (perls-isam STATIC ${SRC})
perls_target_link_libraries (perls-isam perls-lcmtypes perls-common perls-math
  ${PERLSX_EIGEN3}
  ${PERLSX_GLIB}
  ${PERLSX_ISAM}
  ${PERLSX_SUITESPARSE}
  )

