set(CMAKE_BUILD_TYPE Release)

# SOURCE FILES
set (SRC
  main.cpp
  fileio_thread.cpp     fileio_thread.h
  interf_thread.cpp     interf_thread.h
  isam_server.cpp       isam_server.h
)

# EXTERNAL DEPS
perlsx_eigen3 ()
perlsx_glib ()
perlsx_isam ()
perlsx_suitesparse ()

# BUILD EXE
perls_add_executable (isam-server ${SRC})
perls_target_link_libraries (isam-server perls-lcmtypes perls-common perls-math perls-isam
  ${PERLSX_EIGEN3}
  ${PERLSX_GLIB}
  ${PERLSX_ISAM}
  ${PERLSX_SUITESPARSE}
  )

