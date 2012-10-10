# EXTERNAL DEPS
perlsx_glib ()
perlsx_gsl ()
perlsx_m ()

# BUILD EXE
perls_add_executable (velodyne-refcal main.c)
perls_target_link_libraries (velodyne-refcal perls-lcmtypes perls-common perls-math perls-sensors
  ${PERLSX_GLIB}
  ${PERLSX_GSL}
  ${PERLSX_M}
  )
