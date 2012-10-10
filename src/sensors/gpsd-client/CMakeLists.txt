# EXTERNAL DEPS
perlsx_glib ()
perlsx_m ()
perlsx_gps ()

# BUILD EXE
perls_add_executable (gpsd3-client main3.c)
perls_target_link_libraries (gpsd3-client perls-lcmtypes perls-common
  ${PERLSX_GLIB}
  ${PERLSX_M}
  ${PERLSX_GPS}
  )
