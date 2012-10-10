# EXTERNAL DEPS
perlsx_glib ()

# BUILD EXE
perls_add_executable (heartbeat main.c)
perls_target_link_libraries (heartbeat perls-lcmtypes perls-common
  ${PERLSX_GLIB}
  )
