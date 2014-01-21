# EXTERNAL DEPS
perlsx_glib ()

# BUILD EXE
perls_add_executable (camlog-logger main.c)
perls_target_link_libraries (camlog-logger perls-lcmtypes perls-common perls-vision
  ${PERLSX_GLIB}
  )
