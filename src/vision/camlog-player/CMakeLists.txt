# EXTERNAL DEPS
perlsx_glib ()

# BUILD EXE
perls_add_executable (camlog-player main.c)
perls_target_link_libraries (camlog-player perls-lcmtypes perls-common perls-vision
  ${PERLSX_GLIB}
  )
