# EXTERNAL DEPS
perlsx_pthread ()
perlsx_glib ()

# BUILD EXE
perls_add_executable (camlog-convert main.c)
perls_target_link_libraries (camlog-convert perls-common perls-lcmtypes perls-vision
  ${PERLSX_PTHREAD}
  ${PERLSX_GLIB}
  )
