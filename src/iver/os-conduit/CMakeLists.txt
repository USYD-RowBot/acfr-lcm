# EXTERNAL DEPS
perlsx_glib ()
perlsx_pthread ()

# BUILD EXE
perls_add_executable (os-conduit main.c)
perls_target_link_libraries (os-conduit perls-lcmtypes perls-common perls-math
  ${PERLSX_GLIB}
  ${PERLSX_PTHREAD}
  )

