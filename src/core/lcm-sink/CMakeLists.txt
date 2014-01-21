# EXTERNAL DEPS
perlsx_glib ()

# BUILD EXE
perls_add_executable (lcm-sink main.c)
perls_target_link_libraries (lcm-sink perls-common perls-lcmtypes
  ${PERLSX_GLIB}
  )
