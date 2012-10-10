# EXTERNAL DEPS
perlsx_glib ()

# BUILD EXE
perls_add_executable (logbook main.c)
perls_target_link_libraries (logbook perls-lcmtypes perls-common
  ${PERLSX_GLIB}
  )
