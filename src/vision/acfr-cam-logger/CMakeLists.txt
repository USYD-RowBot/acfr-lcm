# EXTERNAL DEPS
perlsx_glib ()

# BUILD EXE
perls_add_executable (acfr-cam-logger main.c)
perls_target_link_libraries (acfr-cam-logger perls-lcmtypes perls-common perls-vision
  ${PERLSX_GLIB}
  )


#perls_add_executable (acfr-cam-logger main.c)
#perls_target_link_libraries (acfr-cam-logger perls-lcmtypes perls-common  lcmtypes_bot2-core ${LIB_GLIB})
