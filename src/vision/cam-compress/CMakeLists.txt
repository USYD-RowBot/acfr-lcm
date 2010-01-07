# EXTERNAL DEPS
perlsx_glib ()

# BUILD EXE
perls_add_executable (cam-compress main.c bayer.c)
perls_target_link_libraries (cam-compress perls-lcmtypes perls-common 
  ${PERLSX_GLIB} jpeg
  )

