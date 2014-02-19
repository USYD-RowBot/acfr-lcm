# EXTERNAL DEPS
perlsx_glib ()

# BUILD EXE
perls_add_executable (cam-compress main.c)
perls_target_link_libraries (cam-compress perls-lcmtypes perls-common perls-vision
  ${PERLSX_GLIB} turbojpeg
  )

