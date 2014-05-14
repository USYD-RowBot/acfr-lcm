# EXTERNAL DEPS
perlsx_glib ()

# BUILD EXE
perls_add_executable (cam-viewer main.cpp)
perls_target_link_libraries (cam-viewer perls-lcmtypes perls-common perls-vision
  ${PERLSX_GLIB} turbojpeg
  )

