# EXTERNAL DEPS
perlsx_dc1394 ()

# BUILD EXE
perls_add_executable (dc1394-camera main.c perls_dc1394_camera.c perls_dc1394_camera.h)
perls_target_link_libraries (dc1394-camera perls-lcmtypes perls-common perls-math
  ${PERLSX_DC1394}
  )
