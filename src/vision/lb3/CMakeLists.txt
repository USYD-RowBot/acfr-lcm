# EXTERNAL DEPS
perlsx_pthread ()
perlsx_dc1394 ()
perlsx_jpeg ()
perlsx_m ()

# BUILD EXE
perls_add_executable (lb3 main.c ladybug3.c ladybug3.h)
perls_target_link_libraries (lb3 perls-lcmtypes perls-common perls-vision
  ${PERLSX_PTHREAD}
  ${PERLSX_DC1394}
  ${PERLSX_JPEG}
  ${PERLSX_MATH}
  )

perls_add_executable (lb3-convert lb3_convert.c)
perls_target_link_libraries (lb3-convert perls-common perls-vision
  ${PERLSX_JPEG}
  )
