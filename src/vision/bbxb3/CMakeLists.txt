# EXTERNAL DEPS
perlsx_dc1394 ()
perlsx_jpeg ()

# BUILD EXE
perls_add_executable (bbxb3 main.cpp bumblebeexb3.cpp bumblebeexb3.h)
perls_target_link_libraries (bbxb3 perls-lcmtypes perls-common
  ${PERLSX_DC1394}
  ${PERLSX_JPEG}
  )
