# EXTERNAL DEPS
perlsx_opencv ()

# BUILD EXE
perls_add_executable (clahs main.c)
perls_target_link_libraries(clahs perls-common perls-vision
  ${PERLSX_OPENCV}
  )
