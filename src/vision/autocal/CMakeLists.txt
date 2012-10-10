# EXTERNAL DEPS
perlsx_m ()
perlsx_opencv ()

# BUILD EXE
perls_add_executable (autocal main.cpp autoCalHelpers.c autoCalHelpers.h)
perls_target_link_libraries (autocal perls-common
  ${PERLSX_M}
  ${PERLSX_OPENCV}
  )
