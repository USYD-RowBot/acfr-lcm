# EXTERNAL DEPS
perlsx_m ()

# BUILD EXE
perls_add_executable (ms-gx3-25 main.c ms_3dm_gx3.c ms_3dm_gx3.h)
perls_target_link_libraries (ms-gx3-25 perls-lcmtypes perls-common perls-math
  ${PERLSX_M}
  )
