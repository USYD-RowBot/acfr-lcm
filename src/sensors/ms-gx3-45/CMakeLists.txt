# EXTERNAL DEPS
perlsx_m ()

# BUILD EXE
perls_add_executable (ms-gx3-45 main.c ms_3dm_gx3_45.c ms_3dm_gx3_45.h)
perls_target_link_libraries (ms-gx3-45 perls-lcmtypes perls-common
  ${PERLSX_M}
  )
