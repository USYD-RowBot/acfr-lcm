# EXTERNAL DEPS
perlsx_m ()

# BUILD EXE
perls_add_executable (ms-gx1 main.c ms_3dm_gx1.c ms_3dm_gx1.h)
perls_target_link_libraries (ms-gx1 perls-lcmtypes perls-common
  ${PERLSX_M}
  )
