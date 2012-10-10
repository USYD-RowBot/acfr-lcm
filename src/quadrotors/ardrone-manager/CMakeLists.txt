# EXTERNAL DEPS
perlsx_m ()

# BUILD EXE
perls_add_executable (ardrone-mission-manager main.c)
perls_target_link_libraries (ardrone-mission-manager perls-lcmtypes perls-common
  ${PERLSX_M}
  )
