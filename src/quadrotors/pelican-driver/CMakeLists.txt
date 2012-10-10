# EXTERNAL DEPS
perlsx_m ()

# BUILD EXE
perls_add_executable (pelican-driver main.c pelican.c pelican.h)
perls_target_link_libraries (pelican-driver perls-lcmtypes perls-common
  ${PERLSX_M}
  )
