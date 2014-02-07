# EXTERNAL DEPS
perlsx_pthread ()

# BUILD EXE
perls_add_executable (rc main.c)
perls_target_link_libraries (rc perls-lcmtypes perls-common perls-math ${PERLSX_PTHREAD})

