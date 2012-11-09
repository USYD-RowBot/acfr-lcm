# BUILD EXE
perls_add_executable (leak main.c)
perls_target_link_libraries (leak perls-lcmtypes perls-common perls-math ${PERLSX_PTHREAD})

