# EXTERNAL DEPS
perlsx_pthread ()

# BUILD EXE
perls_add_executable (iver-motors main.cpp)
perls_target_link_libraries (iver-motors perls-lcmtypes perls-common perls-math ${PERLSX_PTHREAD})

