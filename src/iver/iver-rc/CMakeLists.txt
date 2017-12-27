# EXTERNAL DEPS
perlsx_pthread ()

# BUILD EXE
perls_add_executable (iver-rc main.cpp)
perls_target_link_libraries (iver-rc perls-lcmtypes perls-common perls-math ${PERLSX_PTHREAD})

