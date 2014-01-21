# EXTERNAL DEPS
perlsx_m ()

# BUILD EXE
perls_add_executable (ardrone-navigator main.cpp navigator.cpp)
perls_target_link_libraries (ardrone-navigator perls-lcmtypes perls-common perls-est perls-math
  ${PERLSX_M}
  )
