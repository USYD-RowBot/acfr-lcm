# EXTERNAL DEPS
perlsx_openjaus ()

# BUILD EXE
perls_add_executable (node-manager main.cpp)
perls_target_link_libraries (node-manager perls-lcmtypes perls-common pthread
  ${PERLSX_JAUS}
  ${PERLSX_OPENJAUS}
)
