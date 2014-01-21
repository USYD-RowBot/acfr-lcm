# EXTERNAL DEPS
perlsx_gsl ()

# BUILD EXE
perls_add_executable (mocap main.c)
perls_target_link_libraries (mocap perls-lcmtypes perls-common
  ${PERLSX_GSL}
  )
