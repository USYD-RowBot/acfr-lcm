# EXTERNAL DEPS
perlsx_m ()
perlsx_gsl ()

# BUILD EXE
perls_add_executable (position main.c)
perls_target_link_libraries (position perls-lcmtypes perls-common perls-math
  ${PERLSX_M}
  ${PERLSX_GSL}
  )
