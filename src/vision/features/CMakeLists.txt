# EXTERNAL DEPS
perlsx_glib ()
perlsx_gsl ()
perlsx_opencv ()

# BUILD EXE
perls_add_executable (features main.c)
perls_target_link_libraries (features perls-lcmtypes perls-common perls-vision
  ${PERLSX_GLIB}
  ${PERLSX_GSL}
  ${PERLSX_OPENCV}
  )
