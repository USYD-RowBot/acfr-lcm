# EXTERNAL DEPS
perlsx_spnav ()
perlsx_x11 ()

# BUILD EXE    
perls_add_executable (spacenav main.c)
perls_target_link_libraries (spacenav perls-common perls-lcmtypes
  ${PERLSX_SPNAV}
  ${PERLSX_X11}
  )
