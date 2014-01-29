# EXTERNAL DEPS
perlsx_glib ()

# BUILD EXE
perls_add_executable (state main.c iver_safety.c iver_safety.h 
    iver_nav.c iver_nav.h)
perls_target_link_libraries (state perls-lcmtypes perls-common perls-math
  ${PERLSX_GLIB}
  )

