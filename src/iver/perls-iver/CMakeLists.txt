# SOURCE FILES
set (SRC 
  remotehelm.c             remotehelm.h
  vectormap.c              vectormap.h
  )

# EXTERNAL DEPS
perlsx_bot2 ()
perlsx_glib ()

# BUILD LIBRARY
perls_add_library (perls-iver ${SRC})
perls_target_link_libraries (perls-iver perls-lcmtypes perls-common
  ${PERLSX_BOT2}
  ${PERLSX_GLIB}
  )
