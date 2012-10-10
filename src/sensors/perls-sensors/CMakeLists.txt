# SOURCE FILES
set (SRC
  applanix.c               applanix.h
  rdi.c                    rdi.h
  velodyne.c               velodyne.h
  )

# EXTERNAL DEPS
perlsx_bot2 ()
perlsx_glib ()
perlsx_gsl ()
perlsx_m ()
perlsx_xml2 ()

# BUILD LIBRARY
perls_add_library (perls-sensors ${SRC})
perls_target_link_libraries (perls-sensors perls-lcmtypes perls-common perls-math
  ${PERLSX_BOT2}
  ${PERLSX_GLIB}
  ${PERLSX_GSL}
  ${PERLSX_M}
  ${PERLSX_XML2}
  )
