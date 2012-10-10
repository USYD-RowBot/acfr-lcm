# SOURCE FILES
set (SRC
  main.c
  prosilica.c       prosilica.h
  )

# EXTERNAL DEPS
perlsx_pvapi ()
perlsx_glib ()
perlsx_tiff ()
perlsx_pthread ()

# BUILD EXE
perls_add_executable (acfr-prosilica ${SRC})
perls_target_link_libraries (acfr-prosilica perls-lcmtypes perls-common perls-vision
  ${PERLSX_PTHREAD}
  ${PERLSX_GLIB}
  ${PERLSX_TIFF}
  ${PERLSX_PVAPI}
  )
