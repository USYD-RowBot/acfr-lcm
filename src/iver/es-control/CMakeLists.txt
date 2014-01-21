# EXTERNAL DEPS
# perlsx_glib ()
# perlsx_pthread ()

# BUILD EXE
perls_add_executable (es-control main.c)
perls_target_link_libraries (es-control perls-lcmtypes perls-common perls-math)
#  ${PERLSX_GLIB}
#  ${PERLSX_PTHREAD}
#  )

