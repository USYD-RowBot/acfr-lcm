# EXTERNAL DEPS
perlsx_pthread ()

# BUILD EXE
perls_add_executable (iver-motors main.c)
perls_target_link_libraries (iver-motors perls-lcmtypes acfr-common )

