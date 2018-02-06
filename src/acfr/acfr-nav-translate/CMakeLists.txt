# BUILD EXE
perls_add_executable (acfr-nav-translate main.c)
perls_target_link_libraries (acfr-nav-translate perls-lcmtypes acfr-common m proj)

