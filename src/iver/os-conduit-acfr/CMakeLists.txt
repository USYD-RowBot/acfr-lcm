#//perls_add_executable (os-conduit-acfr main.c)
#perls_target_link_libraries (os-conduit-acfr perls-common perls-math perls-lcmtypes ${LIB_PTHREAD})

perls_add_executable (os-conduit-acfr os-conduit-acfr.c)
perls_target_link_libraries (os-conduit-acfr acfr-common perls-lcmtypes)
