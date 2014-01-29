perls_add_executable (acfr-cam-server main.c)
perls_target_link_libraries (acfr-cam-server perls-lcmtypes perls-common perls-vision lcmtypes_bot2-core)
