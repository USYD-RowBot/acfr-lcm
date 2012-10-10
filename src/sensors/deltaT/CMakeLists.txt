perls_add_executable (deltaT main.c)
perls_target_link_libraries (deltaT perls-lcmtypes perls-common)

perls_add_executable (deltaT-logger logger.c)
perls_target_link_libraries (deltaT-logger perls-lcmtypes perls-common)
