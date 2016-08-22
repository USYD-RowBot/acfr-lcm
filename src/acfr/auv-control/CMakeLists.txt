perls_add_executable (auv-control main.c pid.c timer.c)
perls_target_link_libraries (auv-control perls-lcmtypes perls-common)

perls_add_executable (auv-control-nextGen main_nextGen.c pid.c timer.c)
perls_target_link_libraries (auv-control-nextGen perls-lcmtypes perls-common)
