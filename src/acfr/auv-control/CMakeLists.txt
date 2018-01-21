perls_add_executable (auv-control main.cpp pid.cpp timer.cpp)
perls_target_link_libraries (auv-control perls-lcmtypes perls-common pthread)

perls_add_executable (auv-control-nga main_nga.cpp pid.cpp timer.cpp)
perls_target_link_libraries (auv-control-nga perls-lcmtypes acfr-common pthread)
