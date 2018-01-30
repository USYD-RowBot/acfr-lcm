perls_add_executable (auv-control main.cpp pid.cpp timer.cpp)
perls_target_link_libraries (auv-control perls-lcmtypes perls-common pthread)

perls_add_executable (auv-control-nga main_nga.cpp pid.cpp timer.cpp)
perls_target_link_libraries (auv-control-nga perls-lcmtypes acfr-common pthread)

# new style controllers using abstract base class
perls_add_executable(iver-control iver.cpp controller.cpp pid.cpp)
perls_target_link_libraries(iver-control perls-lcmtypes acfr-common pthread)

perls_add_executable(nga-control nga.cpp controller.cpp pid.cpp)
perls_target_link_libraries(nga-control perls-lcmtypes acfr-common pthread)
