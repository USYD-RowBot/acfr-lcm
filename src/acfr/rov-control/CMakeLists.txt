perls_add_executable (rov-control rov-control.cpp ../auv-control/pid.cpp ../auv-control/timer.cpp)
perls_target_link_libraries (rov-control perls-lcmtypes acfr-common pthread)
