perls_add_executable (rov-control rov-control.c ../auv-control/pid.c ../auv-control/timer.c)
perls_target_link_libraries (rov-control perls-lcmtypes acfr-common)
