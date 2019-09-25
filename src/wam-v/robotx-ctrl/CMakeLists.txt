perls_add_executable (robotx-ctrl main.cpp pid.cpp)
perls_target_link_libraries (robotx-ctrl perls-lcmtypes perls-common pthread)

