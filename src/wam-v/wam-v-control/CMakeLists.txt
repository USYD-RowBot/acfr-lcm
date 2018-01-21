perls_add_executable (wam-v-control main.cpp pid.cpp)
perls_target_link_libraries (wam-v-control perls-lcmtypes perls-common pthread)

