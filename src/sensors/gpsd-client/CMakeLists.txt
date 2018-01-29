perls_add_executable (gpsd3-client main3.c)
perls_target_link_libraries (gpsd3-client perls-lcmtypes acfr-common gps)

perls_add_executable (gpsd3-client-trusty main3_trusty.c)
perls_target_link_libraries (gpsd3-client-trusty perls-lcmtypes acfr-common gps)
