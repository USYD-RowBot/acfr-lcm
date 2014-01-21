perlsx_opencv ()

perls_add_executable (common-cache cache_demo.c)
perls_target_link_libraries(common-cache perls-lcmtypes perls-common ${PERLSX_OPENCV})

perls_add_executable (common-config config_demo.c)
perls_target_link_libraries(common-config perls-common)

perls_add_executable (common-getopt getopt_demo.c)
perls_target_link_libraries (common-getopt perls-common)

perls_add_executable (common-gsd gsd_demo.c)
perls_target_link_libraries (common-gsd perls-common)

perls_add_executable (common-timer timer_demo.c)
perls_target_link_libraries (common-timer perls-common)
