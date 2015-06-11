perls_add_executable (seabotix-interface seabotix_interface.c)
perls_target_link_libraries (seabotix-interface perls-lcmtypes acfr-common)

perls_add_executable (seabotix_ahrs seabotix_ahrs.c)
perls_target_link_libraries (seabotix_ahrs perls-lcmtypes acfr-common pcap)


