perls_add_executable (vLBV_sensors vLBV_sensors.c)
perls_target_link_libraries (vLBV_sensors perls-lcmtypes acfr-common)

perls_add_executable (seabotix_ahrs seabotix_ahrs.c)
perls_target_link_libraries (seabotix_ahrs perls-lcmtypes acfr-common pcap)
