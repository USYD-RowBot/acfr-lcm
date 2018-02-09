#perls_add_executable (novatel main.c time_conversion.c)
#perls_target_link_libraries (novatel perls-lcmtypes perls-common)

perls_add_executable (novatel main_binary.c novatel.c)
perls_target_link_libraries (novatel perls-lcmtypes acfr-common bot2-param-client m)

