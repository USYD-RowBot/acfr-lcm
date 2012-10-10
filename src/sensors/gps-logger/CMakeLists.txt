# EXTERNAL DEPS
perlsx_boost ()

perls_add_executable (gpslog-to-lcm main.cpp)
perls_target_link_libraries (gpslog-to-lcm perls-common perls-lcmtypes ${PERLSX_BOOST})
