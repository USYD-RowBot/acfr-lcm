perlsx_glib ()

perls_add_executable (gpsd-ahrs-munge main.cpp)
perls_target_link_libraries (gpsd-ahrs-munge acfr-common perls-common)
