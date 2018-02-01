# BUILD EXE
perls_add_executable (spektrum-control spektrum-control.c)
perls_target_link_libraries (spektrum-control perls-lcmtypes acfr-common udev)

