INCLUDE_DIRECTORIES(/usr/include/libusb-1.0/ /usr/local/include/aiousb)

perls_add_executable (aio-relay aio-relay.c aiocommon.c)
perls_target_link_libraries (aio-relay perls-lcmtypes acfr-common aiousb)

