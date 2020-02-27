
# BUILD EXE
perls_add_executable (vesc-controller main.c packet.c crc.c buffer.c bldc_interface.c bldc_interface_uart.c )
perls_target_link_libraries (vesc-controller perls-lcmtypes perls-common acfr-common)
