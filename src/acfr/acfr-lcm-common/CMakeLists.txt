set (SRC 
    sensor.c    sensor.h
    serial.c    serial.h
                error.h
	timestamp.c	timestamp.h
)


perls_add_library (acfr-common ${SRC})
perls_target_link_libraries ( acfr-common bot2-param-client)

