set (SRC 
    sensor.c    sensor.h
    serial.c    serial.h
                error.h
)

perls_add_library (acfr-common ${SRC})
#perls_target_link_libraries ()

