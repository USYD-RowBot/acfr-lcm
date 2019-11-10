set (SRC 
    sensor.c    sensor.h
    serial.c    serial.h
                error.h
	timestamp.c	timestamp.h
	nmea.c		nmea.h
	socket.c	socket.h
				units.h
	dfs.c		dfs.h
                spektrum-control.h
#                	auv_map_projection.cpp  auv_map_projection.hpp
)

include_directories ( 
#	/usr/include/uf-3.2
	/usr/include/glib-2.0
	)	

perls_add_library (acfr-common ${SRC})
#perls_target_link_libraries ( acfr-common bot2-param-client ufGeographicConversions)
perls_target_link_libraries ( acfr-common bot2-param-client)

