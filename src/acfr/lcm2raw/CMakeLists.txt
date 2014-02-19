perls_add_executable (lcm2raw main.cpp ../acfr-nav-new/handlers.cpp)
perls_target_link_libraries (lcm2raw perls-lcmtypes perls-common adt seabedcommon seabedslam)
include_directories ( 
	/usr/local/include/libseabedcommon
	/usr/local/include/libadt
	/usr/local/include/libplankton
	/usr/local/include/libflounder
	/usr/local/include/libsnapper
	)				
link_directories (/usr/local/lib)
