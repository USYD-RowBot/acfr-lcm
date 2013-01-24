perls_add_executable (acfr-nav-new acfr_nav.cpp handlers.cpp magfield.c)
perls_target_link_libraries (acfr-nav-new perls-lcmtypes++ perls-common seabedcommon seabedslam adt plankton)

include_directories ( 
	/usr/local/include/libseabedcommon
	/usr/local/include/libadt
	/usr/local/include/libplankton
	/usr/local/include/libflounder
	/usr/local/include/libsnapper
	)				
link_directories (/usr/local/lib)


