perls_add_executable (asv-simple main_small.cpp)
perls_target_link_libraries (asv-simple perls-lcmtypes++ perls-common plankton)

include_directories ( 
	/usr/local/include/libplankton
	)				
	
link_directories (/usr/local/lib)


