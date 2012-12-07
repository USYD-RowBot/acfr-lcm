perls_add_executable (iver-sim main)
perls_target_link_libraries (iver-sim perls-lcmtypes++ perls-common plankton)

include_directories ( 
	/usr/local/include/libplankton
	)				
	
link_directories (/usr/local/lib)


