perls_add_executable (asv-simple main_small.cpp)
perls_target_link_libraries (asv-simple perls-lcmtypes++ perls-common acfr-common)

include_directories ( 
	/usr/include/uf-3.2
	)				
	
link_directories (/usr/local/lib)


