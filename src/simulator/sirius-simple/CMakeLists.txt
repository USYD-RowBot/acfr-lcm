perls_add_executable (sirius-simple main_simple.cpp)
perls_target_link_libraries (sirius-simple perls-lcmtypes++ perls-common acfr-common simulator-common)

include_directories ( 
	/usr/include/uf-3.2
	)				
	
link_directories (/usr/local/lib)


