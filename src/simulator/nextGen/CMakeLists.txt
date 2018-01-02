#perls_add_executable (nextGen-sim main)
#perls_target_link_libraries (nextGen-sim perls-lcmtypes++ perls-common plankton)

perls_add_executable (nextGen-sim-small main_small.cpp)
perls_target_link_libraries (nextGen-sim-small perls-lcmtypes++ perls-common acfr-common)

#perls_add_executable (nextGen-sim main.cpp)
#perls_target_link_libraries (nextGen-sim perls-lcmtypes++ perls-common)


include_directories ( 
	/usr/local/include/libplankton
	)				
	
link_directories (/usr/local/lib)


