perls_add_executable (local-planner local_planner.cpp)
perls_target_link_libraries (local-planner perls-lcmtypes++ perls-common plankton)

include_directories ( 
	/usr/local/include/libplankton
	)				

link_directories (/usr/local/lib)
