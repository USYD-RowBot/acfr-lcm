perls_add_executable (local-planner local_planner.cpp DubinsPath.cpp)
perls_target_link_libraries (local-planner perls-lcmtypes++ perls-common)

link_directories (/usr/local/lib)
