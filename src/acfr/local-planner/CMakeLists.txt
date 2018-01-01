perls_add_executable (local-planner local_planner.cpp DubinsPath.cpp main.cpp)
perls_target_link_libraries (local-planner perls-lcmtypes++ perls-common)

perls_add_executable (local-planner2D local_planner2D.cpp DubinsPath2D.cpp main2D.cpp)
perls_target_link_libraries (local-planner2D perls-lcmtypes++ perls-common)

link_directories (/usr/local/lib)
