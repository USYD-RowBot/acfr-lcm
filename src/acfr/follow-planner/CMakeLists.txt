#perls_add_executable (follow-planner follow_planner.cpp DubinsPath.cpp main.cpp)
#perls_target_link_libraries (follow-planner perls-lcmtypes++ perls-common)

perls_add_executable (follow-planner2D follow_planner2D.cpp main2D.cpp)
perls_target_link_libraries (follow-planner2D perls-lcmtypes++ perls-common acfr-mission)

link_directories (/usr/local/lib)
