perls_add_executable (local-planner-iver local_planner.cpp local_planner_iver.cpp DubinsPath.cpp main_iver.cpp)
perls_target_link_libraries (local-planner-iver perls-lcmtypes++ acfr-common)

#perls_add_executable (local-planner2D local_planner2D.cpp DubinsPath2D.cpp main2D.cpp)
#perls_target_link_libraries (local-planner2D perls-lcmtypes++ acfr-common)

perls_add_executable (local-planner-nga local_planner_nga.cpp local_planner.cpp DubinsPath.cpp main_nga.cpp)
perls_target_link_libraries (local-planner-nga perls-lcmtypes++ acfr-common)

perls_add_executable (local-planner-sirius local_planner.cpp local_planner_sirius.cpp DubinsPath.cpp main_sirius.cpp)
perls_target_link_libraries (local-planner-sirius perls-lcmtypes++ acfr-common)

link_directories (/usr/local/lib)
