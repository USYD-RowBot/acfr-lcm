perls_add_executable(iver-control iver.cpp controller.cpp pid.cpp)
perls_target_link_libraries(iver-control perls-lcmtypes acfr-common pthread)

perls_add_executable(nga-control nga.cpp controller.cpp pid.cpp)
perls_target_link_libraries(nga-control perls-lcmtypes acfr-common pthread)

perls_add_executable(sirius-control sirius.cpp controller.cpp pid.cpp)
perls_target_link_libraries(sirius-control perls-lcmtypes acfr-common pthread)
