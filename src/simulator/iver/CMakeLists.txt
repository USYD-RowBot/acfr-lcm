#perls_add_executable (iver-sim main)
#perls_target_link_libraries (iver-sim perls-lcmtypes++ perls-common plankton)

perls_add_executable (iver-sim-small main_small.cpp)
perls_target_link_libraries (iver-sim-small perls-lcmtypes++ perls-common acfr-common)

#perls_add_executable (iver-sim main.cpp)
#perls_target_link_libraries (iver-sim perls-lcmtypes++ perls-common)


link_directories (/usr/local/lib)


