perls_add_executable (micron-sounder micron-sounder.c)
perls_target_link_libraries (micron-sounder perls-lcmtypes acfr-common)
perls_add_executable (micron-sounder-multiplex micron-sounder-multiplex.c)
perls_target_link_libraries (micron-sounder-multiplex perls-lcmtypes acfr-common)
