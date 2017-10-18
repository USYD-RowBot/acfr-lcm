
# BUILD EXE
perls_add_executable (torqeedo main.c)
perls_target_link_libraries (torqeedo perls-lcmtypes acfr-common) 

