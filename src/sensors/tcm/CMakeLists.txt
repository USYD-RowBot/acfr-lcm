perls_add_executable (tcm main.c tcm.h)
perls_target_link_libraries (tcm perls-lcmtypes perls-common acfr-common)
