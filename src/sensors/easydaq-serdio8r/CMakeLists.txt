perls_add_executable (easydaq-serdio8r main.c easydaq.c easydaq.h)
perls_target_link_libraries (easydaq-serdio8r perls-lcmtypes perls-common)

perls_add_executable (easydaq-tool relaytool.c easydaq.c easydaq.h)
perls_target_link_libraries (easydaq-tool perls-lcmtypes perls-common)
