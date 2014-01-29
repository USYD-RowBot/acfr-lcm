# libest test
perls_add_executable (est-demo test_libest.c)
perls_target_link_libraries (est-demo perls-lcmtypes perls-math perls-est)
