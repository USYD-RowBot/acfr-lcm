perls_add_executable (cam-trigger main.c)
perls_target_link_libraries (cam-trigger perls-lcmtypes perls-common ${LIB_GLIB} ${LIB_BOT2})

perls_add_executable (cam-trigger-diver main-diver.c)
perls_target_link_libraries (cam-trigger-diver perls-lcmtypes perls-common ${LIB_GLIB})

perls_add_executable (cam-test cam-test.c)
perls_target_link_libraries (cam-test perls-lcmtypes readline ncurses ${LIB_GLIB} ${LIB_LCM})

perls_add_executable (cam-trigger-tcp main-tcp.c)
perls_target_link_libraries (cam-trigger-tcp perls-lcmtypes perls-common ${LIB_BOT2})
