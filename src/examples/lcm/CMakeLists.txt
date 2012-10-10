perls_add_executable (lcm-listener listener.c)
perls_target_link_libraries (lcm-listener perls-lcmtypes)

perls_add_executable (lcm-sendmsg send_message.c)
perls_target_link_libraries (lcm-sendmsg perls-lcmtypes)

perls_add_executable (lcm-sendlarge send_large.c)
perls_target_link_libraries (lcm-sendlarge perls-lcmtypes)

perls_add_executable (lcm-listenlarge listen_large.c)
perls_target_link_libraries (lcm-listenlarge perls-lcmtypes)
