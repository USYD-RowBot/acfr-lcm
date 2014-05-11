perls_add_executable (evologics_usbl main_usbl.cpp evologics.c base91.c)
perls_target_link_libraries (evologics_usbl perls-lcmtypes perls-common proj)

perls_add_executable (evologics_modem main_modem.c evologics.c base91.c)
perls_target_link_libraries (evologics_modem perls-lcmtypes perls-common)
