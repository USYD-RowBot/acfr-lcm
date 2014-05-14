perls_add_executable (evologics-usbl main_usbl.cpp evologics.c base91.c)
perls_target_link_libraries (evologics-usbl perls-lcmtypes perls-common proj z)

perls_add_executable (evologics-modem main_modem.c evologics.c base91.c)
perls_target_link_libraries (evologics-modem perls-lcmtypes perls-common z)
