perls_add_executable (owtt-client client.cpp 
  local_manager.h local_manager.cpp
  platform_utils.h platform_utils.cpp) 
perls_target_link_libraries (owtt-client 
  perls-owtt-nav 
  perls-origin-state
  perls-math
  perls-common
  perls-lcmtypes
  ${PERLSX_BOOST}
  )

perls_add_executable (owtt-server server.cpp 
  local_manager.h local_manager.cpp 
  platform_utils.h platform_utils.cpp) 
perls_target_link_libraries (owtt-server 
  perls-owtt-nav 
  perls-origin-state
  perls-math
  perls-common
  perls-lcmtypes
  ${PERLSX_BOOST}
  )

perls_python_install_script (owtt-gps-support gps_support.py)

#perls_add_executable (owtt-centralized centralized.h centralized.cpp)
#perls_target_link_libraries (owtt-centralized
#  perls-owtt-nav 
#  perls-origin-state
#  perls-math
#  perls-common
#  perls-lcmtypes
#  ${PERLSX_BOOST}
#  )

