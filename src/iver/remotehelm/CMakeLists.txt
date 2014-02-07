# EXTERNAL DEPS
perlsx_glib ()
perlsx_m ()

perls_add_executable (remotehelm
                      main.c      remotehelm.h
                      dir_mgmt.c  dir_mgmt.h
                      callbacks.c callbacks.h)
perls_target_link_libraries (remotehelm perls-lcmtypes perls-common perls-iver
  ${PERLSX_GLIB}
  ${PERLSX_M}
  )

perls_python_install_script (rh-jump-abort rh-jump-abort.py)

perls_python_install_script (post-transfer post_transfer.py)

configure_file (PersistentLcmLogger.py.in PersistentLcmLogger.py @ONLY)
perls_python_install_script (persistent-lcm-logger ${CMAKE_CURRENT_BINARY_DIR}/PersistentLcmLogger.py)

configure_file (TopsideLogger.py.in TopsideLogger.py @ONLY)
perls_python_install_script (topside-lcm-logger ${CMAKE_CURRENT_BINARY_DIR}/TopsideLogger.py)

perls_python_install_script (remotehelm-controller RemotehelmController.py)
perls_python_install_script (merge-dives merge_dives.py)
