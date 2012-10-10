perls_bash_install_script (ptunnel-vehicle ptunnel-vehicle.sh)

configure_file (ptunnel-topside.sh.in ptunnel-topside.sh @ONLY)
perls_bash_install_script (ptunnel-topside ${CMAKE_CURRENT_BINARY_DIR}/ptunnel-topside.sh)
