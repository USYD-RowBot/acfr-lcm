option (BUILD_LCM "Build and install third-party LCM?" ON)
if (BUILD_LCM)
  set (LCM_SRC "${THIRD_PARTY_DIR}/lcm-1.2.0.tar.gz")
  set (LCM_DIR "lcm-1.2.0")
    
  add_custom_target (lcm
    COMMAND rm -f ${LCM_DIR}/PERLS_BUILT
    COMMAND cmake ..
    COMMAND make lcm-target
    )

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${LCM_DIR}/PERLS_BUILT)
    add_custom_target (lcm-target
      COMMAND mkdir -p ${LCM_DIR}
      COMMAND tar zxvfp  ${LCM_SRC} -C ${LCM_DIR} --strip 1
      COMMAND cd ${LCM_DIR} && ./bootstrap.sh
      COMMAND cd ${LCM_DIR} && ./configure
      COMMAND make -C ${LCM_DIR} -j
      COMMAND sudo make -C ${LCM_DIR} install
      COMMAND sudo ldconfig
      COMMAND touch ${LCM_DIR}/PERLS_BUILT
      COMMAND cmake ..
      )
    add_dependencies (third-party lcm-target)
  else ()
    message (STATUS "Skipping built target lcm")
  endif ()

  add_custom_target (uninstall-lcm
    COMMAND cd ${LCM_DIR} && sudo make uninstall
    COMMAND sudo ldconfig
    COMMAND rm -f ${LCM_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (uninstall uninstall-lcm)

  add_custom_target (clean-lcm
    COMMAND cd ${LCM_DIR} && make clean
    COMMAND rm -f ${LCM_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (clean clean-lcm)
endif (BUILD_LCM)
