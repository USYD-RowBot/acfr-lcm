option (BUILD_SEGWAY "Build and install third-party IRI-Segway driver?" OFF)
if (BUILD_SEGWAY)
  add_dependencies (third-party segway-target)
  add_dependencies (uninstall uninstall-segway)
  add_dependencies (clean clean-segway)

  # STEP 0: PARENT SEGWAY TARGET
  set (SEGWAY_SRC "${THIRD_PARTY_DIR}/iri-segway")
  set (SEGWAY_DIR "iri-segway")

  add_custom_target (segway
    COMMAND rm -f ${SEGWAY_DIR}/PERLS_BUILT
    COMMAND cmake ..
    COMMAND make segway-target
    )

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${SEGWAY_DIR}/PERLS_BUILT)
    add_custom_target (segway-target
      COMMAND touch ${SEGWAY_DIR}/PERLS_BUILT
      COMMAND cmake ..
      )
    add_dependencies (segway-target
      segway-ftdi 
      segway-util 
      segway-comm
      segway-driver
      )
  else ()
    message (STATUS "Skipping built target segway")    
  endif ()

  add_custom_target (uninstall-segway
    COMMAND rm -f ${SEGWAY_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (uninstall-segway 
    uninstall-segway-ftdi 
    uninstall-segway-util 
    uninstall-segway-comm
    uninstall-segway-driver
    )

  add_custom_target (clean-segway
    COMMAND rm -f ${SEGWAY_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (clean-segway
    clean-segway-ftdi 
    clean-segway-util
    clean-segway-comm
    clean-segway-driver
    )


  # STEP 1: FTDI
  set (FTDI_LIB "libftd2xx.so.0.4.16")
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set (FTDI_SRC "${SEGWAY_SRC}/libftd2xx0.4.16_x86_64.tar.gz")
    set (FTDI_DIR "${SEGWAY_DIR}/libftd2xx0.4.16_x86_64")
  else ()
    set (FTDI_SRC "${SEGWAY_SRC}/libftd2xx0.4.16.tar.gz")
    set (FTDI_DIR "${SEGWAY_DIR}/libftd2xx0.4.16")
  endif ()
  add_custom_target (segway-ftdi
    COMMAND mkdir -p ${FTDI_DIR}
    COMMAND tar zxvf  ${FTDI_SRC} -C ${FTDI_DIR} --strip 1
    COMMAND sudo cp -f ${FTDI_DIR}/${FTDI_LIB} /usr/local/lib/.
    COMMAND cd /usr/local/lib && sudo ln -sf ${FTDI_LIB} libftd2xx.so.0
    COMMAND cd /usr/local/lib && sudo ln -sf libftd2xx.so.0 libftd2xx.so
    COMMAND cd /usr/lib && sudo ln -sf /usr/local/lib/${FTDI_LIB} libftd2xx.so
    COMMAND sudo cp -f ${FTDI_DIR}/*.h /usr/local/include/.
    COMMAND sudo cp -f ${SEGWAY_SRC}/99-ftdi.rules /etc/udev/rules.d/.
    COMMAND sudo ldconfig
    )

  add_custom_target (uninstall-segway-ftdi
    COMMAND sudo rm -f /etc/udev/rules.d/99-ftdi.rules
    COMMAND sudo rm -f /usr/local/include/ftd2xx.h /usr/local/include/WinTypes.h
    COMMAND sudo rm -f /usr/lib/libftd2xx.so
    COMMAND sudo rm -f /usr/local/lib/libftd2xx.so
    COMMAND sudo rm -f /usr/local/lib/libftd2xx.so.0
    COMMAND sudo rm -f /usr/local/lib/${FTDI_LIB}
    COMMAND sudo ldconfig
    )
  add_dependencies (uninstall-segway-ftdi uninstall-segway-util)

  add_custom_target (clean-segway-ftdi)


  # STEP 2: UTILITIES
  set (UTIL_SRC "${SEGWAY_SRC}/utilities-r71.tar.gz")
  set (UTIL_DIR "${SEGWAY_DIR}/utilities-r71")
  add_custom_target (segway-util
    COMMAND mkdir -p ${UTIL_DIR}
    COMMAND tar zxvf ${UTIL_SRC} -C ${UTIL_DIR} --strip 1
    COMMAND cd ${UTIL_DIR}/build && cmake .. && make && sudo make install
    COMMAND sudo ldconfig
    )
  add_dependencies (segway-util segway-ftdi)

  add_custom_target (uninstall-segway-util
    COMMAND cd ${UTIL_DIR}/build && sudo make uninstall
    COMMAND sudo ldconfig
    )
  add_dependencies (uninstall-segway-util uninstall-segway-comm)

  add_custom_target (clean-segway-util
    COMMAND cd ${UTIL_DIR}/build && make clean
    )


  # STEP 3: COMMUNICATIONS
  set (COMM_SRC "${SEGWAY_SRC}/communications-r34.tar.gz")
  set (COMM_DIR "${SEGWAY_DIR}/communications-r71")
  add_custom_target (segway-comm
    COMMAND mkdir -p ${COMM_DIR}
    COMMAND tar zxvf ${COMM_SRC} -C ${COMM_DIR} --strip 1
    COMMAND cd ${COMM_DIR}/build && cmake .. && make && sudo make install
    COMMAND sudo ldconfig
    )
  add_dependencies (segway-comm segway-util)

  add_custom_target (uninstall-segway-comm
    COMMAND cd ${COMM_DIR}/build && sudo make uninstall
    COMMAND sudo ldconfig
    )
  add_dependencies (uninstall-segway-comm uninstall-segway-driver)

  add_custom_target (clean-segway-comm
    COMMAND cd ${COMM_DIR}/build && make clean
    )


  # STEP 4: DRIVER
  set (DRIVER_SRC "${SEGWAY_SRC}/segway_rmp_200-r25.tar.gz")
  set (DRIVER_DIR "${SEGWAY_DIR}/segway_rpm_200-r25")
  add_custom_target (segway-driver
    COMMAND mkdir -p ${DRIVER_DIR}
    COMMAND tar zxvf ${DRIVER_SRC} -C ${DRIVER_DIR} --strip 1
    COMMAND cd ${DRIVER_DIR}/build && cmake .. && make && sudo make install
    COMMAND sudo ldconfig
    )
  add_dependencies (segway-driver segway-comm)

  add_custom_target (uninstall-segway-driver
    COMMAND cd ${DRIVER_DIR}/build && sudo make uninstall
    COMMAND sudo ldconfig
    )

  add_custom_target (clean-segway-driver
    COMMAND cd ${DRIVER_DIR}/build && make clean
    )

endif (BUILD_SEGWAY)
