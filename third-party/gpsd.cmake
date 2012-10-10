option (BUILD_GPSD3 "Build and install third-party GPSD3 (for ppsboard patch)?" OFF)
if (BUILD_GPSD3)
  set (GPSD3_SRC "${THIRD_PARTY_DIR}/gpsd-2.92-ppsboard.tgz")
  set (GPSD3_DIR "gpsd-2.92")

  add_custom_target (gpsd3
    COMMAND rm -f ${GPSD3_DIR}/PERLS_BUILT
    COMMAND cmake ..
    COMMAND make gpsd3-target
    )

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${GPSD3_DIR}/PERLS_BUILT)
    add_custom_target (gpsd3-target
      COMMAND mkdir -p ${GPSD3_DIR}
      COMMAND tar zxvf  ${GPSD3_SRC} -C ${GPSD3_DIR} --strip 1
      COMMAND cd ${GPSD3_DIR} && patch -p1 < ${THIRD_PARTY_DIR}/gpsd-2.92-ppsboard.patch
      COMMAND cd ${GPSD3_DIR} && ./configure && make
      COMMAND cd ${GPSD3_DIR} && sudo make install
      COMMAND sudo ldconfig
      COMMAND touch ${GPSD3_DIR}/PERLS_BUILT
      COMMAND cmake ..
      )
    add_dependencies (third-party gpsd3-target)
  else ()
    message (STATUS "Skipping built target gpsd3")
  endif ()

  add_custom_target (uninstall-gpsd3
    COMMAND cd ${GPSD3_DIR} && sudo make uninstall
    COMMAND sudo ldconfig
    COMMAND rm -f ${GPSD3_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (uninstall uninstall-gpsd3)

  add_custom_target (clean-gpsd3
    COMMAND cd ${GPSD3_DIR} && sudo make clean
    COMMAND rm -f ${GPSD3_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (clean clean-gpsd3)
endif (BUILD_GPSD3)
