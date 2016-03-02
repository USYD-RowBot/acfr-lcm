option (BUILD_PROSILICA "Build and install third-party Allied Vision GigE PvAPI Prosilica SDK?" ON)

if (BUILD_PROSILICA)
  set (PROSILICA_SRC "${THIRD_PARTY_DIR}/PvAPI_1.28_Linux.tgz")
  set (PROSILICA_DIR "PvAPI_1.28_Linux.tgz")

  if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set (PROSILICA_BIN ${PROSILICA_DIR}/bin-pc/x64)
  else ()
    set (PROSILICA_BIN ${PROSILICA_DIR}/bin-pc/x86)
  endif ()

  add_custom_target (prosilica
    COMMAND rm -f ${PROSILICA_DIR}/PERLS_BUILT
    COMMAND cmake ..
    COMMAND make prosilica-target
    )

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${PROSILICA_DIR}/PERLS_BUILT)
    add_custom_target (prosilica-target
      COMMAND mkdir -p ${PROSILICA_DIR}
      COMMAND tar zxvfp  ${PROSILICA_SRC} -C ${PROSILICA_DIR} --strip 1
      COMMAND sudo cp -f ${PROSILICA_DIR}/inc-pc/PvApi.h /usr/local/include/.
      COMMAND sudo cp -f ${PROSILICA_DIR}/inc-pc/PvRegIo.h /usr/local/include/.
      COMMAND sudo cp -f ${PROSILICA_BIN}/SampleViewer /usr/local/bin/prosilica-SampleViewer
      COMMAND sudo cp -f ${PROSILICA_BIN}/CLIpConfig /usr/local/bin/prosilica-CLIpConfig
      COMMAND sudo cp -f ${PROSILICA_BIN}/libPvAPI.so /usr/local/lib/.
      COMMAND sudo cp -f ${PROSILICA_BIN}/libPvJNI.so /usr/local/lib/.
      COMMAND sudo ldconfig
      COMMAND touch ${PROSILICA_DIR}/PERLS_BUILT
      COMMAND cmake ..
      )
    add_dependencies (third-party prosilica-target)
  else ()
    message (STATUS "Skipping built target prosilica")
  endif ()
  
  add_custom_target (uninstall-prosilica
    COMMAND sudo rm -f /usr/local/include/PvApi.h
    COMMAND sudo rm -f /usr/local/include/PvRegIo.h
    COMMAND sudo rm -f /usr/local/bin/prosilica-SampleViewer
    COMMAND sudo rm -f /usr/local/bin/prosilica-CLIpConfig
    COMMAND sudo rm -f /usr/local/lib/libPvAPI.so
    COMMAND sudo rm -f /usr/local/lib/libPvJNI.so
    COMMAND rm -f ${PROSILICA_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (uninstall uninstall-prosilica)

  add_custom_target (clean-prosilica
    COMMAND rm -f ${PROSILICA_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (clean clean-prosilica)
  
endif (BUILD_PROSILICA)
