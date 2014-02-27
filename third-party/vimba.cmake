option (BUILD_VIMBA "Build and install third-party Allied Vision Vimba SDK?" ON)

if (BUILD_VIMBA)
  set (VIMBA_SRC "${THIRD_PARTY_DIR}/AVTVimbaSDK_v1.2.tgz")
  set (VIMBA_DIR "Vimba_1_2")

  if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set (VIMBA_BIN x86_64bit)
  else ()
    set (VIMBA_BIN x86_32bit)
  endif ()

  add_custom_target (vimba
    COMMAND rm -f ${VIMBA_DIR}/PERLS_BUILT
    COMMAND cmake ..
    COMMAND make vimba-target
    )

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${VIMBA_DIR}/PERLS_BUILT)
    add_custom_target (prosilica-target
      COMMAND mkdir -p ${VIMBA_DIR}
      COMMAND tar zxvfp  ${VIMBA_SRC} -C ${VIMBA_DIR} --strip 1
      COMMAND sudo mkdir -p /usr/local/include/vimba
      COMMAND sudo cp -f ${VIMBA_DIR}/VimbaC/Include/* /usr/local/include/vimba/.
      COMMAND sudo cp -f ${VIMBA_DIR}/VimbaCPP/Include/* /usr/local/include/vimba/.
      COMMAND sudo cp -f ${VIMBA_DIR}/AVTImageTransform/Include/* /usr/local/include/vimba/.
      COMMAND sudo cp -f ${VIMBA_DIR}/VimbaC/DynamicLib/${VIMBA_BIN}/* /usr/local/lib/.
      COMMAND sudo cp -f ${VIMBA_DIR}/VimbaCPP/DynamicLib/${VIMBA_BIN}/* /usr/local/lib/.      
      COMMAND sudo cp -f ${VIMBA_DIR}/AVTImageTransform/DynamicLib/${VIMBA_BIN}/* /usr/local/lib/.      
      COMMAND sudo cp -f ${VIMBA_BIN}/Tools/Viewer/Bin/${VIMBA_BIN}/VimbaViewer /usr/local/bin/      
      COMMAND sudo ldconfig
      COMMAND touch ${VIMBA_DIR}/PERLS_BUILT
      COMMAND cmake ..
      )
    add_dependencies (third-party vimba-target)
  else ()
    message (STATUS "Skipping built target vimba")
  endif ()
  
  add_custom_target (uninstall-vimba
    COMMAND sudo rm -rf /usr/local/include/vimba
    COMMAND sudo rm -f /usr/local/bin/VimbaViewer
    COMMAND sudo rm -f /usr/local/lib/libVimbaCPP.so
    COMMAND sudo rm -f /usr/local/lib/libVimbaC.so
    COMMAND sudo rm -f /usr/local/lib/libAVTImageTransform.so
    COMMAND rm -f ${VIMBA_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (uninstall uninstall-vimba)

  add_custom_target (clean-vimba
    COMMAND rm -f ${VIMBA_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (clean clean-vimba)
  
endif (BUILD_VIMBA)
