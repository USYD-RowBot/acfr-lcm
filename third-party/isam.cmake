option (BUILD_ISAM "Build and install third-party iSAM?" OFF)
if (BUILD_ISAM)  
  set (ISAM_SRC "${THIRD_PARTY_DIR}/isam-r5863.tar.gz")
  set (ISAM_DIR "isam-r5863")

  add_custom_target (isam
    COMMAND rm -f ${ISAM_DIR}/PERLS_BUILT
    COMMAND cmake ..
    COMMAND make isam-target
    )

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${ISAM_DIR}/PERLS_BUILT)
    add_custom_target (isam-target
      COMMAND mkdir -p ${ISAM_DIR}
      COMMAND tar zxvfp  ${ISAM_SRC} -C ${ISAM_DIR} --strip 1
      COMMAND cd ${ISAM_DIR} && pwd && make && sudo make install
      COMMAND sudo ldconfig
      COMMAND touch ${ISAM_DIR}/PERLS_BUILT
      COMMAND cmake ..
      )
    add_dependencies (third-party isam-target)
  else ()
    message (STATUS "Skipping built target isam")
  endif ()

  add_custom_target (uninstall-isam
    COMMAND cd ${ISAM_DIR}/ && sudo make clean
    COMMAND sudo rm -rf /usr/local/include/isam
    COMMAND sudo rm -rf /usr/local/lib/libisam.a
    COMMAND sudo ldconfig
    COMMAND rm -f ${ISAM_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (uninstall uninstall-isam)
  
  add_custom_target (clean-isam
    COMMAND cd ${ISAM_DIR} && sudo make clean
    COMMAND rm -f ${ISAM_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (clean clean-isam)
endif (BUILD_ISAM)
