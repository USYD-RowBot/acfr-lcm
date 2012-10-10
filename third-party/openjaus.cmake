option (BUILD_OPENJAUS "Build and install third-party OpenJAUS?" OFF)
if (BUILD_OPENJAUS)
  set (OPENJAUS_SRC "${THIRD_PARTY_DIR}/OpenJAUSv3.3.0b-SDK-Linux.tar.gz")
  set (OPENJAUS_DIR "openjaus-v3.3b")

  add_custom_target (openjaus
    COMMAND rm -f ${OPENJAUS_DIR}/PERLS_BUILT
    COMMAND cmake ..
    COMMAND make openjaus-target
    )

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${OPENJAUS_DIR}/PERLS_BUILT)
    add_custom_target (openjaus-target
      COMMAND mkdir -p ${OPENJAUS_DIR}
      COMMAND tar zxvf ${OPENJAUS_SRC} -C ${OPENJAUS_DIR}
      COMMAND cd ${OPENJAUS_DIR} && patch -p1 < ${THIRD_PARTY_DIR}/OpenJAUSv3.3.0b.patch
      COMMAND make -C ${OPENJAUS_DIR}/libjaus
      COMMAND sudo mkdir -p /usr/local/include/jaus
      COMMAND sudo cp -rf ${OPENJAUS_DIR}/libjaus/include/* /usr/local/include/jaus/.
      COMMAND sudo cp -f ${OPENJAUS_DIR}/libjaus/lib/libjaus.so /usr/local/lib/.
      COMMAND make -C ${OPENJAUS_DIR}/libopenJaus
      COMMAND sudo mkdir -p /usr/local/include/openJaus
      COMMAND sudo cp -rf ${OPENJAUS_DIR}/libopenJaus/include/* /usr/local/include/openJaus/.
      COMMAND sudo cp -f ${OPENJAUS_DIR}/libopenJaus/lib/libopenJaus.so /usr/local/lib/.
      COMMAND sudo ldconfig
      COMMAND touch ${OPENJAUS_DIR}/PERLS_BUILT
      COMMAND cmake ..
      )
    add_dependencies (third-party openjaus-target)
  else ()
    message (STATUS "Skipping built target openjaus")
  endif ()

  add_custom_target (uninstall-openjaus
    COMMAND sudo rm -rf /usr/local/include/jaus
    COMMAND sudo rm -rf /usr/local/include/openJaus
    COMMAND sudo rm -f /usr/local/lib/libjaus.so
    COMMAND sudo rm -f /usr/local/lib/libopenJaus.so
    COMMAND sudo ldconfig
    COMMAND rm -f ${OPENJAUS_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (uninstall uninstall-openjaus)

  add_custom_target (clean-openjaus
    COMMAND cd ${OPENJAUS_DIR} && make clean
    COMMAND rm -f ${OPENJAUS_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (clean clean-openjaus)
endif (BUILD_OPENJAUS)
