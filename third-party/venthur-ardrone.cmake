option (BUILD_ARDRONE "Build and install third-party Venthur ARDrone driver?" OFF)
if (BUILD_ARDRONE)
  set (ARDRONE_SRC "${THIRD_PARTY_DIR}/venthur-python-ardrone-16522be.tar.gz")
  set (ARDRONE_DIR "${THIRD_PARTY_DIR}/venthur_drone")

  add_custom_target (ardrone
    COMMAND rm -f ${ARDRONE_DIR}/PERLS_BUILT
    COMMAND cmake ..
    COMMAND make ardone-target
    )

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${ARDRONE_DIR}/PERLS_BUILT)
    add_custom_target (ardrone-target
      COMMAND mkdir -p ${ARDRONE_DIR}
      COMMAND tar xvzf ${ARDRONE_SRC} -C ${ARDRONE_DIR} --strip 1
      COMMAND cd ${ARDRONE_DIR} && patch -p1 < ${THIRD_PARTY_DIR}/venthur.patch
      COMMAND ${THIRD_PARTY_DIR}/venthur_ardrone.sh install
      COMMAND touch ${ARDRONE_DIR}/PERLS_BUILT
      )
    add_dependencies (third-party ardrone-target)
  else ()
    message (STATUS "Skipping built target venthur-drone")
  endif ()  

  add_custom_target (uninstall-ardrone
    COMMAND rm -rf ${ARDRONE_DIR}
    COMMAND ${THIRD_PARTY_DIR}/venthur_ardrone.sh uninstall
    COMMAND cmake ..
    )
  add_dependencies (uninstall uninstall-ardrone)

  add_custom_target (clean-ardrone
    COMMAND rm -f ${ARDRONE_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (clean clean-ardrone)
endif (BUILD_ARDRONE)
