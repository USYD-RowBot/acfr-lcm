option (BUILD_SMALL "Build and install third-party SMALL?" ON)
if (BUILD_SMALL)
  set (SMALL_SRC "${THIRD_PARTY_DIR}/small.tar.gz")
  set (SMALL_DIR "small")
    
  add_custom_target (small
    COMMAND rm -f ${SMALL_DIR}/PERLS_BUILT
    COMMAND cmake ..
    COMMAND make small-target
    )

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${SMALL_DIR}/PERLS_BUILT)
    add_custom_target (small-target
      COMMAND mkdir -p ${SMALL_DIR}
      COMMAND tar -zxvf  ${SMALL_SRC} -C ${SMALL_DIR} --strip 1
      COMMAND cd ${SMALL_DIR} && cmake .
      COMMAND make -C ${SMALL_DIR} -j
      COMMAND sudo make -C ${SMALL_DIR} install
      COMMAND sudo ldconfig
      COMMAND touch ${SMALL_DIR}/PERLS_BUILT
      COMMAND cmake ..
      )
    add_dependencies (third-party small-target)
  else ()
    message (STATUS "Skipping built target small")
  endif ()

  add_custom_target (uninstall-small
    COMMAND cd ${SMALL_DIR} && sudo make uninstall
    COMMAND sudo ldconfig
    COMMAND rm -f ${SMALL_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (uninstall uninstall-small)

  add_custom_target (clean-small
    COMMAND cd ${SMALL_DIR} && make clean
    COMMAND rm -f ${SMALL_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (clean clean-small)
endif (BUILD_SMALL)
