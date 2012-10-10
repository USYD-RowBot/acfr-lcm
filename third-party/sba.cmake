option (BUILD_SBA "Build and install third-party Sparse Bundle Adjustment?" ON)
if (BUILD_SBA)
  # requires libf2c2-dev
  find_library (LIB_F2C f2c
    DOC "Fullpath to f2c2 library.")
  if (LIB_F2C)
    mark_as_advanced (LIB_F2C)
  else ()
    message (SEND_ERROR "on ubuntu `sudo apt-get install libf2c2-dev`")
  endif ()
  
  # requires lapack
  find_library (LIB_LAPACK lapack 
    DOC "Fullpath to lapack library.")
  if (LIB_LAPACK)
    mark_as_advanced (LIB_LAPACK)
  else ()
    message (SEND_ERROR "on ubuntu `sudo apt-get install liblapack-dev`")
  endif ()

  # requires blas
  find_library (LIB_BLAS blas
    DOC "Fullpath to blas library.")
  if (LIB_BLAS)
    mark_as_advanced (LIB_BLAS)
  else ()
    message (SEND_ERROR "on ubuntu `sudo apt-get install libblas-dev`")
  endif ()

  set (SBA_SRC "${THIRD_PARTY_DIR}/sba-1.6.tgz")
  set (SBA_DIR "sba-1.6")

  add_custom_target (sba
    COMMAND rm -f ${SBA_DIR}/PERLS_BUILT
    COMMAND cmake ..
    COMMAND make sba-target
    )

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${SBA_DIR}/PERLS_BUILT)
    add_custom_target (sba-target
      COMMAND mkdir -p ${SBA_DIR}
      COMMAND tar zxvf  ${SBA_SRC} -C ${SBA_DIR} --strip 1
      COMMAND cd ${SBA_DIR} && patch -p1 < ${THIRD_PARTY_DIR}/sba-1.6.patch
      COMMAND make -C ${SBA_DIR}
      COMMAND sudo mkdir -p /usr/local/include/sba
      COMMAND sudo cp -f ${SBA_DIR}/sba.h /usr/local/include/sba/.
      COMMAND sudo cp -f ${SBA_DIR}/libsba.so /usr/local/lib/.
      COMMAND sudo ldconfig
      COMMAND touch ${SBA_DIR}/PERLS_BUILT
      COMMAND cmake ..
      )
    add_dependencies (third-party sba-target)
  else ()
    message (STATUS "Skipping built target sba")
  endif ()

  add_custom_target (uninstall-sba
    COMMAND sudo rm -f /usr/local/include/sba/sba.h
    COMMAND sudo rm -f /usr/local/lib/libsba.so
    COMMAND sudo ldconfig
    COMMAND rm -f ${SBA_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (uninstall uninstall-sba)

  add_custom_target (clean-sba
    COMMAND cd ${SBA_DIR} && make clean
    COMMAND rm -f ${SBA_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (clean clean-sba)
endif (BUILD_SBA)
