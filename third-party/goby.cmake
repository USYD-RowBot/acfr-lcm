option (BUILD_GOBY "Build and install third-party Goby Soft?" OFF)
if (BUILD_GOBY)
  set (GOBY_DIR "goby-2.0")

  add_custom_target (goby
    COMMAND rm -f ${GOBY_DIR}/PERLS_BUILT
    COMMAND cmake .. -Dbuild_apps=OFF
    COMMAND make goby-target
    )

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${GOBY_DIR}/PERLS_BUILT)
    if (EXISTS ${GOBY_DIR})
      add_custom_target (goby-bzr
        COMMAND bzr up ${GOBY_DIR}
        )
    else ()
      add_custom_target (goby-bzr
        COMMAND bzr co --lightweight lp:goby ${GOBY_DIR}
        )
    endif ()

    add_custom_target (goby-target
      COMMAND cd ${GOBY_DIR} && mkdir -p bin && cd build && cmake .. -Dbuild_apps=OFF && make && sudo make install      
      COMMAND sudo ldconfig
      COMMAND touch ${GOBY_DIR}/PERLS_BUILT
      COMMAND cmake .. -Dbuild_apps=OFF
      )
    add_dependencies (goby-target goby-bzr)
    add_dependencies (third-party goby-target)
  else ()
    message (STATUS "Skipping built target goby")
  endif ()  

  add_custom_target (uninstall-goby
    COMMAND sudo rm -f /usr/local/lib/libgoby*
    COMMAND sudo rm -rf /usr/local/include/goby
    COMMAND sudo ldconfig
    COMMAND rm -f ${GOBY_DIR}/PERLS_BUILT
    COMMAND cmake .. -Dbuild_apps=OFF
    )
  add_dependencies (uninstall uninstall-goby)

  add_custom_target (clean-goby
    COMMAND cd ${GOBY_DIR}/build && make clean
    COMMAND rm -f ${GOBY_DIR}/PERLS_BUILT
    COMMAND cmake .. -Dbuild_apps=OFF
    )
  add_dependencies (clean clean-goby)
endif (BUILD_GOBY)
