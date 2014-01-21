find_package (Doxygen)
if (DOXYGEN_FOUND)
  configure_file (${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  configure_file (${CMAKE_CURRENT_SOURCE_DIR}/lcmdef_example.c.html.in ${CMAKE_CURRENT_BINARY_DIR}/lcmdef_example.c.html @ONLY)
  configure_file (${CMAKE_CURRENT_SOURCE_DIR}/post_transfer.py.html.in ${CMAKE_CURRENT_BINARY_DIR}/post_transfer.py.html @ONLY)
  configure_file (${CMAKE_CURRENT_SOURCE_DIR}/c_example.c.html.in ${CMAKE_CURRENT_BINARY_DIR}/c_example.c.html @ONLY)
  configure_file (${CMAKE_CURRENT_SOURCE_DIR}/c_example.h.html.in ${CMAKE_CURRENT_BINARY_DIR}/c_example.h.html @ONLY)

  # doc
  add_custom_target (doc
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating documentation with Doxygen" VERBATIM
    )

  # docs alias
  add_custom_target (docs)
  add_dependencies (docs doc)
else ()
  message (SEND_ERROR "Doxygen not found.  on ubuntu do `sudo apt-get install doxygen`")
endif ()
