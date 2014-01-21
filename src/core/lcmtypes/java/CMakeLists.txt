foreach (PACKAGE ${PERLS_LCM_PACKAGES})
  # round up .lcm definition files
  file (GLOB LCM_DEFS ${PERLS_LCMDEFS_DIR}/${PACKAGE}/*.lcm)

  # emit java lcmtypes
  foreach (LCM_DEF ${LCM_DEFS})
    get_filename_component (LCM_TYPE ${LCM_DEF} NAME_WE)
    string (REGEX REPLACE "^${PACKAGE}_" "" LCM_TYPE ${LCM_TYPE})

    set (LCM_TYPE_JAVA ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE}/${LCM_TYPE}.java)
    set (LCM_TYPES_JAVA  ${LCM_TYPES_JAVA}  ${LCM_TYPE_JAVA})

    add_custom_command (
      OUTPUT ${LCM_TYPE_JAVA}
      COMMAND ${LCM_GEN_EXE} ${LCM_DEF} --lazy --java --jpath ${CMAKE_CURRENT_BINARY_DIR}
      DEPENDS ${LCM_DEF}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      )
  endforeach () # foreach LCM_DEF
endforeach () #foreach PACKAGE

perls_java_add_library (perls_lcmtypes.jar ${LCM_TYPES_JAVA})
