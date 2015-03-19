macro (perlsx_bot2)
  set (INSTALLMSG "did you build libbot2 in third-party?")
  perlsx_find_library (bot2-core ${INSTALLMSG} ${ARGN})
#  perlsx_find_library (bot2-frames ${INSTALLMSG} ${ARGN})
#  perlsx_find_library (bot2-frames-renderers ${INSTALLMSG} ${ARGN})
#  perlsx_find_library (bot2-lcmgl-client ${INSTALLMSG} ${ARGN})
#  perlsx_find_library (bot2-lcmgl-renderer ${INSTALLMSG} ${ARGN})
  perlsx_find_library (bot2-param-client ${INSTALLMSG} ${ARGN})
#  perlsx_find_library (bot2-vis ${INSTALLMSG} ${ARGN})


  if (BOT2-CORE_FOUND)
      LIST(APPEND PERLSX_BOT2 ${BOT2-CORE_LIBRARIES})
  endif()
  
  if (BOT2-FRAMES_FOUND)
      LIST(APPEND PERLSX_BOT2 ${BOT2-FRAMES_LIBRARIES})
  endif()
  
  if (BOT2-FRAMES-RENDERERS_FOUND)
      LIST(APPEND PERLSX_BOT2 ${BOT2-FRAME-RENDERERS_LIBRARIES})
  endif()
  
  if (BOT2-LCMGL-CLIENT_FOUND)
      LIST(APPEND PERLSX_BOT2 ${BOT2-LCMGL-CLIENT_LIBRARIES})
  endif()
  
  if (BOT2-LCMGL-RENDERER_FOUND)
      LIST(APPEND PERLSX_BOT2 ${BOT2-LCMGL-RENDERER_LIBRARIES})
  endif()
  
  if (BOT2-PARAM-CLIENT_FOUND)
      LIST(APPEND PERLSX_BOT2 ${BOT2-PARAM-CLIENT_LIBRARIES})
  endif()
  
  if (BOT2-VIS_FOUND)
      LIST(APPEND PERLSX_BOT2 ${BOT2-VIS_LIBRARIES})
  endif()
  
#   AND
#      BOT2-FRAMES_FOUND AND
#      BOT2-FRAMES-RENDERERS_FOUND AND
#      BOT2-LCMGL-CLIENT_FOUND AND
#      BOT2-LCMGL-RENDERER_FOUND AND
#      BOT2-PARAM-CLIENT_FOUND AND
#      BOT2-VIS_FOUND
#      )
#    set (PERLSX_BOT2 
#      ${BOT2-CORE_LIBRARIES}
#      ${BOT2-FRAMES_LIBRARIES}
#      ${BOT2-FRAMES-RENDERERS_LIBRARIES}
#      ${BOT2-LCMGL-CLIENT_LIBRARIES}
#      ${BOT2-LCMGL-RENDERER_LIBRARIES}
#      ${BOT2-PARAM-CLIENT_LIBRARIES}
#      ${BOT2-VIS_LIBRARIES}
#      )
#  else ()
#    perlsx_error (${INSTALLMSG})
#  endif ()

  # to link against bot2, you also need to link against glib, so be
  # helpful and include it here
  perlsx_glib ()
  if (GLIB_FOUND)
    set (PERLSX_BOT2 ${PERLSX_BOT2} ${PERLSX_GLIB})
  endif ()
endmacro ()
