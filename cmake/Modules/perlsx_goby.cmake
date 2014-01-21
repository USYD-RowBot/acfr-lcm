macro (perlsx_goby)
  perlsx_find_library (goby_acomms "did you build goby in third-party?" ${ARGN})
  perlsx_find_library (goby_common "did you build goby in third-party?" ${ARGN})
  perlsx_find_library (goby_pb "did you build goby in third-party?" ${ARGN})
  perlsx_find_library (goby_util "did you build goby in third-party?" ${ARGN})

  if (GOBY_ACOMMS_FOUND AND
      GOBY_COMMON_FOUND AND
      GOBY_PB_FOUND AND
      GOBY_UTIL_FOUND)
    set (PERLSX_GOBY
      ${GOBY_ACOMMS_LIBRARIES}
      ${GOBY_COMMON_LIBRARIES}
      ${GOBY_PB_LIBRARIES}
      ${GOBY_UTIL_LIBRARIES})
    else ()
      message (SEND_FATAL "goby not found")
  endif ()
endmacro ()
