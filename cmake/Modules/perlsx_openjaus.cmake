macro (perlsx_openjaus)
  perlsx_find_library (jaus "did you install openjaus in third-party?" ${ARGN})
  perlsx_find_library (openJaus "did you install openjaus in third-party?" ${ARGN})

  if (JAUS_FOUND)
    include_directories (${JAUS_INCLUDE_DIR})
    set (PERLSX_JAUS ${JAUS_LIBRARIES})
  endif ()
  if (OPENJAUS_FOUND)
    include_directories (${OPENJAUS_INCLUDE_DIR})
    set (PERLSX_OPENJAUS ${OPENJAUS_LIBRARIES})
  endif ()
endmacro ()