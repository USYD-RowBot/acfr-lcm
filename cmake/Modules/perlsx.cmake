# helpful functions for finding perls external dependencies

#===============================================================================
macro (perlsx_error)
  message (FATAL_ERROR ${ARGN}) # wrapper so that all perlsx error messages have same level
endmacro ()

#===============================================================================
macro (perlsx_find_library LIB INSTALLMSG)
  string (TOUPPER ${LIB} LIBUP)

  set (IS_REQUIRED true)
  foreach(arg ${ARGN})
    if ("${arg}" MATCHES "OPTIONAL")
      set (IS_REQUIRED false)
    else () # pass to find_library()
      if (${LIBARGN})
        set(LIBARGN "${LIBARGN} ${arg}")
      else ()
        set(LIBARGN "${arg}") # No space in front of PKGARGN
      endif()
    endif()
  endforeach(arg ${ARGN})

  # no need to try again if already found
  if (NOT ${LIBUP}_LIBRARIES)
    find_library (${LIBUP}_LIBRARIES ${LIB} # mimick find_package lib var name
      DOC "Fullpath to ${LIB} library."
      ${LIBARGN}
      )
    mark_as_advanced (FORCE ${LIBUP}_LIBRARIES)
  endif ()

  if (${LIBUP}_LIBRARIES)
    set (${LIBUP}_FOUND TRUE)
  else ()
    set (${LIBUP}_FOUND FALSE)
      if (IS_REQUIRED)
        perlsx_error ("lib${LIB} not found\; ${INSTALLMSG}")
      endif()
  endif ()  
endmacro ()

#===============================================================================
macro (perlsx_find_package PKG INSTALLMSG)
  string (TOUPPER ${PKG} PKGUP)

  set (IS_REQUIRED true)
  set (PKGARGN "")
  foreach(arg ${ARGN})
    if ("${arg}" MATCHES "OPTIONAL")
      set (IS_REQUIRED false)
    else () # pass to find_package()
      if (${PKGARGN})
        set(PKGARGN "${PKGARGN} ${arg}")
      else ()
        set(PKGARGN "${arg}") # No space in front of PKGARGN
      endif()
    endif()
  endforeach(arg ${ARGN})

  # no need to try again if already found
  if ( (NOT ${PKG}_FOUND) AND (NOT ${PKGUP}_FOUND) )
    find_package (${PKG} ${PKGARGN})
  endif ()

  if ( (NOT ${PKG}_FOUND) AND (NOT ${PKGUP}_FOUND) AND ${IS_REQUIRED})
    perlsx_error ("package ${PKG} not found\; ${INSTALLMSG}")
  else ()
    set (${PKGUP}_FOUND TRUE)
  endif ()
endmacro ()
