macro (perlsx_las)
  perlsx_find_package (LAS "on ubuntu build and install from source v1.7 http://liblas.org/download.html" ${ARGN})
  mark_as_advanced (
    LAS_C_LIBRARY
    LAS_C_LIBRARY_DEBUG
    LAS_INCLUDE_DIR
    LAS_LIBRARY
    LAS_LIBRARY_DEBUG
    )
    
  if (LAS_FOUND)
    include_directories (${LAS_INCLUDE_DIR})
    set (PERLSX_LAS ${LAS_LIBRARIES})
  endif ()
endmacro ()
