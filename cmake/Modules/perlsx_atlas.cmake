macro (perlsx_atlas)
  perlsx_find_library (cblas "on ubuntu `sudo apt-get install libatlas-base-dev" ${ARGN})
  perlsx_find_library (atlas "on ubuntu `sudo apt-get install libatlas-base-dev" ${ARGN})
  if (CBLAS_FOUND AND ATLAS_FOUND)
    set (PERLSX_ATLAS ${CBLAS_LIBRARIES} ${ATLAS_LIBRARIES})
  endif ()
endmacro ()