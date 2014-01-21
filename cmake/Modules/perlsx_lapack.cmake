macro (perlsx_lapack)
  # find_package(LAPACK) doesn't seem to be working on ubuntu 10.04, so revert to using find_library
  perlsx_find_library (lapack "on ubuntu `sudo apt-get install liblapack-dev`" ${ARGN})
  if (LAPACK_FOUND)
    set (PERLSX_LAPACK ${LAPACK_LIBRARIES})
  endif ()
endmacro ()
