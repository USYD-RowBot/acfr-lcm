macro (perlsx_blas)
  # find_package(BLAS) doesn't seem to be working on ubuntu 10.04, so revert to using find_library()
  perlsx_find_library (blas "on ubuntu `sudo apt-get install libblas-dev`" ${ARGN})
  if (BLAS_FOUND)
    set (PERLSX_BLAS ${BLAS_LIBRARIES})
  endif ()
endmacro ()