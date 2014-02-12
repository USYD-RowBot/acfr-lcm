macro (perlsx_jpeg)
  perlsx_find_package (JPEG "on ubuntu `sudo apt-get install libjpeg-dev`" ${ARGN})
  if (JPEG_FOUND)
    include_directories (${JPEG_INCLUDE_DIR})
    set (PERLSX_JPEG ${JPEG_LIBRARIES})
  endif ()
endmacro ()
