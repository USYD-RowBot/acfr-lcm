macro (perlsx_imagemagick)
  perlsx_find_package (ImageMagick "on ubuntu `sudo apt-get install libmagick9-dev libmagick++9-dev`" ${ARGN}
    COMPONENTS MagickWand MagickCore Magick++
    )
  if (ImageMagick_FOUND)
    include_directories (${ImageMagick_INCLUDE_DIRS})
    set (PERLSX_IMAGEMAGICK ${ImageMagick_LIBRARIES})
  endif ()
endmacro ()
