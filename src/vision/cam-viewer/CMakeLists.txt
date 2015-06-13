# EXTERNAL DEPS
perlsx_glib ()

# BUILD EXE
perls_add_executable (cam-viewer main.cpp)
perls_target_link_libraries (cam-viewer perls-lcmtypes perls-common 
  ${PERLSX_GLIB} turbojpeg opencv_core opencv_imgproc opencv_highgui	
  )

