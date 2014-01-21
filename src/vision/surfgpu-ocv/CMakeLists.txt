# EXTERNAL DEPS
perlsx_opencv ()
perlsx_opengl ()
perlsx_opencv_gpu ()

perls_add_executable (surfgpu-ocv-server server.cpp)
perls_target_link_libraries (surfgpu-ocv-server perls-lcmtypes perls-common perls-vision 
  ${PERLSX_OPENCV_GPU}
  ${PERLSX_OPENCV}
  )
set_target_properties (surfgpu-ocv-server
  PROPERTIES 
  COMPILE_FLAGS "-Wno-write-strings -O3 -Wall"
  )

# COMMAND LINE UTIL
perls_add_executable (surfgpu-ocv-client client.c)
perls_target_link_libraries (surfgpu-ocv-client perls-lcmtypes perls-common perls-vision
  ${PERLSX_OPENCV}
  )

perls_python_install_script (surfgpu-ocv-params params.py)
