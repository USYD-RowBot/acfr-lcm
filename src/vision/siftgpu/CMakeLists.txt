# EXTERNAL DEPS
perlsx_opencv ()
perlsx_opengl ()
perlsx_siftgpu ()

# SERVER
perls_add_executable (siftgpu-server server.cpp)
perls_target_link_libraries (siftgpu-server perls-lcmtypes perls-common perls-vision 
  ${PERLSX_SIFTGPU}
  ${PERLSX_OPENGL}
  ${PERLSX_OPENCV}
  )
set_target_properties (siftgpu-server
  PROPERTIES 
  COMPILE_FLAGS "-Wno-write-strings -O3 -Wall"
  )

# COMMAND LINE UTIL
perls_add_executable (siftgpu-client client.c)
perls_target_link_libraries (siftgpu-client perls-lcmtypes perls-common perls-vision
  ${PERLSX_OPENCV}
  )

perls_python_install_script (siftgpu-params params.py)
