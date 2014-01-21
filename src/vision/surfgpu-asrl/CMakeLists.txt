# EXTERNAL DEPS
perlsx_opencv ()
perlsx_opengl ()
perlsx_surfgpu ()

perls_add_executable (surfgpu-asrl-server server.cpp)
perls_target_link_libraries (surfgpu-asrl-server perls-lcmtypes perls-common perls-vision 
  ${PERLSX_SURFGPU}
  ${PERLSX_OPENCV}
  )
set_target_properties (surfgpu-asrl-server
  PROPERTIES 
  COMPILE_FLAGS "-Wno-write-strings -O3 -Wall"
  )

# COMMAND LINE UTIL
perls_add_executable (surfgpu-client client.c)
perls_target_link_libraries (surfgpu-client perls-lcmtypes perls-common perls-vision
  ${PERLSX_OPENCV}
  )
