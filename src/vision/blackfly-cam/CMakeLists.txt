# BUILD EXE
set(CMAKE_C_FLAGS "-O3 -std=gnu11")
perls_add_executable (blackfly-cam blackfly.c)
perls_target_link_libraries (blackfly-cam perls-lcmtypes  flycapture-c tiff acfr-common pthread
  )
