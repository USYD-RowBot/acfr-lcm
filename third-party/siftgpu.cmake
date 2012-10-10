option (BUILD_SIFTGPU "Build and install third-party SiftGPU? (requires Nvidia card)" OFF)
if (BUILD_SIFTGPU)
  # requires libglew
  find_package (GLEW)
  if (GLEW_FOUND)
    mark_as_advanced (GLEW_INCLUDE_DIR GLEW_LIBRARY)
  else ()
    message (SEND_ERROR "on ubuntu `sudo apt-get install libglew-dev`")
  endif ()

  # requires libdevil
  find_package (IL)
  if (IL_FOUND)
    mark_as_advanced (ILUT_LIBRARY ILU_LIBRARY IL_INCLUDE_DIR IL_LIBRARY)
  else ()
    message (SEND_ERROR "on ubuntu `sudo apt-get install libdevil-dev`")
  endif ()

#   # requires cuda
#   find_library (LIB_CUDA cudart 
#     PATHS /usr/local/cuda/lib64 /usr/local/cuda/lib
#     )
#   if (LIB_CUDA)
#     mark_as_advanced (LIB_CUDA)
#   else ()
#     message (SEND_ERROR "CUDA library not found; download CUDA toolkit for ubuntu at http://www.nvidia.com/getcuda and install to /usr/local/cuda\n
#                          CUDA installation instructions can be found at http://moelhave.dk/2009/12/nvidia-cuda-on-ubuntu-karmic-koala/")
#   endif ()
  
  set (SIFTGPU_SRC "${THIRD_PARTY_DIR}/SiftGPU-V360.zip")
  set (SIFTGPU_DIR "SiftGPU-V360")

  add_custom_target (siftgpu
    COMMAND rm -f ${SIFTGPU_DIR}/PERLS_BUILT
    COMMAND cmake ..
    COMMAND make siftgpu-target
    )

  if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${SIFTGPU_DIR}/PERLS_BUILT)
    add_custom_target (siftgpu-target
      COMMAND mkdir -p ${SIFTGPU_DIR}
      COMMAND unzip -o ${SIFTGPU_SRC}
      COMMAND cd ${SIFTGPU_DIR} && patch -p1 < ${THIRD_PARTY_DIR}/SiftGPU-V360.patch
      COMMAND make -C ${SIFTGPU_DIR}/linux all
      COMMAND sudo mkdir -p /usr/local/include/siftgpu
      COMMAND sudo cp -f ${SIFTGPU_DIR}/src/SiftGPU/SiftGPU.h /usr/local/include/siftgpu/.
      COMMAND sudo cp -f ${SIFTGPU_DIR}/linux/bin/libsiftgpu.so /usr/local/lib/.
      COMMAND sudo ldconfig
      COMMAND touch ${SIFTGPU_DIR}/PERLS_BUILT
      COMMAND cmake ..
      )
    add_dependencies (third-party siftgpu-target)
  else ()
    message (STATUS "Skipping built target siftgpu")
  endif ()

  add_custom_target (uninstall-siftgpu
    COMMAND sudo rm -f /usr/local/include/siftgpu/SiftGPU.h
    COMMAND sudo rm -f /usr/local/lib/libsiftgpu.so
    COMMAND sudo ldconfig
    COMMAND rm -f ${SIFTGPU_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (uninstall uninstall-siftgpu)

  add_custom_target (clean-siftgpu
    COMMAND cd ${SIFTGPU_DIR}/linux && make clean
    COMMAND rm -f ${SIFTGPU_DIR}/PERLS_BUILT
    COMMAND cmake ..
    )
  add_dependencies (clean clean-siftgpu)
endif (BUILD_SIFTGPU)
