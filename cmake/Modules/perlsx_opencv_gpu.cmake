macro (perlsx_opencv_gpu)
  perlsx_find_library (opencv_gpu "

   You must install opencv 2.3 with CUDA enabled.

" ${ARGN})
  if (OPENCV_GPU_FOUND)
    add_definitions (-DOPENCV_GPU_FOUND)
    set (PERLSX_OPENCV_GPU ${OPENCV_GPU_LIBRARIES})
  endif ()
endmacro ()
