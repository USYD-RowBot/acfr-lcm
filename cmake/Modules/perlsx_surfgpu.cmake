macro (perlsx_surfgpu)
  perlsx_find_library (gpusurf "

You must build and *manually* install the libgpusurf.so library provided from http://asrl.utias.utoronto.ca/code/gpusurf

For help, ask Paul O. or wait until he writes a README.

" ${ARGN})
  if (GPUSURF_FOUND)
    add_definitions (-DGPUSURF_FOUND)
    set (PERLSX_SURFGPU ${GPUSURF_LIBRARIES})
  endif ()
endmacro ()
