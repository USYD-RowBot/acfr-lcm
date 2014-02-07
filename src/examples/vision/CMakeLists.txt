# vis_cvu_demo
perls_add_executable (vis-cvu cvu_demo.c)
perls_target_link_libraries (vis-cvu perls-math perls-vision)

# triangulate
perls_add_executable (vis-triangulate test_triangulate.c)
perls_target_link_libraries (vis-triangulate perls-math perls-vision)
install (FILES test_triangulate.m
  DESTINATION ${EXAMPLE_SHARE_PATH}
)

# botlcm_listener
perls_add_executable (vis-botimg-listener botimage_listener.c)
perls_target_link_libraries (vis-botimg-listener perls-lcmtypes perls-vision)

# twoview bundle adjustment test
perls_add_executable (vis-ba test_ba.c)
perls_target_link_libraries (vis-ba perls-math perls-vision)
set (test_ba_dummy test_ba.m test_ba_buildpts.m test_ba_H.m)
install (FILES ${test_ba_dummy}
  DESTINATION ${EXAMPLE_SHARE_PATH}
)

# putative correspondence search test
perls_add_executable (vis-corr test_corr.c)
perls_target_link_libraries (vis-corr perls-math perls-vision)
file(GLOB filelist "_test_corr_files/*.txt" "_test_corr_files/*.jpg")
foreach (file ${filelist})
  install (FILES ${file}
  DESTINATION ${EXAMPLE_SHARE_PATH}/_test_corr_files
  )
endforeach()

# gnuplot tester
perls_add_executable (vis-gnuplot test_gnuplot.c)
perls_target_link_libraries (vis-gnuplot perls-math perls-vision)

# twoview bundle adjustment test
perls_add_executable (vis-relorient test_relorient.c)
perls_target_link_libraries (vis-relorient perls-math perls-vision)

# siftgpu tester
if (${BUILD_VISION_siftgpu})
  perls_add_executable (vis-siftgpu test_siftgpu.cpp)
  target_link_libraries (vis-siftgpu perls-math perls-vision)
  set_target_properties (vis-siftgpu PROPERTIES 
    COMPILE_FLAGS "-Wno-write-strings -O3 -Wall"
    )
endif ()

# surfgpu tester
if (${BUILD_VISION_surfgpu})
  perls_add_executable (vis-surfgpu test_surfgpu.c)
  target_link_libraries (vis-surfgpu perls-math perls-common perls-vision)
endif ()

# harris/zernike feature tester
perls_add_executable (vis-zernike test_zernike.c)
perls_target_link_libraries (vis-zernike perls-math perls-vision)
install (FILES test_zernike.m
  DESTINATION ${EXAMPLE_SHARE_PATH}
)

