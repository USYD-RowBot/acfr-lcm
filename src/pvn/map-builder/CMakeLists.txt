# EXTERNAL DEPS
perlsx_gsl ()
perlsx_las ()
perlsx_eigen3 ()
perlsx_opencv ()
perlsx_pcl ()
perlsx_isam ()
perlsx_suitesparse ()

# BUILD EXE
perls_add_executable (map-builder map_builder.h
                                  main.cpp 
                                  laser_map.cpp
                                  seed_map.cpp
                                  eview_map.cpp
                                  eview_process.cpp
                                  build_vocab.cpp
                                  sparsify_map.cpp
                                  build_chow_liu_tree.cpp)

perls_target_link_libraries (map-builder perls-lcmtypes perls-common perls-math perls-vision perls-sensors perls-pvn
  ${PERLSX_GSL}
  ${PERLSX_LAS}
  ${PERLSX_EIGEN3}
  ${PERLSX_OPENCV}
  ${PERLSX_PCL}
  ${PERLSX_ISAM}
  ${PERLSX_SUITESPARSE}
  )
