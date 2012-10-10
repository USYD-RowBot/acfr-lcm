# EXTERNAL DEPS
perlsx_gsl ()
perlsx_las ()
perlsx_eigen3 ()
perlsx_opencv ()
perlsx_pcl ()

# BUILD EXE
perls_add_executable (tmpcor main.cpp tmpcor.h
                             webcam_process.cpp
                             preprocess.cpp)

perls_target_link_libraries (tmpcor perls-lcmtypes perls-common perls-math perls-vision perls-sensors perls-pvn
  ${PERLSX_GSL}
  ${PERLSX_LAS}
  ${PERLSX_EIGEN3}
  ${PERLSX_OPENCV}
  ${PERLSX_PCL}
  )