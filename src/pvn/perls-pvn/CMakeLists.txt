# SOURCE FILES
set (SRC 
     pvn_util.cpp               pvn_util.h
     pvn_chowliutree.cpp        pvn_chowliutree.h
     pvn_eview.cpp              pvn_eview.h
     pvn_image_data.cpp         pvn_image_data.h
     combination.hpp
    )

# EXTERNAL DEPS
perlsx_gsl ()
perlsx_las ()
perlsx_eigen3 ()
perlsx_opencv ()
perlsx_pcl ()


# BUILD LIBRARY
perls_add_library (perls-pvn ${SRC})
perls_target_link_libraries (perls-pvn perls-lcmtypes perls-common perls-math perls-vision perls-sensors
                            ${PERLSX_GSL}
                            ${PERLSX_LAS}
                            ${PERLSX_EIGEN3}
                            ${PERLSX_OPENCV}
                            ${PERLSX_PCL})
