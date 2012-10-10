# EXTERNAL DEPS
perlsx_boost ()
perlsx_eigen3 ()
perlsx_gsl ()

include_directories (eigen_utils/)

perls_add_folder (nav "Build owtt-nav group?" ON)
perls_add_folder (platforms "Build owtt-nav platforms?" ON)
perls_add_folder (test "Build owtt-nav test?" OFF)
  
perls_python_install_script (owtt-publish-naive-osm publish_naive_osm_pose.py)

