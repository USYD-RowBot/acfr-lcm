# gslu_demo
perls_add_executable (math-gslu gslu_demo.c)
perls_target_link_libraries (math-gslu perls-common perls-math)
install (FILES gslu_demo.m
  DESTINATION ${EXAMPLE_SHARE_PATH}
)

# fasttrig_demo
perls_add_executable (math-fasttrig fasttrig_demo.c)
perls_target_link_libraries (math-fasttrig perls-common perls-math)

# homogenous_demo
perls_add_executable (math-homogenous homogenous_demo.c)
perls_target_link_libraries (math-homogenous perls-math)

# plane_demo
perls_add_executable (math-plane plane_demo.c)
perls_target_link_libraries (math-plane perls-math)
set (plane_demo_dummy  plane_demo.m plane_estim_RANSAC.m plane_demo_xyz_real.txt plane_demo_xyz_synthetic.txt)
install (FILES ${plane_demo_dummy}
  DESTINATION ${EXAMPLE_SHARE_PATH}
)

# so3_demo
perls_add_executable (math-so3 so3_demo.c)
perls_target_link_libraries (math-so3 perls-math)

# ssc_demo
perls_add_executable (math-ssc ssc_demo.c)
perls_target_link_libraries (math-ssc perls-common perls-math)
