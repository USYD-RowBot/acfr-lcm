# externals
perlsx_lcm ()
perlsx_opencv ()

perls_add_executable (botimg-publish main.c)
perls_target_link_libraries (botimg-publish perls-vision perls-lcmtypes
  ${PERLSX_LCM}
  ${PERLSX_OPENCV}
)
