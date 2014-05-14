# SOURCE FILES
set (SRC
  main.cpp              
  interf_thread.cpp      interf_thread.h
  scan_match_thread.cpp  scan_match_thread.h
                         velodyne_scan_matcher.h
  vis_thread.cpp         vis_thread.h
)

# EXTERNAL DEPS
perlsx_ann ()
perlsx_boost ()
perlsx_pcl ()

# HACK will remove when we are no longer dependent on andrew segals gicp code
include_directories ("/usr/local/include/gicp")

#message ("TEST: ${PERLSX_PCL}")

# BUILD EXE
perls_add_executable (velodyne-scan-matcher ${SRC})
perls_target_link_libraries (velodyne-scan-matcher perls-lcmtypes perls-common perls-sensors
  ${PERLSX_BOOST} 
  ${PERLSX_GICP}
  ${PERLSX_PCL}
  ${PERLSX_ANN}
  /usr/local/lib/libgicp.a  #also a hack untill i remove andrew segals gicp code
  )
