# SOURCE
set (SRC
  main.cpp                                  
  pose_data_sources.cpp             pose_data_sources.h
  renderers.h
  renderer_compass.cpp
  renderer_dvl_beams.cpp
  renderer_sat_map.cpp
  renderer_image.cpp
  renderer_isam_graph.cpp
  renderer_lcmgl.cpp
  renderer_planar_laser.cpp
  renderer_planar_target.cpp
  renderer_pvn_map.cpp
  renderer_range_circles.cpp
  renderer_robot_pose.cpp           renderer_robot_pose.h
  renderer_status_text.cpp
  renderer_vectormap_mission.cpp
  renderer_velodyne.cpp
  renderer_hauv.cpp                 
  renderer_vanctrl.cpp
  renderer_hauv_pose_attach.cpp
  renderer_camctrl.cpp
  renderer_plot.cpp
  renderer_util.cpp                 renderer_util.h
  )

# EXTERNAL DEPS
perlsx_bot2 ()
perlsx_curl ()
perlsx_glib ()
perlsx_gtk2 ()
perlsx_m ()
perlsx_opencv ()
perlsx_opengl ()
perlsx_glut ()

# BUILD EXE
add_definitions ("${PERLS_ACTIVE_LIB_DEFINES}")
perls_add_executable (viewer ${SRC})
perls_target_link_libraries (viewer ${PERLS_LIBRARIES}
  ${PERLSX_BOT2}
  ${PERLSX_CURL}
  ${PERLSX_GLIB}
  ${PERLSX_GTK2}
  ${PERLSX_M}
  ${PERLSX_OPENCV}
  ${PERLSX_OPENGL}
  ${PERLSX_GLUT}
  )
