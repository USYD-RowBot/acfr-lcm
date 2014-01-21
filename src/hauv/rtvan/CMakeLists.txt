# SOURCE FILES
set (SRC
  main.c
  feature_thread.c      feature_thread.h
  hauv_thread.c         hauv_thread.h
  link_thread.c         link_thread.h
  plot_thread.c         plot_thread.h
  saliency_thread.c     saliency_thread.h
  secam_thread.c        secam_thread.h
  isam_interf_thread.c  isam_interf_thread.h
  shared_memory.c       shared_memory.h
  twoview_thread.c      twoview_thread.h
  van_util.c            van_util.h
)

# EXTERNAL DEPS
perlsx_glib ()
perlsx_gsl ()
perlsx_m ()
perlsx_opencv ()

# BUILD EXE
perls_add_executable (rtvan ${SRC})
perls_target_link_libraries (rtvan perls-lcmtypes perls-common perls-math perls-sensors perls-vision
  ${PERLSX_GLIB}
  ${PERLSX_GSL}
  ${PERLSX_M}
  #${PERLSX_OPENCV} # adding OPENCV causes seg fault in sba chol inv
)

perls_add_executable (rtvan-timer van_timer.cpp) 
perls_target_link_libraries (rtvan-timer perls-lcmtypes perls-common)
