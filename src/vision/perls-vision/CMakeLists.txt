# SOURCE FILES
set (SRC
  botimage.c         botimage.h
  camera.c           camera.h
  calib.c            calib.h
  clahs.c            clahs.h
  distortion.c       distortion.h
  epipolar.c         epipolar.h
  feature.c          feature.h
  homography.c       homography.h
  ladybug3_util.c    ladybug3_util.h
  modelfit.c         modelfit.h
  opencv_util.c      opencv_util.h
  highgui_util.c     highgui_util.h
  pccs.c             pccs.h
  plot.h             plot.c
  sba.c              sba.h
  featuregpu.c       featuregpu.h
  triangulate.c      triangulate.h
  twoview.c          twoview.h
  zernike.c          zernike.h
  sba_haralick_cov.c sba_haralick_cov.h
  sba_ut_cov.c       sba_ut_cov.h
  )

#NOTE: WE SHOULD CALL OPENCV AFTER
#==14034==    at 0x82C03E8: dscal_ (in /usr/lib/libcxcore.so.4.0.0)
#==14034==    by 0x82BF4FC: dpotf2_ (in /usr/lib/libcxcore.so.4.0.0)
#==14034==    by 0x949AC21: sba_mat_cholinv (in /usr/local/lib/libsba.so)
#==14034==    by 0x948BA7C: sba_motstr_levmar_x (in /usr/local/lib/libsba.so)
# The above is the valgrid report when opencv comes before sba or f2c.
# Currently figuring out this with sba developers. ayoung (2012.1.6)

#
# EXTERNAL DEPS
perlsx_bot2 ()
perlsx_glib ()
perlsx_gsl ()
perlsx_m ()
perlsx_pvapi ()
perlsx_sba ()
perlsx_tiff ()
perlsx_lapack ()
perlsx_blas ()
perlsx_f2c ()
perlsx_opencv () # WE SHOULD CALL OPENCV AFTER SBA

# BUILD LIBRARY
perls_add_library (perls-vision ${SRC})
perls_target_link_libraries (perls-vision perls-lcmtypes perls-common perls-math
  ${PERLSX_BOT2}
  ${PERLSX_GLIB}
  ${PERLSX_GSL}
  ${PERLSX_M}
  ${PERLSX_PVAPI}
  ${PERLSX_SBA}
  ${PERLSX_TIFF}
  ${PERLSX_LAPACK}
  ${PERLSX_BLAS}
  ${PERLSX_F2C}
  ${PERLSX_OPENCV} # WE SHOULD CALL OPENCV AFTER SBA
  )
