# SOURCE FILES
set (SRC
              est.h
  est_core.c  est_core.h
  ekf.c       ekf.h
  ukf.c       ukf.h
  pf.c        pf.h
  # observation models
  est_omf/est_omf.h
  est_omf/omf_dstar_ssp1_z.c
  est_omf/omf_rdi_pd4_uvw.c
  est_omf/omf_kvh_dsp3000_c.c
  est_omf/omf_ms_gx1_rph.c
  est_omf/omf_ms_gx1_abc.c
  est_omf/omf_gpsd_xy.c
  est_omf/omf_gpsd3_xy.c
  est_omf/omf_osi_motor_count_u.c
  # process models
  est_pmf/est_pmf.h
  est_pmf/est_pmf.c
  est_pmf/pmf_const_vel_body.c
  est_pmf/pmf_differential_drive.c
  )

# EXTERNAL DEPS
perlsx_bot2 ()
perlsx_gsl ()
perlsx_m ()

# BUILD LIBRARY
perls_add_library (perls-est ${SRC})
perls_target_link_libraries (perls-est perls-lcmtypes perls-math
  ${PERLSX_BOT2}
  ${PERLSX_GSL}
  ${PERLSX_M}
  )

