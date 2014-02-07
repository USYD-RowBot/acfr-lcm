# SOURCE FILES
set (SRC 
                           compare.h
  dm.c                     dm.h
  fasttrig.c               fasttrig.h
                           gsl_util.h
  gsl_util_array.c         gsl_util_array.h
  gsl_util_blas.c          gsl_util_blas.h
  gsl_util_boolean.c       gsl_util_boolean.h
  gsl_util_eigen.c         gsl_util_eigen.h
  gsl_util_index.c         gsl_util_index.h
  gsl_util_linalg.c        gsl_util_linalg.h
  gsl_util_math.c          gsl_util_math.h
  gsl_util_matrix.c        gsl_util_matrix.h
                           gsl_util_permutation.h
  gsl_util_rand.c          gsl_util_rand.h
  gsl_util_statistics.c    gsl_util_statistics.h
  gsl_util_vector.c        gsl_util_vector.h
  homogenous.c             homogenous.h
  plane.c                  plane.h
  ransac.c                 ransac.h
  rphcorr.c                rphcorr.h
                           so3.h
  ssc.c                    ssc.h
  unscented_transform.c    unscented_transform.h 
  )

# EXTERNAL DEPS
perlsx_gsl ()
perlsx_m ()

# BUILD LIBRARY
perls_add_library (perls-math ${SRC})
perls_target_link_libraries (perls-math perls-common
  ${PERLSX_GSL}
  ${PERLSX_M}
  )

