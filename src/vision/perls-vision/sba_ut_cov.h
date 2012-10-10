#ifndef __PERLS_VISION_SBA_UT_COV_H__
#define __PERLS_VISION_SBA_UT_COV_H__

#include <glib.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-lcmtypes/perllcm_van_feature_t.h"

/**
 * @defgroup PerlsVisionSbaUt SBA UT covariance estimate
 * @brief Estimate the relative pose of two-view SBA with unscented transform
 * @ingroup PerlsVision
 * 
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Estimate the relative pose covariance using the unscented transform method
 *
 * @param params 
 *
 * @note The first 5 entries of params assumed to be 0 since camera 1's frame acts as the origin
 *
 */
void
vis_sba_ut_cov_2v_H (const gsl_matrix *sigmaCalib,
                     const gsl_vector *params, const gsl_matrix *H, const gsl_matrix *u1,
                     const gsl_matrix *u2, const gsl_matrix *K,
                     const gsl_vector *distCoeffs, const perllcm_van_feature_t *fi, 
                     const gsl_vector *featscalei, const gsl_vector *featscalej,
                     GMutex *sbaMutex, gsl_matrix *rel_pose_cov);

static inline gsl_matrix *
vis_sba_ut_cov_2v_H_alloc (const gsl_matrix *sigmaCalib,
                           const gsl_vector *params, const gsl_matrix *H, const gsl_matrix *u1,
                           const gsl_matrix *u2, const gsl_matrix *K,
                           const gsl_vector *distCoeffs, const perllcm_van_feature_t *fi, 
                           const gsl_vector *featscalei, const gsl_vector *featscalej,
                           GMutex *sbaMutex)
{
    gsl_matrix *rel_pose_cov = gsl_matrix_calloc (5, 5);
    vis_sba_ut_cov_2v_H (sigmaCalib, params, H, u1, u2, K, distCoeffs, fi, featscalei, featscalej, sbaMutex, rel_pose_cov);
    return rel_pose_cov;
}

/**
 * Estimate the relative pose covariance using the unscented transform method
 *
 * @param params 10 + 3*N vector where N is the number of points in u1 and u2
 *
 * @note The first 5 entries of params assumed to be 0 since camera 1's frame acts as the origin
 *
 */
void
vis_sba_ut_cov_2v_Rae (const gsl_matrix *sigmaCalib,
                       const gsl_vector *params, const gsl_matrix *u1,
                       const gsl_matrix *u2, const gsl_matrix *K,
                       const gsl_vector *distCoeffs, 
                       const gsl_vector *featscalei, const gsl_vector *featscalej,
                       GMutex *sbaMutex, gsl_matrix *rel_pose_cov);

/**
 * allocation version of vis_sba_ut_cov_2v
 */
static inline gsl_matrix *
vis_sba_ut_cov_2v_Rae_alloc (const gsl_matrix *sigmaCalib,
                             const gsl_vector *params, const gsl_matrix *u1,
                             const gsl_matrix *u2, const gsl_matrix *K,
                             const gsl_vector *distCoeffs, 
                             const gsl_vector *featscalei, const gsl_vector *featscalej,
                             GMutex *sbaMutex)
{
    gsl_matrix *rel_pose_cov = gsl_matrix_calloc (5, 5);
    vis_sba_ut_cov_2v_Rae (sigmaCalib, params, u1, u2, K, distCoeffs, featscalei, featscalej, sbaMutex, rel_pose_cov);
    return rel_pose_cov;
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif // __PERLS_VISION_SBA_UT_COV_H__
