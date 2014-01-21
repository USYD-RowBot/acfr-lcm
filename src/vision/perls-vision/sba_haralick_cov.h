#ifndef __PERLS_VISION_SBA_HARALICK_COV_H__
#define __PERLS_VISION_SBA_HARALICK_COV_H__

#include <glib.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/**
 * @defgroup PerlsVisionSbaHaralick SBA Haralick covariance estimate
 * @brief Estimate the relative pose of two-view SBA with Haralick's equations
 * @ingroup PerlsVision
 * 
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Estimate the relative pose covariance using Haralick's method
 *
 * @param params 10 + 3*N vector where N is the number of corresponding features
 *
 * @note The first 5 entries of params assumed to be 0 since camera 1's frame acts as the origin
 *
 * This function computes the Haralick covariance estimate of \Theta for 2-view SBA:
 *
 * \f[ \Sigma_{\Theta} = \left(\mathrm{J}_{{\bf X}_c}^\top \Sigma_{{\bf
  X}_u}^{-1} \mathrm{J_{{\bf X}_c}}\right)^{-1} + \frac{1}{4} \underbrace{\left(\mathrm{J}_{{\bf
  X}_c}^\top \Sigma_{{\bf X}_u}^{-1} \mathrm{J_{{\bf X}_c}}\right)^{-1}
  \mathrm{A}^\top}_{2\mathrm{B}}\Sigma_{{\bf X}_c} \underbrace{\mathrm{A} \left(\mathrm{J}_{{\bf
  X}_c}^\top \Sigma_{{\bf X}_u}^{-1} \mathrm{J_{{\bf X}_c}}\right)^{-1}}_{2\mathrm{B}^\top} \f]
 *
 */
void
vis_sba_haralick_cov_2v_Rae (const gsl_matrix *sigmaCalib,
                             const gsl_vector *params, const gsl_matrix *u1,
                             const gsl_matrix *u2, const gsl_matrix *K,
                             const gsl_vector *distCoeffs,
                             const gsl_vector *featscalei, const gsl_vector *featscalej,
                             gsl_matrix *rel_pose_cov);

/**
 * allocation version of vis_sba_haralick_cov_2v_Rae
 */
static inline gsl_matrix *
vis_sba_haralick_cov_2v_Rae_alloc (const gsl_matrix *sigmaCalib,
                                   const gsl_vector *params, const gsl_matrix *u1,
                                   const gsl_matrix *u2, const gsl_matrix *K,
                                   const gsl_vector *distCoeffs,
                                   const gsl_vector *featscalei, const gsl_vector *featscalej)
{
    gsl_matrix *rel_pose_cov = gsl_matrix_calloc (5, 5);
    vis_sba_haralick_cov_2v_Rae (sigmaCalib, params, u1, u2, K, distCoeffs, featscalei, featscalej, rel_pose_cov);
    return rel_pose_cov;
}

/**
 * Estimate the relative pose covariance using Haralick's method (homography-based SBA)
 *
 * @param params 16 + 2*N vector where N is the number of corresponding features
 *
 * @note The first 8 entries of params assumed to be 0 since camera 1's frame acts as the origin
 *
 * This function computes the Haralick covariance estimate of \Theta for 2-view homography-based SBA:
 *
 * \f[ \Sigma_{\Theta} = \left(\mathrm{J}_{{\bf X}_c}^\top \Sigma_{{\bf
  X}_u}^{-1} \mathrm{J_{{\bf X}_c}}\right)^{-1} + \frac{1}{4} \underbrace{\left(\mathrm{J}_{{\bf
  X}_c}^\top \Sigma_{{\bf X}_u}^{-1} \mathrm{J_{{\bf X}_c}}\right)^{-1}
  \mathrm{A}^\top}_{2\mathrm{B}}\Sigma_{{\bf X}_c} \underbrace{\mathrm{A} \left(\mathrm{J}_{{\bf
  X}_c}^\top \Sigma_{{\bf X}_u}^{-1} \mathrm{J_{{\bf X}_c}}\right)^{-1}}_{2\mathrm{B}^\top} \f]
 *
 */
void
vis_sba_haralick_cov_2v_H (const gsl_matrix *sigmaCalib,
                           const gsl_vector *params, const gsl_matrix *u1,
                           const gsl_matrix *u2, const gsl_matrix *K,
                           const gsl_vector *distCoeffs,
                           const gsl_vector *featscalei, const gsl_vector *featscalej,
                           gsl_matrix *rel_pose_cov);

/**
   allocation version of vis_sba_haralick_cov_2v_h
*/
static inline gsl_matrix *
vis_sba_haralick_cov_2v_H_alloc (const gsl_matrix *sigmaCalib,
                                 const gsl_vector *params, const gsl_matrix *u1,
                                 const gsl_matrix *u2, const gsl_matrix *K,
                                 const gsl_vector *distCoeffs,
                                 const gsl_vector *featscalei, const gsl_vector *featscalej)
{
    gsl_matrix *rel_pose_cov = gsl_matrix_calloc (5, 5);
    vis_sba_haralick_cov_2v_H (sigmaCalib, params, u1, u2, K, distCoeffs, featscalei, featscalej, rel_pose_cov);
    return rel_pose_cov;
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif // __PERLS_VISION_SBA_HARALICK_COV_H__
