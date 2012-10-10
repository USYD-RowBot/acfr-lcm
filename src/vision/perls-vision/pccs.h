#ifndef __PERLS_VISION_PCCS_H__
#define __PERLS_VISION_PCCS_H__

#include <stdbool.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-lcmtypes/perllcm_pose3d_t.h"
#include "perls-lcmtypes/perllcm_van_feature_t.h"
#include "perls-lcmtypes/perllcm_van_plot_debug_t.h"

#include "perls-math/gsl_util_index.h"

/**
 * @defgroup PerlsVisionPccs pccs
 * @brief find putative correspondence using pose constraint and similarity score
 * @ingroup PerlsVision
 * @include: perls-vision/pccs.h
 * @author Ayoung Kim
 * 
 * @{
 */

#define VIS_PCCS_SIMSCORE_MIN 0
#define VIS_PCCS_SIMSCORE_MAX 1


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief This is the main function in pccs and others are wrapper functions. To use this without wrapper function, user needs to pre-allocate sel1 and sel2 with n1 and n2 accordingly. Using wrapper functions is recommended.
 * @see vis_pccs_corrset_feat_alloc wrapper via feature_t
 * @see vis_pccs_corrset_gsl_alloc wrapper via gsl
 * 
 * @param uv1               input feature points in image1 (2 x n1 matrix). Similarly, uv2 for image2 (2 x n2 maxtrix).
 * @param z1                scene depth prior for feature points in image1 (n1 x 1 vector). z2 for image 2 (n2 x 1 vector).
 * @param cov_z1            scene depth prior uncertainty. See pccs.c for option USE_FIXED_COV_Z.
 * @param K                 calibration matrix.
 * @param X_c2c1            relative pose 21.
 * @param P_c2c1            related covariance to the relative pose.
 * @param key1                feature structure containing keys. Need to be provided when is called from vis_pccs_corrset_gsl. (Either key of f should be provided. Do not need both.)
 * @param f1              feature descriptor for uv1. key2 for uv2. Need to be provided when is called from vis_pccs_corrset_feat or vis_pccs_corrset_feat_alloc. (Either key of f should be provided. Do not need both.)
 * @param simAB_thres       Similarity score threshold. pccs will accept it as correspondences if \f$simA > (simAB_{thres})*(simAB_{thres})*(simB)\f$.
 * @param chiSquare2dof     search bound. if zero, we will set this in pccs.
 * @param simscore_minmax   depending on feature type, we choose max score (1, e.g. harris) or min score (0, e.g. sift)
 * @param sel1 (and sel2) selected indeces (returned).
 * @param pd populated plot_debug_t if needed (NULL = no plot) (optionally returned).
 *
 * @return number of putative corr. N and -1 when error.
 */
int 
vis_pccs_corrset_core (const gsl_matrix *uv1, const gsl_matrix *uv2,                        /* input matrices uv1 (2 x n1) and uv2 (2 x n2) */
                       const gsl_vector *z1, const gsl_vector *z2,                          /* input depth prior in gsl, z1 (n1 x 1) and z2 (n2 x 1)  */
                       const gsl_vector *cov_z1, const gsl_vector *cov_z2,                  /* input depth prior uncertainty in gsl */
                       const gsl_matrix *K,                                                 /* calibration matrix K */
                       const gsl_vector *X_c2c1,                                            /* relative pose 21 */
                       const gsl_matrix *P_c2c1,                                            /* related covariance to the relative pose */
                       const gsl_matrix_float *key1, const gsl_matrix_float *key2,          /* feature key1 and key2 from image 1 and 2 */
                       const perllcm_van_feature_t *f1, const perllcm_van_feature_t *f2,    /* feature structure containing keys */
                       gslu_index *sel1, gslu_index *sel2,                                  /* return sel: no allocation, therefore size of n1 and n2 */
                       double simAB_thres,                                                  /* will accept sel if simA > (simAB_thres)*(simAB_thres)*(simB) */
                       double chiSquare2dof,                                                /* search bound if zero, we will set this again */
                       bool simscore_minmax,                                                /* depending on feature type, we choose max score (1, e.g. harris) or min score (0, e.g. sift) */
                       perllcm_van_plot_debug_t *pd);                                       /* return populated plot_debug_t if needed (NULL = no plot) */


/**
 * @brief A wrapper of vis_pccs_corrset_core when input format in lcm feature_t. 
 * User needs to pre-allocate sel1 and sel2 with n1 and n2 accordingly in advance.
 * This function is usually called inside of vis_pccs_corrset_feat_alloc where sel allocation happens.
 *
 * @param f1              feature_t containing key and uv.
 * @param p21             relative pose 21, containing mu and sigma
 * @param K               calibration matrix K
 * @param simAB_thres       Similarity score threshold. pccs will accept it as correspondences if \f$simA > (simAB_{thres})*(simAB_{thres})*(simB)\f$.
 * @param chiSquare2dof   search bound. if zero, we will set this in pccs.
 *
 * @param sel1 (and sel2) selected indeces (returned).
 * @param pd populated plot_debug_t if needed (NULL = no plot) (optionally returned).
 *
 * @return number of putative corr. N and -1 when error.
 * 
 * @see vis_pccs_corrset_core
 */
int
vis_pccs_corrset_feat (const perllcm_van_feature_t *f1, const perllcm_van_feature_t *f2,    /* feature_t of image 1 and 2 */
                       const perllcm_pose3d_t p21,                                          /* relative pose 21, containing mu and sigma */
                       const gsl_matrix *K,                                                 /* calibration matrix K */
                       double simAB_thres,                                                  /* will accept sel if simA > (simAB_thres)*(simAB_thres)*(simB) */
                       double chiSquare2dof,                                                /* search bound if zero, we will set this again */
                       gslu_index *sel1, gslu_index *sel2,                                  /* return sel: no allocation, therefore size of n1 and n2 */
                       perllcm_van_plot_debug_t *pd);                                       /* return populated plot_debug_t if needed (NULL = no plot) */


/**
 * @brief A wrapper of vis_pccs_corrset_core when input format in lcm feature_t. 
 * This allocate memory for inliers indeces sel1 and sel2, and calls vis_pccs_corrset_feat internally.
 *
 * @param f1              feature_t containing key and uv.
 * @param p21             relative pose 21, containing mu and sigma
 * @param K               calibration matrix K
 * @param simAB_thres       Similarity score threshold. pccs will accept it as correspondences if \f$simA > (simAB_{thres})*(simAB_{thres})*(simB)\f$.
 * @param chiSquare2dof   search bound. if zero, we will set this in pccs.
 *
 * @param sel1 (and sel2) selected indeces (returned).
 * @param pd populated plot_debug_t if needed (NULL = no plot) (optionally returned).
 *
 * @return number of putative corr. N and -1 when error.
 * 
 * @see vis_pccs_corrset_core
 */
int 
vis_pccs_corrset_feat_alloc (const perllcm_van_feature_t *f1, const perllcm_van_feature_t *f2,   /* feature_t of image 1 and 2 */
                             const perllcm_pose3d_t p21,                                         /* relative pose 12, containing mu and sigma */
                             const gsl_matrix *K_gsl,                                            /* calibration matrix K */
                             double simAB_thres,                                                 /* will accept sel if simA > (simAB_thres)*(simAB_thres)*(simB) */
                             double chiSquare2dof,                                               /* search bound if zero, we will set this again */
                             gslu_index **sel1, gslu_index **sel2,                               /* return sel: allocate during function call with number of pairs found */
                             perllcm_van_plot_debug_t *pd);                                      /* return populated plot_debug_t if needed (NULL = no plot) */

/**
 * @brief A wrapper of vis_pccs_corrset_core when input format is in gsl. 
 * This allocate memory for inliers indeces sel1 and sel2, and calls vis_pccs_corrset_core internally.
 *
 * @param uv1               input feature points in image1 (2 x n1) maxtrix. uv2 for image2 (2 x n2) maxtrix
 * @param z1                scene depth prior for feature points in image1 (n1 x 1) vector. z2 for image 2 (n2 x 1) vector
 * @param cov_z1            scene depth prior uncertainty, See pccs.c for option USE_FIXED_COV_Z
 * @param K                 calibration matrix K
 * @param X_c2c1            relative pose 21
 * @param P_c2c1            related covariance to the relative pose
 * @param key1              feature descriptor for uv1. key2 for uv2. (Either key of f should be provided. Do not need both.)
 * @param simAB_thres       Similarity score threshold. pccs will accept it as correspondences if \f$simA > (simAB_{thres})*(simAB_{thres})*(simB)\f$.
 * @param simscore_minmax   depending on feature type, we choose max score (1, e.g. harris) or min score (0, e.g. sift)
 * @param sel1 (and sel2) selected indeces (returned).
 *
 * @return number of putative corr. N and -1 when error.
 * 
 * @see vis_pccs_corrset_core
 *
 */
int 
vis_pccs_corrset_gsl_alloc (const gsl_matrix *uv1, const gsl_matrix *uv2,               /* input matrices uv1 (2 x n1) and uv2 (2 x n2)*/           
                            const gsl_vector *z1, const gsl_vector *z2,                 /* input depth prior in gsl, z1 (n1 x 1) and z2 (n2 x 1)  */
                            const gsl_vector *cov_z1, const gsl_vector *cov_z2,         /* input depth prior uncertainty in gsl */
                            const gsl_matrix *K,                                        /* calibration matrix K */
                            const gsl_vector *X_c2c1,                                   /* relative pose 21 */
                            const gsl_matrix *P_c2c1,                                   /* related covariance to the relative pose */
                            const gsl_matrix_float *key1, const gsl_matrix_float *key2, /* feature key1 and key2 from image 1 and 2 */
                            gslu_index **sel1, gslu_index **sel2,                       /* return sel indeces: allocate during function call with number of pairs found */
                            double simAB_thres,                                         /* will accept sel if simA > (simAB_thres)*(simAB_thres)*(simB) */
                            bool simscore_minmax);                                      /* depending on feature type, we choose max score (harris) or min score (sift) */

/**
 * @brief pre calculate parameters needed in point transfer in case of using ukf.
 *  For 6 dof pose + uv + scene depth prior (z), we have
 *  \f$L = 6 + 2 + 1  = 9\f$ and 
 *  \f$r = 2*L +1     = 19\f$ (total number of sigma points).
 *
 * @param X_c2c1            relative pose 21
 * @param P_c2c1            related covariance to the relative pose
 * @param K                 calibration matrix K
 * @param invK              pre-calculated inverted K
 * @param cov_z             uncertainty related to the scene depth
 * @param X_vectors         simga points to be returned. (Lxr = 9x19) matrix.
 * @param weights           weihts to be returned. 19 weights
 * @param H_pre_vectors     Homographies to be returned. (9x19) matrix. Each column represent 3x3 Homography matrix.
 *
 * @return 1 when successful, 0 with error.
 */
int 
vis_pccs_precalc_single_xfer_params_ukf (const gsl_vector *X_c2c1, const gsl_matrix *P_c2c1, 
                                         const gsl_matrix *K, const gsl_matrix *invK,
                                         const double cov_z,
                                         gsl_matrix *X_vectors, gsl_vector *weights, gsl_matrix *H_pre_vectors);

/**
 * @brief Given calibration matrix, navigation prior, depth uncertainty,
 *        transfer sigma points to get projected image points uv2p.
 *        For efficiency, we use precalculated X_vectors, weights, H_pre_vectors.
 *
 * @param K                 calibration matrix K.
 * @param X_c2c1            relative pose 21.
 * @param P_c2c1            related covariance to the relative pose.
 * @param uv1               input feature points in image1 (2 x n1) maxtrix. uv2 for image2 (2 x n2) maxtrix
 * @param z1                scene depth prior for feature points in image1 (n1 x 1) vector. z2 for image 2 (n2 x 1) vector
 * @param X_vectors         simga points from vis_pccs_precalc_single_xfer_params_ukf.
 * @param weights           weihts from vis_pccs_precalc_single_xfer_params_ukf.
 * @param H_pre_vectors     Homographies vis_pccs_precalc_single_xfer_params_ukf.
 * 
 * @param uv2p projection of image point uv1 onto image 2 = uv2p ( 2x1 vector [u2p, v2p]) (returned).
 * @param covuv2p Uncertainty related to this transformation = cov_uv2p (2x2 matrix) (returned).
 */
void 
vis_pccs_relview_single_ptxfer_ukf (const gsl_matrix *K, const gsl_vector *X_c2c1, const gsl_matrix *P_c2c1, const gsl_vector *uv1, const double z1, 
                                    const gsl_matrix *X_vectors, const gsl_vector *weights, gsl_matrix *H_pre_vectors,
                                    gsl_vector *uv2p, gsl_matrix *cov_uv2p);


/**
 * @brief Transfer a single pixel point uv to uvp. uvp = Hinf * uv + K t/z.
 *
 * @param X         6x1 pose [x,y,z,r,p,h].
 * @param uv        2x1 pixel location [u,v].
 * @param Z         scene depth.
 * @param K         3x3 camera calibration matrix.
 * @param h_inf     Infinite homography stacked in a column. Hinf = [h11 h12 h13 h21 h22 h23 h31 h32 h33].
 *
 * @param uvp       2x1 projected pixel location in 2nd image (returned).
 */
void 
vis_pccs_single_ptxfer (const gsl_vector *X,     // [6 x 1] pose [x,y,z,r,p,h]
                        const gsl_vector *uv,    // [2 x 1] pixel location [u,v]
                        const double z,          // scene depth
                        const gsl_matrix *K,     // [3 x 3] camera calibration matrix
                        const gsl_vector *h_inf, // h_inf = [h11 h12 h13 h21 h22 h23 h31 h32 h33]
                        gsl_vector *uvp);        // [2 x 1] projected pixel location in 2nd image

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif //__PERLS_VISION_PCCS_H__
