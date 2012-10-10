#ifndef __PERLS_VISION_TWOVIEW_H__
#define __PERLS_VISION_TWOVIEW_H__

#include <stdint.h>
#include <stdbool.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-common/cache.h"
#include "perls-math/gsl_util_index.h"

#include "perls-lcmtypes/perllcm_pose3d_t.h"
#include "perls-lcmtypes/perllcm_van_feature_collection_t.h"
#include "perls-lcmtypes/perllcm_van_options_t.h"
#include "perls-lcmtypes/perllcm_van_plot_debug_t.h"
#include "perls-lcmtypes/perllcm_van_vlink_t.h"
#include "perls-lcmtypes/perllcm_van_plink_t.h"
#include "perls-lcmtypes/se_publish_link_t.h"
#include "perls-lcmtypes/perllcm_isam_vlink_t.h"

#define VIS_TV_PROC_PCCS 1
#define VIS_TV_PROC_FIN 2
#define VIS_TV_PROC_HIN 3

#ifdef __cplusplus
extern "C" {
#endif

perllcm_van_vlink_t *
vis_twoview_vlink (int64_t utime_i, int64_t utime_j, int32_t type, int32_t errmsg,
                   gslu_index *seli, gslu_index *selj, gslu_index *sel_h, gslu_index *sel_f,
                   gsl_vector *z_gsl, gsl_matrix *R_gsl);

se_publish_link_t *
vis_twoview_se_vlink (int64_t utime_i, int64_t utime_j,
                      int32_t type, int32_t errmsg, int64_t link_id,
                      gsl_vector *z_gsl, gsl_matrix *R_gsl);

perllcm_isam_vlink_t *
vis_twoview_isam_vlink (int64_t utime_i, int64_t utime_j,
                        perllcm_van_plink_t *plink,
                        int32_t type, int32_t errmsg, int64_t link_id,
                        gsl_vector *z_gsl, gsl_matrix *R_gsl);


gsl_vector *
vis_tv_use_navprior (const gsl_matrix *K, const gsl_matrix *uv1, const gsl_matrix *uv2, 
                     const gsl_vector *x21, const gsl_matrix *p21, const gsl_vector *x21_prior,
                     int nsamples, bool verbose);

/* Once 5 dof estimated relative pose is found from two images
 * check with navigator prior to verify consistency with navigatation
 *
 * nav_p21 [6x1], nav_cov21 [6x6], cam_p21 [5x1], cam_cov21 [5x5]
 *
 * when passed, return VANLCM_VLINK_T_MSG_NO_ERROR (0)
 * additionally min_mdist found will be returned.
 */
int32_t 
vis_tv_mdist_check_error (const gsl_vector *nav_p21, const gsl_matrix *nav_cov21,
                          const gsl_vector *cam_p21, const gsl_matrix *cam_cov21,
                          const double thresh, double *min_mdist);

/* check min point criteria
 * return error message if found
 * when passed, return VANLCM_VLINK_T_MSG_NO_ERROR (0)
 */
int32_t
vis_tv_minpt_check_error (size_t n, size_t minpt, size_t proc_name);

/* print out BA result
 */
void
vis_tv_print_sba (const gsl_vector *rel_pose21, const gsl_matrix *rel_pose_cov21, int64_t dt);

/* print summary motion 
 */
void
vis_tv_print_motion (const gsl_vector *nav21, const gsl_matrix *navS21,
                     const gsl_vector *horn21,
                     const gsl_vector *cam21, const gsl_matrix *camS21);

/*  when debugging mode is on (plot_option_t)
 *  populate necessary data (plot_debug_t) except search bound
 *  before publishing into network
 *  
 *  NOTE: we prepare plot debug for search bound inside pccs.c
 *  see _vis_pccs_prepare_plot_debug ()
 */
void
vis_tv_prepare_plot_debug (perllcm_van_plot_debug_t *pd, perllcm_van_options_t van_opt, int32_t type, int32_t errmsg,
                           gslu_index *selic, gslu_index *seljc, gslu_index *sel_h, gslu_index *sel_f, int32_t model,
                           gsl_matrix *K_or_H,
                           gsl_vector *z_gsl, gsl_matrix *R_gsl, gsl_matrix *X1, perllcm_pose3d_t p21,
                           size_t nlink_remaining);

perllcm_van_plot_debug_t *
vis_tv_init_plot_debug (perllcm_van_options_t van_opt, 
                        const perllcm_van_feature_collection_t *fci, const perllcm_van_feature_collection_t *fcj, 
                        const perllcm_van_calib_t *calib);

#ifdef __cplusplus
}
#endif

#endif //__PERLS_VISION_TWOVIEW_H__
