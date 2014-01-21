#ifndef __PERLS_VISION_MODELFIT_H__
#define __PERLS_VISION_MODELFIT_H__

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/gsl_util_index.h"
#include "perls-math/gsl_util_linalg.h"
/**
 * @defgroup PerlsVisionModelfit modelfit
 * @brief Fit model and compute GIC score
 * @ingroup PerlsVision
 * @include: perls-vision/modelfit.h
 * @author Ayoung Kim
 * 
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum vis_modelfit_method
{
    VIS_MODELFIT_OCV_RANSAC=1, 
    VIS_MODELFIT_OCV_LMEDS, 
    VIS_MODELFIT_RANSAC
} vis_modelfit_method_t;

typedef enum vis_modelfit_model
{
    VIS_MODELFIT_H=1, 
    VIS_MODELFIT_F, 
    VIS_MODELFIT_E
} vis_modelfit_model_t;

/* wrapper fucntion to be called from twoview_thread
 * calles vis_modelfit_F and vis_modelfit_gic internally
 * and returns gic and inliers
 */
double
vis_modelfit_inliers_gic_3D (gsl_matrix *uvi_sel, gsl_matrix *uvj_sel, 
                             gslu_index **sel_f, gsl_matrix **uvi_f, gsl_matrix **uvj_f,
                             gsl_matrix *F, vis_modelfit_method_t method);


/* wrapper fucntion to be called from twoview_thread
 * calles vis_modelfit_H and vis_modelfit_gic internally
 * and returns gic and inliers
 */
double
vis_modelfit_inliers_gic_2D (gsl_matrix *uvi_sel, gsl_matrix *uvj_sel, 
                             gslu_index **sel_h, gsl_matrix **uvi_h, gsl_matrix **uvj_h,
                             gsl_matrix *H, vis_modelfit_method_t method);

/* given uv1 and uv2, find the fundamental matrix F
 * returns number of inliers, 0 on error
 */
double
vis_modelfit_F_ocv (const gsl_matrix *uv1, const gsl_matrix *uv2,   // uv1, uv2 [2 x N] array of pixel points in image 1 and 2
                    gsl_matrix *F,                                  // resulting [3 x 3] Fundamental matrix
                    gslu_index **sel,                               // inliers to be returned
                    vis_modelfit_method_t method);                  // RANSAC, LMEDS, or ALL

/* given uv1 and uv2, find homography H
 * returns number of inliers, 0 on error
 */
double
vis_modelfit_H_ocv (const gsl_matrix *uv1, const gsl_matrix *uv2,   // uv1, uv2 [2 x N] array of pixel points in image 1 and 2
                    gsl_matrix *H,                                  // resulting [3 x 3] Homography
                    gslu_index **sel,                               // inliers to be returned
                    vis_modelfit_method_t method);                  // RANSAC, LMEDS


double
vis_modelfit_E_ransac (const gsl_matrix *xy1, const gsl_matrix *xy2, 
                       gsl_matrix *E, gslu_index **sel, gsl_vector **error);

double
vis_modelfit_H_ransac (const gsl_matrix *xy1, const gsl_matrix *xy2, 
                       gsl_matrix *H, gslu_index **sel, gsl_vector **error);

/* computes Torr's GIC score given:
 * reprojection error vector of model, 
 * dof of underlying structure {2 or 3}
 * dof of model
 * default value for lambda1 = 2.0 and lambda2 = 4.0
 */
double
vis_modelfit_gic (const gsl_matrix* model, vis_modelfit_model_t model_type,
                  const gsl_matrix* uv1, const gsl_matrix* uv2,  
                  double lambda1, double lambda2);

/* returns the model's reprojection error
 * uv1, uv2 [2 x N] inlier pixel coords
 * model    [3 x 3] model such as H, E, or F
 * err      [1 x N] vector of reprojection error
 * model_type = {VIS_MODELFIT_H, VIS_MODELFIT_F, VIS_MODELFIT_E}
                 if it is F or E, calculate point to line distance
                          H, calculate point to point di
 */
gsl_vector *
vis_modelfit_reprojection_error (const gsl_matrix *uv1, const gsl_matrix *uv2, 
                                 const gsl_matrix *model, vis_modelfit_model_t model_type);


gslu_linalg_SV *
_SV_decomp_full_alloc_fortran (const gsl_matrix *A);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif //__PERLS_VISION_MODELFIT_H__
