#ifndef __PERLS_VISION_PLOT_H__
#define __PERLS_VISION_PLOT_H__

#include <stdbool.h>
#include <stdint.h>
#include <opencv/cv.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_van_feature_t.h"
#include "perls-lcmtypes/perllcm_van_plot_debug_t.h"
#include "perls-lcmtypes/perllcm_van_saliency_t.h"
#include "perls-lcmtypes/perllcm_van_scene_prior_t.h"

#include "perls-math/gsl_util_index.h"

#define VIS_PLOT_STACK_HORZ 0
#define VIS_PLOT_STACK_VERT 1

#define VIS_PLOT_IN    1
#define VIS_PLOT_OUT   2
#define VIS_PLOT_INOUT 3

// name of the plot windows
#define PLOT_WIN_SIFTGPU            "SIFTGPU"
#define PLOT_WIN_SURF               "SURF"
#define PLOT_WIN_HARRIS             "Harris"
#define PLOT_WIN_SCENEPRIOR         "SCENE PRIOR"
#define PLOT_WIN_PUTCORR            "PUTATIVE CORR"
#define PLOT_WIN_SEARCH_ELLIPSES1   "ellises1"
#define PLOT_WIN_SEARCH_ELLIPSES2   "ellises2"
#define PLOT_WIN_IN_OUT             "IN/OUT"
#define PLOT_WIN_IN                 "INLIERS"
#define PLOT_WIN_OUT                "OUTLIERS"
#define PLOT_WIN_SUMMARY            "SUMMARY"
#define PLOT_WIN_VERIFY             "MANUAL VERIFICATION"
#define PLOT_WIN_SALIENCY           "SALIENCY"

/*#define PLOT_WIN_POSE
#define PLOT_WIN_3DPTS
#define PLOT_WIN_POSE_3DPTS*/


#ifdef __cplusplus
extern "C" {
#endif

/*
 * Adds utime label to upper left corner of image
 */
void
vis_plot_add_utime (IplImage *image, 
                    int64_t utime, 
                    const CvPoint *_pt,      // set to NULL to use default
                    const CvFont *_font,     // set to NULL to use default
                    const CvScalar *_color); // set to NULL to use default

/*
 * Adds text label to upper left corner of image
 */
void
vis_plot_add_text (IplImage *image, 
                   const char *text,
                   const CvPoint *_pt,      // set to NULL to use default
                   const CvFont *_font,     // set to NULL to use default
                   const CvScalar *_color); // set to NULL to use default

/*
 * Plots a pair of images side by side or stacked vertically at desired scale
 */
void
vis_plot_pair (IplImage *img1, 
               IplImage *img2, 
               const char *named_window, 
               float scale, 
               int stack_type);

/*
 * Plots image feature points
 */
void
vis_plot_feature (const IplImage *image, 
                  const perllcm_van_feature_t *f, 
                  const char *named_window, 
                  float scale, 
                  int64_t utime); // image utime, set to 0 to not display it
/*
 * Plot saliency information with image
 */
void
vis_plot_saliency (const perllcm_van_saliency_t *sal,
                   const IplImage *image,
                   const char *named_window,
                   const float scale);

/*
 * Plots bathymetry scene depth prior
 */
void
vis_plot_scene_prior (const IplImage *image,
                      const perllcm_van_scene_prior_t *sp,
                      const char *named_window,
                      int64_t utime, const float scale); // image utime, set to 0 to not display it

/*
 * Plots point correspondences
 * sel1_vgsl, sel2_vgsl = vector containing inliers indeces
 * uv1_mgsl, uv2_mgsl = 2xN matrix containing both inliers and outliers
 * stack_type = {VIS_PLOT_STACK_HORZ, VIS_PLOT_STACK_VERT}
 * in_or_out = {VIS_PLOT_IN, VIS_PLOT_OUT, VIS_PLOT_INOUT}
 */
void 
vis_plot_correspondences (IplImage *img1, IplImage *img2, 
                          const gslu_index *sel1, const gslu_index *sel2, 
                          const gsl_matrix *uv1, const gsl_matrix *uv2, 
                          bool stack_type, int in_or_out, const char *named_window,
                          gslu_index *clr_idx_vec, const float scale);

/* plot search bound (ellipses) in pccs
 */
void 
vis_plot_pccs_ellipses (IplImage *img1, IplImage *img2, 
                        const gsl_matrix *uv1, const gsl_matrix *uv2p, const gsl_matrix *cov2p, const gsl_matrix *uv2,
                        const gsl_matrix *F, const double chiSquare2dof,
                        bool stack_type, const char *named_window, const float scale, int64_t dt);

// GNU PLOT
//----------------------------------------------------------------------------

/* plot coordinate frame
 * given frame center c and rotation matrix R
 * c: gsl_vector (3)
 * R: gsl_matrix (3,3)
 */
int
vis_plot_coordinate_frame (const gsl_vector *center, const gsl_matrix *R,
                           FILE *gp, int arrow_count, int z_axis_color);

/* 3d plot of relative pose
 * using gnuplot piping
 * internally calls 5 dof or 6 dof
 */
void
vis_plot_relpose (FILE *gp, const gsl_vector *x21, const double scale, const gsl_vector *nav);

/*  plot 3d points
 * using gnuplot piping
 */
void 
vis_plot_3dpts (FILE *gp, const gsl_matrix *X);

/*  plot both relative pose and the 3d points
 *  using gnuplot piping
 */
void 
vis_plot_relpose_3dpts (FILE *gp, const gsl_vector *x21, const gsl_matrix *X, const double scale, const gsl_vector *nav);

// PLOT_DEBUG_T
//----------------------------------------------------------------------------
/* wrapper for vis_plot_correspondences
 * with option putative corr.
 */
void
vis_plot_pccs_debugplot (const perllcm_van_plot_debug_t *pd, 
                         IplImage *imgi, IplImage *imgj,
                         const gslu_index *sel1, const gslu_index *sel2, 
                         const gsl_matrix *uv1, const gsl_matrix *uv2,
                         const float scale);

/* wrapper for vis_plot_correspondences
 * with option inliers
 */
void
vis_plot_in_and_outliers_debugplot (const perllcm_van_plot_debug_t *pd, 
                                    IplImage *imgi, IplImage *imgj,
                                    gsl_matrix *uvi_sel, gsl_matrix *uvj_sel,
                                    int in_or_out, const float scale);


int
vis_plot_manual_verification_debugplot (const perllcm_van_plot_debug_t *pd, 
                                        IplImage *imgi, IplImage *imgj,
                                        gsl_matrix *uvi_sel, gsl_matrix *uvj_sel, 
                                        const float scale);

/* wrapper for vis_plot_pccs_ellipses
 * read from plot_debug_t
 * and call vis_plot_pccs_ellipses internally
 */
void
vis_plot_pccs_ellipses_plotdebug (const perllcm_van_plot_debug_t *pd, 
                                  IplImage *imgi, IplImage *imgj,
                                  gsl_matrix *uvi, gsl_matrix *uvj, 
                                  const bool vice_versa, const float scale, int64_t dt);

void 
vis_plot_summary_debugplot  (const perllcm_van_plot_debug_t *pd, 
                             IplImage *imgi, IplImage *imgj,
                             gsl_matrix *uv1_pccs, gsl_matrix *uv2_pccs,
                             const float scale);


#ifdef __cplusplus
}
#endif

#endif //__PERLS_VISION_PLOT_H__
