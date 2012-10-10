#ifndef __PERLS_VISION_BA_H__
#define __PERLS_VISION_BA_H__

#include <glib.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <sba/sba.h>

#include "perls-lcmtypes/perllcm_van_feature_t.h"

#define VIS_SBA_ERROR (-1)

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Wrapper to img_proj_Rae so it can be called externally.  Uses math.h instead of
 * fasttrig.h to avoid numerical errors
 */
void
vis_sba_img_proj_Rae (const gsl_vector *params,
                      const gsl_matrix *K,
                      const gsl_vector *distCoeffs,
                      int npts, 
                      int ncam,
                      int cnp,
                      int pnp,
                      int mnp,
                      gsl_vector *predicted_measurements);

/*
 * Wrapper to img_proj_Rae so it can be called externally.  Uses math.h instead of
 * fasttrig.h to avoid numerical errors
 */
void
vis_sba_img_proj_H (const gsl_vector *params,
                    const gsl_matrix *K,
                    const gsl_vector *distCoeffs,
                    int npts, 
                    int ncam,
                    int cnp,
                    int pnp,
                    int mnp,
                    gsl_vector *predicted_measurements);

/*
 * 2-view jacobian of projection function.  Useful for Haralick-like estimation of
 * relative-pose covariance
 */
void
vis_sba_img_proj_Rae_jacob (const gsl_vector *params,
                            const gsl_matrix *K,
                            const gsl_vector *distCoeffs,
                            int ncam,        // n number of cameras (2 for twoview)
                            gsl_matrix *J);

/*
 * Allocation version of vis_sba_img_proj_Rae_jacob
 */
static inline gsl_matrix *
vis_sba_img_proj_Rae_jacob_alloc (const gsl_vector *params,
                                  const gsl_matrix *K,
                                  const gsl_vector *distCoeffs,
                                  int ncam)
{
    int mnp = 2; //(u,v) feature points
    int cnp = 5; //[a e r p y]
    int pnp = 3; //(x,y,z) 3D scene points
    int npts = (params->size - ncam*cnp) / pnp;
    gsl_matrix *J = gsl_matrix_calloc (ncam*mnp*npts, ncam*cnp + pnp*npts);
    vis_sba_img_proj_Rae_jacob (params, K, distCoeffs, ncam, J);
    return J;
}

/*
 * 2-view jacobian of projection function (homography).  Useful for Haralick-like
 * estimation of relative-pose covariance
 */
void
vis_sba_img_proj_H_jacob (const gsl_vector *params,
                          const gsl_matrix *K,
                          const gsl_vector *distCoeffs,
                          int ncam,        // n number of cameras (2 for twoview)
                          gsl_matrix *J);

/*
 * Allocation version of vis_sba_img_proj_Rae_jacob
 */
static inline gsl_matrix *
vis_sba_img_proj_H_jacob_alloc (const gsl_vector *params,
                                const gsl_matrix *K,
                                const gsl_vector *distCoeffs,
                                int ncam)
{
    int mnp = 2; //(u,v) feature points
    int cnp = 8; //8 for plane-induced homography
    int pnp = 2; //(x,y) 2D scene points in camera 1
    int npts = (params->size - ncam*cnp) / pnp;
    gsl_matrix *J = gsl_matrix_calloc (ncam*mnp*npts, ncam*cnp + pnp*npts);
    vis_sba_img_proj_H_jacob (params, K, distCoeffs, ncam, J);
    return J;
}

/*
 * vis_sba_2v_rae
 * twoview bundle adjustment for 3D structure
 * cost function is defined from two camera matrices P1=[eye(3) zeros(3,1)] and P2 [R t], and 
 * a vector of radial distortion coefficients.
 * It uses 5 dof pose representation in cost function calculation
 *
 */
int
vis_sba_2v_rae_nonlin (const gsl_vector *X_c2c1,     // initial estimate of relative pose, 6 dof [x y z r p h]
                       const gsl_matrix *K,          // calibration matrix [3 x 3] 
                       const gsl_vector *distCoeffs, // distortion coefficient [5 x 1] (k1, k2, p1, p2, k3)
                       const gsl_matrix *uv1,        // uv image points in image 1 [2 x N] 
                       const gsl_matrix *uv2, 
                       const gsl_matrix *X1,         // intial guess of 3D structure points [3 x N]
                       gsl_vector *p12,              // output 1 = optimized 5 dof pose [azim, elev, r, p, h]
                       gsl_matrix *S12,              // output 2 = related covariance matrix [5 x 5]
                       gsl_vector *params,               // optional output 3 = optimized pose + structure
                       int verbose,                  // verbose: 1=ON, 0=OFF
                       GMutex *sba_mutex,
                       void (*f_robust) (const double *const e, int n, double * const w),   // robust cost function
                       const gsl_vector *featscalei_f,
                       const gsl_vector *featscalej_f);

/*
 * vis_sba_2v_rae
 * twoview bundle adjustment for 3D structure
 * cost function is defined from two camera matrices P1=[eye(3) zeros(3,1)] and P2 [R t]
 * it uses 5 dof pose representation in cost function calculation
 *
 */
int
vis_sba_2v_rae (const gsl_vector *X_c2c1,     // initial estimate of relative pose, 6 dof [x y z r p h]    
                const gsl_matrix *K,          // calibration matrix [3 x 3] 
                const gsl_matrix *uv1,        // uv image points in image 1 [2 x N] 
                const gsl_matrix *uv2, 
                const gsl_matrix *X1,         // intial guess of 3D structure points [3 x N]
                gsl_vector *p12,              // output 1 = optimized 5 dof pose [azim, elev, r, p, h]
                gsl_matrix *S12,              // output 2 = related covariance matrix [5 x 5]
                gsl_vector *params,               // optional output 3 = [optimized pose, optimized structure] (set to NULL to disable)
                int verbose,                  // verbose: 1=ON, 0=OFF
                GMutex *sba_mutex,
                void (*f_robust) (const double *const e, int n, double * const w),   // robust cost function
                const gsl_vector *featscalei_f,
                const gsl_vector *featscalej_f);

/*
 * vis_sba_2v_h_nonlin
 * twoview bundle adjustment for planar structure (nonlinear projection)
 * cost function is defined using homography H = K*(R-n't/d)K^-1
 * uses 5 dof pose representation in cost function calculation
 * 
 */
int
vis_sba_2v_h_nonlin (const gsl_vector *X_c2c1,       // initial estimate of relative pose, 6 dof [x y z r p h]    
                     const gsl_matrix *K,            // calibration matrix [3 x 3] 
                     const gsl_vector *distCoeffs,   // distortion coefficient [5 x 1] (k1, k2, p1, p2, k3)
                     const gsl_vector *n_o,          // initial guess for plane normal vector n_o = [0 0 -1]
                     const double d_o,               // initial guess for scene depth (d = mean(Z))
                     const gsl_matrix *uv1,          // uv image points in image 1 [2 x N]  
                     const gsl_matrix *uv2, 
                     const gsl_matrix *uv1_proj,     // intial guess of projected points [2 x N]
                     gsl_vector *p12,                // output 1 = optimized 5 dof pose [azim, elev, r, p, h]
                     gsl_matrix *S12,                // output 2 = related covariance matrix [5 x 5]
                     gsl_vector *params,             // optional output 3 = optimized pose + structure
                     int verbose,                    // verbose: 1=ON, 0=OFF
                     GMutex *sba_mutex,
                     const gsl_vector *featscalei_h,
                     const gsl_vector *featscalej_h);

/*
 * vis_sba_2v_h
 * twoview bundle adjustment for planar structure
 * cost function is defined using homography H = K*(R-n't/d)K^-1
 * uses 5 dof pose representation in cost function calculation
 * 
 */
int
vis_sba_2v_h (const gsl_vector *X_c2c1,       // initial estimate of relative pose, 6 dof [x y z r p h]    
              const gsl_matrix *K,            // calibration matrix [3 x 3] 
              const gsl_vector *n_o,          // initial guess for plane normal vector n_o = [0 0 -1]
              const double d_o,               // initial guess for scene depth (d = mean(Z))
              const gsl_matrix *uv1,          // uv image points in image 1 [2 x N]  
              const gsl_matrix *uv2, 
              const gsl_matrix *uv1_proj,     // intial guess of projected points [2 x N]
              gsl_vector *p12,                // output 1 = optimized 5 dof pose [azim, elev, r, p, h]
              gsl_matrix *S12,                // output 2 = related covariance matrix [5 x 5]
              gsl_vector *params,             // optional output 3 = optimized pose + structure
              int verbose,                    // verbose: 1=ON, 0=OFF
              GMutex *sba_mutex,
              const gsl_vector *featscalei_h,
              const gsl_vector *featscalej_h);

/* run vis_sba_2v_h using Homography model (nonlinear projection)
 * wrapper function for vis_sba_2v_h
 * using Homography, generate uv1p and run BA internally
 * returns the same results as vis_sba_2v_h
 */
int
vis_sba_2v_h_nonlin_from_model (gsl_matrix *H, const gsl_vector *distCoeffs, const gsl_matrix *uvi_h, const gsl_matrix *uvj_h, 
                                const gsl_matrix *K, const perllcm_van_feature_t *fi, const gsl_vector *X_c2c1,
                                gsl_vector *rel_pose21, gsl_matrix *rel_pose_cov21, gsl_vector *params, int verbose,
                                GMutex *sba_mutex, const gsl_vector *featscalei_h, const gsl_vector *featscalej_h);

/* run vis_sba_2v_h using Homography model
 * wrapper function for vis_sba_2v_h
 * using Homography, generate uv1p and run BA internally
 * returns the same results as vis_sba_2v_h
 */
int
vis_sba_2v_h_from_model (gsl_matrix *H, const gsl_matrix *uvi_h, const gsl_matrix *uvj_h, 
                         const gsl_matrix *K, const perllcm_van_feature_t *fi, const gsl_vector *X_c2c1,
                         gsl_vector *rel_pose21, gsl_matrix *rel_pose_cov21, gsl_vector *params, int verbose,
                         GMutex *sba_mutex, const gsl_vector *featscalei_h, const gsl_vector *featscalej_h);

/*
 * Enforce the triangulation constraint.  If all points in uvi_f and uvj_f satisfy the
 * triangulation constraint, they will be copied into uvi_f_triconst and uvj_f_triconst,
 * respectively.  Otherwise points will be selectively copied into uvi_f_triconst and
 * uvj_f_triconst.
 *
 * This is useful for knowing how much to allocate to the optional
 * gsl_vector *params argument in 2-view sba.
 *
 * @note The points uvi_f and uvj_f are assumed to be undistorted
 */
void
vis_sba_2v_rae_enforce_tri_const_alloc (const gsl_matrix *K, const gsl_vector *X_c2c1, 
                                        const gsl_matrix *uvi_f, const gsl_matrix *uvj_f, 
                                        double min_dist, double max_dist,
                                        gsl_matrix **uvi_f_triconst, gsl_matrix **uvj_f_triconst);

/* run vis_sba_2v_rae with triangulation process and nonlinear camera projection
 * wrapper function for vis_sba_2v_rae
 * using relative motion, generate 3D points and run BA internally
 * returns the same results as vis_sba_2v_rae
 */
int
vis_sba_2v_rae_nonlin_from_model_with_tri (const gsl_matrix *K, const gsl_vector *distCoeffs, const gsl_vector *X_c2c1, 
                                           const gsl_matrix *uvi_f, const gsl_matrix *uvj_f, size_t minpt, 
                                           int pt3d_debug_mode, double min_dist, double max_dist,
                                           gsl_vector *rel_pose21, gsl_matrix *rel_pose_cov21, gsl_vector *params, int verbose, gsl_matrix **X1_plot,
                                           GMutex *sba_mutex, const gsl_vector *featscalei_f, const gsl_vector *featscalej_f);

/* run vis_sba_2v_rae with triangulation process
 * wrapper function for vis_sba_2v_rae
 * using relative motion, generate 3D points and run BA internally
 * returns the same results as vis_sba_2v_rae
 */
int
vis_sba_2v_rae_from_model_with_tri (const gsl_matrix *K, const gsl_vector *X_c2c1, const gsl_matrix *uvi_f, const gsl_matrix *uvj_f,
                                    size_t minpt, int pt3d_debug_mode, double min_dist, double max_dist,
                                    gsl_vector *rel_pose21, gsl_matrix *rel_pose_cov21, gsl_vector *params, int verbose, gsl_matrix **X1_plot,
                                    GMutex *sba_mutex, const gsl_vector *featscalei_f, const gsl_vector *featscalej_f);

int
vis_sba_2v_rae_from_model_wo_tri (const gsl_matrix *K, const gsl_vector *X_c2c1, const gsl_matrix *uvi_f, const gsl_matrix *uvj_f,
                                  size_t minpt, int pt3d_debug_mode, double min_dist, double max_dist,
                                  gsl_vector *rel_pose21, gsl_matrix *rel_pose_cov21, int verbose, gsl_matrix **X1_plot,
                                  GMutex *sba_mutex);

/* initialize x21 for SBA using Horn's relative orientation
 * compare mahalanobis distance between relative orientation result and navigation
 * returns error msg and x21_o when there's no error
 */
int32_t
vis_sba_init_relorient (const gsl_matrix *K, const gsl_matrix *uv1, const gsl_matrix *uv2, 
                        const gsl_vector *x21_nav, const gsl_matrix *p21_nav,
                        double min_dist, double max_dist,
                        const double thresh, int verbose, gsl_vector *x21_o);

#ifdef __cplusplus
}
#endif

#endif // __PERLS_VISION_SBA_H__
