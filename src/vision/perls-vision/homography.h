#ifndef __PERLS_VISION_HOMOGRAPHY_H__
#define __PERLS_VISION_HOMOGRAPHY_H__

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/**
 * @defgroup PerlsVisionHomography homography
 * @brief Homograpy related functions
 * @ingroup PerlsVision
 * @include: perls-vision/homography.h
 * @author Ayoung Kim
 * 
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#define VIS_HOMOG_NUM_POSES_FROM_H 2

/**
 * @brief Computes the plane-induced homography \f$H = K (R - t n'/d) K^{-1}\f$.
 *
 * @param H 3x3 homography marix.
 * @param K 3x3 pinhole camera calibration.
 * @param R 3x3 othonormal rotation matrix.
 * @param t 3x1 translation vector.
 * @param n 3x1 plane normal vector.
 * @param d camera orthogonal distance to the plane.
 */
void
vis_homog_matrix_plane_induced (gsl_matrix *H, const gsl_matrix *K, const gsl_matrix *R, 
                                const gsl_vector *t, const gsl_vector *n, const double d);


/**
 * @brief Projects pixel locations from image 1 into image 2, i.e.  uv2 = H*uv1.
 * @param H 3x3 homography matrix.
 * @param uv1 2xN or 3xN (homogenous) array of 2D pixel locations.
 * @param uv2 2xN or 3xN (homogenous) array of 2D pixel locations.
 */
void
vis_homog_project (const gsl_matrix *H, const gsl_matrix *uv1, gsl_matrix *uv2);

/**
 * @brief Projects pixel locations from image 1 into image 2, i.e.  uv2 = H*uv1.
 * @param H 3x3 homography matrix.
 * @param K 3x3 intrinsic matrix.
 * @param distCoeffs 5x1 vector of distortion coefficients
 * @param uv1 2xN or 3xN (homogenous) array of UNDISTORTED 2D pixel locations.
 * @param uv2 2xN or 3xN (homogenous) array of 2D pixel locations.
 */
void
vis_homog_project_nonlin (const gsl_matrix *H, const gsl_matrix *K, const gsl_vector *distCoeffs, const gsl_matrix *uv1, gsl_matrix *uv2);

/**
 * @brief Allocation version of vis_homog_project. It projects pixel locations from image 1 into image 2, i.e.  uv2 = H*uv1.
 * @see vis_homog_project.
 * 
 * @return Allocted projected pixel points.
 */
gsl_matrix *
vis_homog_project_alloc (const gsl_matrix *H, const gsl_matrix *uv1);

/** 
 * @brief Single point version of vis_homog_project. It projects a single point [uv] using homography.
 * @see vis_homog_project.
 */
void
vis_homog_single_gsl_pt_project (const gsl_matrix *H, const gsl_vector *uv1, gsl_vector *uv2);

/**
 * @brief Allocation version of vis_homog_single_gsl_pt_project.
 * @see vis_homog_single_gsl_pt_project.
 * @return projected uv pixel point.
 */ 
gsl_vector *
vis_homog_single_gsl_pt_project_alloc (const gsl_matrix *H, const gsl_vector *uv1);

/**
 * @brief Same function as vis_homog_single_gsl_pt_project when inputs are all doubles.
 * @see vis_homog_single_gsl_pt_project.
 *
 * @param H homography to map uv to uvp.
 * @param u pixel point (or v).
 * @param up projected pixel point (or vp).
 */
void
vis_homog_single_pt_project (const gsl_matrix *H, const double u, double v, double *up, double *vp);

/**
 * @brief Computes infinite homography \f$ H = K R K^{-1} \f$.
 * 
 * @param K calibration matrix.
 * @param R rotation matrix.
 * @param Hinf Infinite homograpy computed (returned).
 */
void
vis_homog_matrix_infinite (gsl_matrix *Hinf, const gsl_matrix *K, const gsl_matrix *R);

/**
 * @brief Allocation version of vis_homog_matrix_infinite.
 * @see vis_homog_matrix_infinite.
 * @param K calibration matrix.
 * @param R rotation matrix.
 *
 * @return Hinf Infinite homograpy computed (returned).
 */
gsl_matrix *
vis_homog_matrix_infinite_alloc (const gsl_matrix *K, const gsl_matrix *R);

/**
 * @brief Decompose plane-induced homography into 2 physically possible rotation and
 * translation.  Taken from "An Invitation to 3D Vision" chapter 5.
 *
 * @param H 3x3 Homography matrix between two images
 * @param R_mats pointer to 2 pre-allocated 3x3 rotation matrices
 * @param t_vecs pointer to 2 pre-allocated 3x1 translation vectors
 * @param N_vecs pointer to 2 pre-allocated 3x1 surface normal vectors
 *
 * @note Of the two possibilities, the user must decide which one is the true pose from
 * external methods, like a prior on the triangulated 3D scene points.
 *
 * @return EXIT_SUCCESS if successful, EXIT_FAILURE if not
 */
int
vis_homog_pose_decomp (const gsl_matrix *H, 
                       gsl_matrix *R_mats[VIS_HOMOG_NUM_POSES_FROM_H],
                       gsl_vector *t_vecs[VIS_HOMOG_NUM_POSES_FROM_H],
                       gsl_vector *N_vecs[VIS_HOMOG_NUM_POSES_FROM_H]);

/**
 * @brief Free the matrices and vectors allocated by vis_homog_pose_decomp()
 */
static inline void
vis_homog_pose_decomp_free (gsl_matrix **R_mats, gsl_vector **t_vecs, gsl_vector **N_vecs)
{
    if (R_mats) {
        for (int i=0; i<VIS_HOMOG_NUM_POSES_FROM_H; i++)
            gsl_matrix_free (R_mats[i]);
        free (R_mats);
    }
    
    if (t_vecs) {
        for (int i=0; i<VIS_HOMOG_NUM_POSES_FROM_H; i++)
            gsl_vector_free (t_vecs[i]);
        free (t_vecs);
    }

    if (N_vecs) {
        for (int i=0; i<VIS_HOMOG_NUM_POSES_FROM_H; i++)
            gsl_vector_free (N_vecs[i]);
        free (N_vecs);
    }
}

/**
 * @brief Allocation version of vis_homog_pose_decomp()
 */
static inline int
vis_homog_pose_decomp_alloc (const gsl_matrix *H, gsl_matrix ***R_mats, gsl_vector ***t_vecs, gsl_vector ***N_vecs)
{
    *R_mats = calloc (VIS_HOMOG_NUM_POSES_FROM_H, sizeof (**R_mats));
    *t_vecs = calloc (VIS_HOMOG_NUM_POSES_FROM_H, sizeof (**t_vecs));
    *N_vecs = calloc (VIS_HOMOG_NUM_POSES_FROM_H, sizeof (**N_vecs));

    gsl_matrix **_R_mats = *R_mats;
    gsl_vector **_t_vecs = *t_vecs;
    gsl_vector **_N_vecs = *N_vecs;

    for (int i=0; i<VIS_HOMOG_NUM_POSES_FROM_H; i++) {
        _R_mats[i] = gsl_matrix_calloc (3, 3);
        _t_vecs[i] = gsl_vector_calloc (3);
        _N_vecs[i] = gsl_vector_calloc (3);
    }

    return vis_homog_pose_decomp (H, *R_mats, *t_vecs, *N_vecs);
}

#ifdef __cplusplus
}
#endif

#endif // __PERLS_VISION_HOMOGRAPHY_H__
