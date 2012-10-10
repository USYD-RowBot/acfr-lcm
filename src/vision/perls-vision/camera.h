#ifndef __PERLS_VISION_CAMERA_H__
#define __PERLS_VISION_CAMERA_H__

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Computes the 3x4 camera projection matrix P = K[R | t]
 *
 * K = 3x3 pinhole camera calibration, if NULL K = eye(3)
 * R = 3x3 orthonormal rotation matrix, if NULL R = eye(3)
 * t = 3x1 translation vector, if NULL t = zeros(3,1)
 */
void
vis_camera_matrix (gsl_matrix *P, const gsl_matrix *K, const gsl_matrix *R, const gsl_vector *t);

// x_lc is 6x1 pose of camera w.r.t. local-level reference frame
void
vis_camera_matrix_from_pose (gsl_matrix *P, const gsl_matrix *K, const double x_lc[6]);

/* Projects 3D points X into camera image plane using the provided projection matrix P
 * P  = 3x4 camera project matrix
 * X  = 3xN or 4xN (homogenous) array of 3D points
 * uv = 2xN or 3xN (homogenous) array of 2D pixel locations
 */
void
vis_camera_project (const gsl_matrix *P, const gsl_matrix *X, gsl_matrix *uv);

/*
 * Projects 3D points X into camera image plane using nonlinear projection model
 * P = 3x4 camera project matrix
 * K = 3x3 camera matrix
 * distCoeffs = 5x1 vector of distortion coefficients: [k1, k2, p1, p2, k3]
 * X = 3xN or 4xN (homogenous) array of 3D points
 * uv = 2xN or 3xN (homogenous) array of 2D pixel locations
 */
void
vis_camera_project_nonlin (const gsl_matrix *P, const gsl_matrix *K, const gsl_vector *distCoeffs, const gsl_matrix *X, gsl_matrix *uv);

/**
 * J -> Jacobian w.r.t  extended state .i.e [mu;X;Y;Z]. 2x9 matrix
 * mu -> mean state (6x1) vector
 * K -> intrinsic camera parameters
 * XYZ -> 3D point
 */
void
vis_camera_projection_jacobian(double J_mu[2*6], double J_XYZ[2*6], double mu[6], double K[3*3], double XYZ[3]); 

#ifdef __cplusplus
}
#endif

#endif // __PERLS_VISION_CAMERA_H__
