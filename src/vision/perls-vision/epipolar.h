#ifndef __PERLS_VISION_EPIPOLAR_H__
#define __PERLS_VISION_EPIPOLAR_H__

#include <stdbool.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VIS_EPI_NUM_POSES_FROM_E 4

/**
 * @defgroup PerlsVisionEpipolar epipolar
 * @brief Epipolar geometry related functions
 * @ingroup PerlsVision
 * @include: perls-vision/epipolar.h
 * @author Ayoung Kim
 * 
 * @{
 */

/**
 * @brief Constructs fundamental matrix from calibration matrix (K), orientation matrix (R) and translation vector (t).
 * \f[F = K [R|t]\f]
 *
 * @param K 3x3 calibration matrix.
 * @param R 3x3 rotation matrix.
 * @param t 3-vector for translation.
 * @param F constructed 3x3 fundamental matrix (returned).
 *
 * @return -1 on error, 1 o.w.
 */
int
vis_epi_F_from_KRT (gsl_matrix *F, const gsl_matrix *K, const gsl_matrix *R, const gsl_vector *t);

/**
 * @brief Constructs fundamental matrix from calibration matrix (K) and 6-vector pose (X).
 * Given \f$X=[t_x, t_y, t_z, roll, pitch, yaw]\f$, \f$R = Rot(z,yaw) \cdot Rot(y,pitch) \cdot Rot(x,roll)\f$ and \f$t=[t_x, t_y, t_z]\f$, 
 * resulting fundamental matrix \f$F = K [R|t]\f$.
 *
 * @param K 3x3 calibration matrix.
 * @param X 6-vector containting orientation change and translation.
 * @param F constructed 3x3 fundamental matrix (returned).
 * @return -1 on error, 1 o.w.
 */
int
vis_epi_F_from_KX (gsl_matrix *F, const gsl_matrix *K, const gsl_vector *X);

/**
 * @brief Calculates the rigid body transformation that aligns the two point sets together using Arun's algorithm.
 * This function uses Arun's method to compute the RBT.
 * \f[corres_{point2} = R*corres_{point1} + t \f]
 * Reference: "Least Squares fitting of two 3D point sets", K. S. Arun, T. S. Huang
 * and S. D. Blostein. IEEE transactions on Pattern Analysis and Machine Intelligence. 
 * Volume PAMI-9 No. 5, September 1987.
 *
 * @param num_point Number of points.
 * @param dim Dimension of space (2 or 3).
 * @param corres_point1 matrix of dimension [dim x num_point].
 * @param corres_point2 matrix of dimension [dim x num_point].
 * @param R Rotation matrix [dim x dim] (returned).
 * @param t Translation vector [dim x 1] (returned).
 *
 * @param R 3x3 relative orientation (returned).
 * @param t 3-vector baseline direction (returned).
 *
 * @return -1 on error, 1 o.w.
 *
 * @author Gaurav Pandy
 */
int
vis_epi_rbt_arun(gsl_matrix *corres_point1, gsl_matrix* corres_point2,
                 int num_point, int dim,  gsl_matrix* R, gsl_vector* t);

/**
 * @brief Calculates the rigid body transformation that aligns the two point sets together using Horn's algorithm.
 * This algorithm is based upon:
 *   Horn, B.K.P.  Relative Orientation, MIT A.I. Memo #994 September 1987
 * 
 * @param K 3x3 calibration matrix.
 * @param uv1 2xN image coordinates (uv2 is also 2xN).
 * @param Ro 3x3 initial guess for relative orientation.
 *
 * @param R 3x3 relative orientation (returned).
 * @param t 3-vector baseline direction (returned).
 *
 * @return Residual error 
 */
double
vis_epi_relorient_horn (const gsl_matrix *K, const gsl_matrix *uv1, const gsl_matrix *uv2, const gsl_matrix *Ro,
                        gsl_matrix *R, gsl_vector *t,
                        bool verbose);

/**
 * @brief Decompose an essential matrix into the 2 possible rotation matrices and 2
 * possible translation vectors, for a total of 4 possibile relative poses. Useful for
 * seeding nonlinear least squares when no other guess of R, t is available (from say,
 * odometry)
 *
 * Method is taken from Berthold K.P. Horn's "Recovering Baseline and Orientation from
 * Essential Matrix"
 *
 * @param E 3x3 Essential matrix
 * @param R_mats pointer to 4 pre-allocated 3x3 rotation matrices
 * @param t_vecs pointer to 4 pre-allocated 3x1 translation vectors
 *
 * @return EXIT_SUCCESS if successful, EXIT_FAILURE if not
 */
int
vis_epi_horn_decomp (const gsl_matrix *E, 
                     gsl_matrix *R_mats[VIS_EPI_NUM_POSES_FROM_E],
                     gsl_vector *t_vecs[VIS_EPI_NUM_POSES_FROM_E]);

/**
 * @brief Free the matrices and vectors allocated by vis_epi_horn_decomp_alloc()
 */
static inline void
vis_epi_horn_decomp_free (gsl_matrix **R_mats, gsl_vector **t_vecs)
{
    if (R_mats) {
        for (int i=0; i<VIS_EPI_NUM_POSES_FROM_E; i++)
            gsl_matrix_free (R_mats[i]);
        free (R_mats);
    }

    if (t_vecs) {
        for (int i=0; i<VIS_EPI_NUM_POSES_FROM_E; i++)
            gsl_vector_free (t_vecs[i]);
        free (t_vecs);
    }
}

/**
 * allocation version of vis_epi_horn_decomp()
 */
static inline int
vis_epi_horn_decomp_alloc (const gsl_matrix *E, gsl_matrix ***R_mats, gsl_vector ***t_vecs)
{
    *R_mats = calloc (VIS_EPI_NUM_POSES_FROM_E, sizeof (**R_mats));
    *t_vecs = calloc (VIS_EPI_NUM_POSES_FROM_E, sizeof (**t_vecs));

    gsl_matrix **_R_mats = *R_mats;
    gsl_vector **_t_vecs = *t_vecs;

    for (int i=0; i<VIS_EPI_NUM_POSES_FROM_E; i++) {
        _R_mats[i] = gsl_matrix_calloc (3, 3);
        _t_vecs[i] = gsl_vector_calloc (3);
    }

    return vis_epi_horn_decomp (E, *R_mats, *t_vecs);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // __PERLS_VISION_EPIPOLAR_H__
