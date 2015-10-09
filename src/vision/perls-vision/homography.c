#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/gsl_util.h"
#include "perls-math/gsl_util_vector.h"
#include "perls-math/gsl_util_blas.h"
#include "perls-math/gsl_util_linalg.h"
#include "perls-math/homogenous.h"

#include "homography.h"
#include "distortion.h"


void
vis_homog_matrix_plane_induced (gsl_matrix *H, const gsl_matrix *K, const gsl_matrix *R,
                                const gsl_vector *t, const gsl_vector *n, const double d)
{
    assert (H->size1 == 3 && H->size2 == 3 &&
            K->size1 == 3 && K->size2 == 3 &&
            R->size1 == 3 && R->size2 == 3 &&
            t->size  == 3 && n->size  == 3);

    // Kinv
    GSLU_MATRIX_VIEW (Kinv_view, 3, 3);
    gslu_matrix_inv (&Kinv_view.matrix, K);

    // tnd = t * n^T / d
    GSLU_MATRIX_VIEW (tnd_view, 3, 3);
    gsl_matrix_const_view t_view = gsl_matrix_const_view_vector (t, t->size, 1);
    gsl_matrix_const_view n_view = gsl_matrix_const_view_vector (n, n->size, 1);
    gslu_blas_mmT (&tnd_view.matrix, &t_view.matrix, &n_view.matrix);
    gsl_matrix_scale (&tnd_view.matrix, 1.0/d);

    // (R-t*n'/d)
    GSLU_MATRIX_VIEW (Rtnd_view, 3, 3);
    gsl_matrix_memcpy (&Rtnd_view.matrix, R);
    gsl_matrix_sub (&Rtnd_view.matrix, &tnd_view.matrix);

    // H = K*(R-t*n'/d)*K^-1
    GSLU_MATRIX_VIEW (H_view, 3, 3);
    GSLU_MATRIX_VIEW (work, 3, 3);
    gslu_blas_mmm (H, K, &Rtnd_view.matrix, &Kinv_view.matrix, &work.matrix);
}


void
vis_homog_project (const gsl_matrix *H, const gsl_matrix *uv1, gsl_matrix *uv2)
{
    assert ( gslu_matrix_is_same_size (uv1, uv2) && H->size1 == 3 && H->size2 == 3);

    gsl_matrix *uv1_h, *uv2_h;
    if (uv1->size1 == 2)
    {
        uv1_h = homogenize_alloc (uv1);
        uv2_h = gsl_matrix_alloc (3, uv2->size2);
    }
    else
    {
        uv1_h = (gsl_matrix*) uv1;
        uv2_h = uv2;
    }
    gslu_blas_mm (uv2_h, H, uv1_h);

    if (uv1->size1 == 2)
    {
        dehomogenize (uv2_h, uv2);
        gsl_matrix_free (uv1_h);
        gsl_matrix_free (uv2_h);
    }
}

void
vis_homog_project_nonlin (const gsl_matrix *H, const gsl_matrix *K, const gsl_vector *distCoeffs, const gsl_matrix *uv1, gsl_matrix *uv2)
{
    gsl_matrix *uv2_tmp = gsl_matrix_alloc (uv2->size1, uv2->size2);
    /* project the points using homography */
    vis_homog_project (H, uv1, uv2_tmp);
    /* apply radial distortion */
    vis_distort_pts_radial (uv2_tmp, uv2, K, distCoeffs);
    gsl_matrix_free (uv2_tmp);
}

gsl_matrix *
vis_homog_project_alloc (const gsl_matrix *H, const gsl_matrix *uv1)
{
    assert ( (uv1->size1 == 2 || uv1->size1 == 3) && H->size1 == 3 && H->size2 == 3);

    gsl_matrix *uv1_h, *uv2_h, *uv2;

    if (uv1->size1 == 2)
        uv1_h = homogenize_alloc (uv1);
    else
        uv1_h = (gsl_matrix*) uv1;

    uv2 = gsl_matrix_alloc (2, uv1->size2);
    uv2_h = gsl_matrix_alloc (3, uv1->size2);

    gslu_blas_mm (uv2_h, H, uv1_h);

    if (uv1->size1 == 2)
    {
        dehomogenize (uv2_h, uv2);
        gsl_matrix_free (uv1_h);
        gsl_matrix_free (uv2_h);

        return uv2;
    }
    else
    {
        gsl_matrix_free (uv1_h);
        gsl_matrix_free (uv2);

        return uv2_h;
    }
}

void
vis_homog_single_gsl_pt_project (const gsl_matrix *H, const gsl_vector *uv1, gsl_vector *uv2)
{
    /*
        uv2 = Hinf * uv1
        [uvp_h1]  = [H11 H12 H13] [u] + [K11 K12 K13] [t1/z]
        [uvp_h2]    [H21 H22 H23] [v] + [K21 K22 K23] [t2/z]
        [uvp_h3]    [H31 H32 H33] [1] + [K11 K12 K13] [t3/z]
    */

    double u = gsl_vector_get (uv1,0);
    double v = gsl_vector_get (uv1,1);

    double H11 = gsl_matrix_get (H, 0,0);
    double H12 = gsl_matrix_get (H, 0,1);
    double H13 = gsl_matrix_get (H, 0,2);
    double H21 = gsl_matrix_get (H, 1,0);
    double H22 = gsl_matrix_get (H, 1,1);
    double H23 = gsl_matrix_get (H, 1,2);
    double H31 = gsl_matrix_get (H, 2,0);
    double H32 = gsl_matrix_get (H, 2,1);
    double H33 = gsl_matrix_get (H, 2,2);

    double up = (H11*u+H12*v+H13) / (H31*u+H32*v+H33);
    double vp = (H21*u+H22*v+H23) / (H31*u+H32*v+H33);

    // return uvp
    gsl_vector_set (uv2, 0, up);
    gsl_vector_set (uv2, 1, vp);

}

gsl_vector *
vis_homog_single_gsl_pt_project_alloc (const gsl_matrix *H, const gsl_vector *uv1)
{
    /*
        uv2 = Hinf * uv1
        [uvp_h1]  = [H11 H12 H13] [u] + [K11 K12 K13] [t1/z]
        [uvp_h2]    [H21 H22 H23] [v] + [K21 K22 K23] [t2/z]
        [uvp_h3]    [H31 H32 H33] [1] + [K11 K12 K13] [t3/z]
    */
    double u = gsl_vector_get (uv1,0);
    double v = gsl_vector_get (uv1,1);

    gsl_vector *uv2 = gsl_vector_alloc (2);
    double H11 = gsl_matrix_get (H, 0,0);
    double H12 = gsl_matrix_get (H, 0,1);
    double H13 = gsl_matrix_get (H, 0,2);
    double H21 = gsl_matrix_get (H, 1,0);
    double H22 = gsl_matrix_get (H, 1,1);
    double H23 = gsl_matrix_get (H, 1,2);
    double H31 = gsl_matrix_get (H, 2,0);
    double H32 = gsl_matrix_get (H, 2,1);
    double H33 = gsl_matrix_get (H, 2,2);

    double up = (H11*u+H12*v+H13) / (H31*u+H32*v+H33);
    double vp = (H21*u+H22*v+H23) / (H31*u+H32*v+H33);

    // return uvp
    gsl_vector_set (uv2, 0, up);
    gsl_vector_set (uv2, 1, vp);

    return uv2;
}

void
vis_homog_single_pt_project (const gsl_matrix *H, const double u, double v, double *up, double *vp)
{
    /*
        uv2 = Hinf * uv1
        [uvp_h1]  = [H11 H12 H13] [u] + [K11 K12 K13] [t1/z]
        [uvp_h2]    [H21 H22 H23] [v] + [K21 K22 K23] [t2/z]
        [uvp_h3]    [H31 H32 H33] [1] + [K11 K12 K13] [t3/z]
    */

    double H11 = gsl_matrix_get (H, 0,0);
    double H12 = gsl_matrix_get (H, 0,1);
    double H13 = gsl_matrix_get (H, 0,2);
    double H21 = gsl_matrix_get (H, 1,0);
    double H22 = gsl_matrix_get (H, 1,1);
    double H23 = gsl_matrix_get (H, 1,2);
    double H31 = gsl_matrix_get (H, 2,0);
    double H32 = gsl_matrix_get (H, 2,1);
    double H33 = gsl_matrix_get (H, 2,2);

    *up = (H11*u+H12*v+H13) / (H31*u+H32*v+H33);
    *vp = (H21*u+H22*v+H23) / (H31*u+H32*v+H33);
}

void
vis_homog_matrix_infinite (gsl_matrix *Hinf, const gsl_matrix *K, const gsl_matrix *R)
{
    GSLU_MATRIX_VIEW (work, 3, 3);
    GSLU_MATRIX_VIEW (invK, 3, 3);
    gslu_matrix_inv (&invK.matrix, K);
    gslu_blas_mmm (Hinf, K, R, &invK.matrix, &work.matrix);             // K*R*inv(K)
}

gsl_matrix *
vis_homog_matrix_infinite_alloc (const gsl_matrix *K, const gsl_matrix *R)
{
    gsl_matrix *Hinf = gsl_matrix_alloc (3,3);

    GSLU_MATRIX_VIEW (work, 3, 3);
    GSLU_MATRIX_VIEW (invK, 3, 3);
    gslu_matrix_inv (&invK.matrix, K);
    gslu_blas_mmm (Hinf, K, R, &invK.matrix, &work.matrix);             // K*R*inv(K)

    return Hinf;
}

int
vis_homog_pose_decomp (const gsl_matrix *H,
                       gsl_matrix *R_mats[VIS_HOMOG_NUM_POSES_FROM_H],
                       gsl_vector *t_vecs[VIS_HOMOG_NUM_POSES_FROM_H],
                       gsl_vector *N_vecs[VIS_HOMOG_NUM_POSES_FROM_H])
{
    assert (H->size1 == 3 && H->size2 == 3);

    /* compute eigenvalues of H'*H */
    gsl_matrix *HTH = gslu_blas_mTm_alloc (H, H);
    gslu_eigen *eigsHTH = gslu_eigen_decomp_alloc (HTH);

    /* scale H by middle eigenvalue */
    double scaleFactor = 1 / sqrt (gsl_vector_get(eigsHTH->D, 1));
    gsl_matrix *HScaled = gsl_matrix_calloc (H->size1, H->size2);
    gsl_matrix_memcpy (HScaled, H);
    gsl_matrix_scale (HScaled, scaleFactor);

    /* compute SVD of HScaled'*HScaled */
    gsl_matrix *HTHScaled = gslu_blas_mTm_alloc (HScaled, HScaled);
    gslu_linalg_SV *svdHTHScaled = gslu_linalg_SV_decomp_full_alloc (HTHScaled);

    /* Make sure det(V) > 0 */
    if (gslu_matrix_det (svdHTHScaled->V) < 0)
        gsl_matrix_scale (svdHTHScaled->V, -1);

    /* si is the ith singular value */
    double s1, s3;
    s1 = gsl_vector_get (svdHTHScaled->S, 0);
    s3 = gsl_vector_get (svdHTHScaled->S, 2);
    /* vi is the ith right singular vector */
    gsl_vector_view v1, v2, v3;
    v1 = gsl_matrix_column (svdHTHScaled->V, 0);
    v2 = gsl_matrix_column (svdHTHScaled->V, 1);
    v3 = gsl_matrix_column (svdHTHScaled->V, 2);

    gsl_vector *u1, *u2;
    u1 = gsl_vector_calloc (3);
    u2 = gsl_vector_calloc (3);

    /* gslu_vector_printf (svdHTHScaled->S, "S"); */
    /* printf ("%f --- %f\n", s1, s3); */

    /* We are trying to compute:
       u1 = (sqrt(1-s3)*v1 + sqrt(s1 - 1)*v3) / sqrt(s1 - s3);
       u2 = (sqrt(1-s3)*v1 - sqrt(s1 - 1)*v3) / sqrt(s1 - s3);
    */
    gsl_vector *scaledV1, *scaledV3;
    scaledV1 = gsl_vector_calloc (3);
    scaledV3 = gsl_vector_calloc (3);
    gsl_vector_memcpy (scaledV1, &v1.vector);
    gsl_vector_memcpy (scaledV3, &v3.vector);
    gsl_vector_scale (scaledV1, sqrt (1-s3));
    gsl_vector_scale (scaledV3, sqrt (s1-1));
    gsl_vector_memcpy (u1, scaledV1);
    gsl_vector_add (u1, scaledV3);
    gsl_vector_scale (u1, 1 / sqrt (s1 - s3));
    gsl_vector_memcpy (u2, scaledV1);
    gsl_vector_sub (u2, scaledV3);
    gsl_vector_scale (u2, 1 / sqrt (s1 - s3));

    /* We are trying to compute:
     U1 = [v2 u1 cross(v2, u1)];
     U2 = [v2 u2 cross(v2, u2)];
     W1 = [H*v2 H*u1 cross(H*v2, H*u1)];
     W2 = [H*v2 H*u2 cross(H*v2, H*u2)];
     */
    gsl_matrix *U1, *U2, *W1, *W2;
    U1 = gsl_matrix_calloc (3, 3);
    U2 = gsl_matrix_calloc (3, 3);
    W1 = gsl_matrix_calloc (3, 3);
    W2 = gsl_matrix_calloc (3, 3);

    /* Get columns of all the above */
    gsl_vector_view U1Col1 = gsl_matrix_column (U1, 0);
    gsl_vector_view U1Col2 = gsl_matrix_column (U1, 1);
    gsl_vector_view U1Col3 = gsl_matrix_column (U1, 2);
    gsl_vector_view U2Col1 = gsl_matrix_column (U2, 0);
    gsl_vector_view U2Col2 = gsl_matrix_column (U2, 1);
    gsl_vector_view U2Col3 = gsl_matrix_column (U2, 2);
    gsl_vector_view W1Col1 = gsl_matrix_column (W1, 0);
    gsl_vector_view W1Col2 = gsl_matrix_column (W1, 1);
    gsl_vector_view W1Col3 = gsl_matrix_column (W1, 2);
    gsl_vector_view W2Col1 = gsl_matrix_column (W2, 0);
    gsl_vector_view W2Col2 = gsl_matrix_column (W2, 1);
    gsl_vector_view W2Col3 = gsl_matrix_column (W2, 2);
    gsl_vector *HScaledV2 = gslu_blas_mv_alloc (HScaled, &v2.vector);
    gsl_vector *HScaledU1 = gslu_blas_mv_alloc (HScaled, u1);
    gsl_vector *HScaledU2 = gslu_blas_mv_alloc (HScaled, u2);

    /* U1, U2 */
    gsl_vector_memcpy (&U1Col1.vector, &v2.vector);
    gsl_vector_memcpy (&U1Col2.vector, u1);
    gslu_vector_cross (&U1Col3.vector, &v2.vector, u1);
    gsl_vector_memcpy (&U2Col1.vector, &v2.vector);
    gsl_vector_memcpy (&U2Col2.vector, u2);
    gslu_vector_cross (&U2Col3.vector, &v2.vector, u2);

    /* W1, W2 */
    gslu_blas_mv (&W1Col1.vector, HScaled, &v2.vector);
    gslu_blas_mv (&W1Col2.vector, HScaled, u1);
    gslu_vector_cross (&W1Col3.vector, HScaledV2, HScaledU1);
    gslu_blas_mv (&W2Col1.vector, HScaled, &v2.vector);
    gslu_blas_mv (&W2Col2.vector, HScaled, u2);
    gslu_vector_cross (&W2Col3.vector, HScaledV2, HScaledU2);

    /* Compute the 4 mathematically possible solutions, only return the 2 phyiscally
     * possible solutions */
    gsl_matrix *R = gsl_matrix_calloc (3, 3);
    gsl_vector *t = gsl_vector_calloc (3);
    gsl_vector *N = gsl_vector_calloc (3);
    gsl_matrix *HScaledMinusR;
    int j=0;
    for (int i=0; i<4; i++)
    {

        switch (i)
        {
        case 0:
            gslu_blas_mmT (R, W1, U1);
            gslu_vector_cross (N, &v2.vector, u1);
            HScaledMinusR = gsl_matrix_calloc (3, 3);
            gsl_matrix_memcpy (HScaledMinusR, HScaled);
            gsl_matrix_sub (HScaledMinusR, R);
            gslu_blas_mv (t, HScaledMinusR, N);
            break;
        case 1:
            gslu_blas_mmT (R, W2, U2);
            gslu_vector_cross (N, &v2.vector, u2);
            HScaledMinusR = gsl_matrix_calloc (3, 3);
            gsl_matrix_memcpy (HScaledMinusR, HScaled);
            gsl_matrix_sub (HScaledMinusR, R);
            gslu_blas_mv (t, HScaledMinusR, N);
            break;
        case 2:
            gslu_blas_mmT (R, W1, U1);
            gslu_vector_cross (N, &v2.vector, u1);
            gsl_vector_scale (N, -1);
            HScaledMinusR = gsl_matrix_calloc (3, 3);
            gsl_matrix_memcpy (HScaledMinusR, HScaled);
            gsl_matrix_sub (HScaledMinusR, R);
            gslu_blas_mv (t, HScaledMinusR, N);
            break;
        case 3:
            gslu_blas_mmT (R, W2, U2);
            gslu_vector_cross (N, &v2.vector, u2);
            gsl_vector_scale (N, -1);
            HScaledMinusR = gsl_matrix_calloc (3, 3);
            gsl_matrix_memcpy (HScaledMinusR, HScaled);
            gsl_matrix_sub (HScaledMinusR, R);
            gslu_blas_mv (t, HScaledMinusR, N);
            break;
        }

        if (gsl_vector_get (N, 2) > 0)
        {
            if (j == 3)
                return EXIT_FAILURE;
            gsl_matrix_memcpy (R_mats[j], R);
            gsl_vector_memcpy (N_vecs[j], N);
            gsl_vector_memcpy (t_vecs[j], t);
            j++;
        }
    }

    /* clean up */
    gslu_eigen_free (eigsHTH);
    gsl_matrix_free (HTH);
    gsl_matrix_free (HScaled);
    gsl_matrix_free (HTHScaled);
    gslu_linalg_SV_free (svdHTHScaled);
    gsl_vector_free (u1);
    gsl_vector_free (u2);
    gsl_vector_free (scaledV1);
    gsl_vector_free (scaledV3);
    gsl_matrix_free (U1);
    gsl_matrix_free (U2);
    gsl_matrix_free (W1);
    gsl_matrix_free (W2);
    gsl_vector_free (HScaledV2);
    gsl_vector_free (HScaledU1);
    gsl_vector_free (HScaledU2);
    gsl_matrix_free (R);
    gsl_vector_free (t);
    gsl_vector_free (N);

    return EXIT_SUCCESS;
}
