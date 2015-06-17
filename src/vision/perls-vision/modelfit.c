#include <stdio.h>
#include <stdlib.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_poly.h>

#include <opencv/cv.h>

#include "perls-common/error.h"
#include "perls-math/gsl_util.h"
#include "perls-math/homogenous.h"
#include "perls-math/ransac.h"

#include "opencv_util.h"
#include "modelfit.h"
#include "homography.h"

#define VIS_MODELFIT_TORR_LAMBDA1 2
#define VIS_MODELFIT_TORR_LAMBDA2 4

#define VIS_MODELFIT_H_RANSAC_SAMPLE_SIZE 5
#define VIS_MODELFIT_E_RANSAC_SAMPLE_SIZE 6

/* enable debug calls to gslu_matrix_printf ?*/
//#define MODELFIT_VERBOSE

double
_modelfit_compute_gic (const gsl_vector *reprojection_error, vis_modelfit_model_t model_type,
                       double lambda1, double lambda2)
{

    // assing dof struct and model according to model_type
    int dof_struct, dof_model;

    if (model_type == VIS_MODELFIT_H)
    {
        dof_struct = 2;  // 2D structure
        dof_model  = 8;  // 8 dof for homograpy
    }
    else if (model_type == VIS_MODELFIT_F)
    {
        dof_struct = 3;  // 3D structure
        dof_model  = 7;  // 7 dof for fundamental matrix
    }
    else if (model_type == VIS_MODELFIT_E)
    {
        dof_struct = 3;  // 3D structure
        dof_model  = 5;  // 5 dof for essential matrix
    }
    else
    {
        ERROR ("Unknown model type to compute gic.");
        abort ();
    }

    // compute GIC score
    if (reprojection_error)
    {
        int N = reprojection_error->size;

        // gic = sum of reprj error + lambda1*dof_struct*N + lambda2*dof_model
        if (N > 0) // number of pair > 0
            return gslu_vector_sum (reprojection_error) + lambda1*dof_struct*N + lambda2*dof_model;
        else
            return GSL_POSINF;
    }
    else // NULL error
        return GSL_POSINF;
}

gsl_vector *
_modelfit_reprojection_error_H (const gsl_matrix *uv1, const gsl_matrix *uv2,
                                const gsl_matrix *H)
{
    assert (uv1->size1 == 2 && uv1->size2 == uv2->size2);

    int N = uv1->size2;

    // err = (u1-u2p)^2 + (v1-v2p)^2 + (u2-u1p)^2 + (v2-v1p)^2
    gsl_vector *err = gsl_vector_alloc (N);

    // inv of model
    GSLU_MATRIX_VIEW (H_inv, 3, 3);
    gslu_matrix_inv (&H_inv.matrix, H);
    gsl_matrix *work = gsl_matrix_alloc (2, N);

    // project fwd & bwd
    vis_homog_project (&H_inv.matrix, uv2, work);       // work = uv1p
    gsl_matrix_sub (work, uv1);                         // work = (uv2p - uv1)^2
    gsl_matrix_mul_elements (work, work);
    gsl_vector_view u_part = gsl_matrix_row (work, 0);
    gsl_vector_view v_part = gsl_matrix_row (work, 1);
    gsl_vector_memcpy (err, &u_part.vector);
    gsl_vector_add (err, &v_part.vector);

    vis_homog_project (H, uv1, work);                   // work = uv2p
    gsl_matrix_sub (work, uv2);                         // work = (uv1p - uv2)^2
    gsl_matrix_mul_elements (work, work);
    u_part = gsl_matrix_row (work, 0);
    v_part = gsl_matrix_row (work, 1);
    gsl_vector_add (err, &u_part.vector);
    gsl_vector_add (err, &v_part.vector);

    gslu_matrix_free (work);

    return err; // err_on1 = err_on1 + err_on2
}

gsl_vector *
_modelfit_reprojection_error_F (const gsl_matrix *uv1, const gsl_matrix *uv2,
                                const gsl_matrix *F)
{
    assert (uv1->size1 == 2 && uv1->size2 == uv2->size2);

    int N = uv1->size2;
    gsl_vector *err = gsl_vector_alloc (N);

    // transpose of model
    GSLU_MATRIX_VIEW (F_trans, 3,3);
    gsl_matrix_transpose_memcpy (&F_trans.matrix, F);

    GSLU_VECTOR_VIEW (lon1, 3);
    GSLU_VECTOR_VIEW (lon2, 3);
    GSLU_VECTOR_VIEW (uv1_h, 3, {0, 0, 1});
    GSLU_VECTOR_VIEW (uv2_h, 3, {0, 0, 1});

    for (size_t i=0; i<N; i++)
    {
        gsl_vector_set (&uv1_h.vector, 0, gsl_matrix_get (uv1, 0, i));
        gsl_vector_set (&uv1_h.vector, 1, gsl_matrix_get (uv1, 1, i));
        gsl_vector_set (&uv2_h.vector, 0, gsl_matrix_get (uv2, 0, i));
        gsl_vector_set (&uv2_h.vector, 1, gsl_matrix_get (uv2, 1, i));

        // point to line dist = |ax+by+c| / sqrt(a*a+b*b)
        // uv2 ~ epi line 1 distance
        gslu_blas_mv (&lon2.vector, F, &uv1_h.vector);
        double a = gsl_vector_get (&lon2.vector, 0);
        double b = gsl_vector_get (&lon2.vector, 1);
        double err_sqr1 = gslu_vector_dot (&lon2.vector, &uv2_h.vector);
        err_sqr1 = err_sqr1*err_sqr1/(a*a+b*b);

        // uv1 ~ epi line 2 distance
        gslu_blas_mv (&lon1.vector, &F_trans.matrix, &uv2_h.vector);
        a = gsl_vector_get (&lon1.vector, 0);
        b = gsl_vector_get (&lon1.vector, 1);
        double err_sqr2 = gslu_vector_dot (&lon1.vector, &uv1_h.vector);
        err_sqr2 = err_sqr2*err_sqr2/(a*a+b*b);

        gsl_vector_set (err, i, err_sqr1+err_sqr2);
    }

    return err; // err_on1 = err_on1 + err_on2
}

gsl_vector *
_modelfit_sampson_error_F (const gsl_matrix *xy1, const gsl_matrix *xy2,
                           const gsl_matrix *D, const gsl_matrix *F)
{
    assert (xy1->size1 == 2 && xy1->size2 == xy2->size2);

    int N = xy1->size2;
    gsl_vector *err = gsl_vector_alloc (N);

    // transpose of model
    GSLU_MATRIX_VIEW (F_trans, 3,3);
    gsl_matrix_transpose_memcpy (&F_trans.matrix, F);

    GSLU_VECTOR_VIEW (J1_row, 3);
    GSLU_VECTOR_VIEW (J2_row, 3);
    GSLU_VECTOR_VIEW (xy1_h, 3, {0, 0, 1});
    GSLU_VECTOR_VIEW (xy2_h, 3, {0, 0, 1});
    GSLU_VECTOR_VIEW (F_col, 9);

    for (size_t i=0; i<N; i++)
    {
        gsl_vector_set (&xy1_h.vector, 0, gsl_matrix_get (xy1, 0, i));
        gsl_vector_set (&xy1_h.vector, 1, gsl_matrix_get (xy1, 1, i));
        gsl_vector_set (&xy2_h.vector, 0, gsl_matrix_get (xy2, 0, i));
        gsl_vector_set (&xy2_h.vector, 1, gsl_matrix_get (xy2, 1, i));

        // error = eps^2 / J^2
        gslu_blas_mv (&J1_row.vector, &F_trans.matrix, &xy1_h.vector);
        gslu_blas_mv (&J2_row.vector, F, &xy2_h.vector);

        double jacob = gsl_vector_get (&J1_row.vector, 0)*gsl_vector_get (&J1_row.vector, 0)
                       + gsl_vector_get (&J1_row.vector, 1)*gsl_vector_get (&J1_row.vector, 1)
                       + gsl_vector_get (&J2_row.vector, 0)*gsl_vector_get (&J2_row.vector, 0)
                       + gsl_vector_get (&J2_row.vector, 1)*gsl_vector_get (&J2_row.vector, 1);

        gsl_vector_const_view D_row = gsl_matrix_const_row (D, i);
        gslu_matrix_stack (&F_col.vector, F, CblasNoTrans);
        double eps = gslu_vector_dot (&F_col.vector, &D_row.vector);

        gsl_vector_set (err, i, eps/jacob);
    }

    return err; // err_on1 = err_on1 + err_on2
}

double
vis_modelfit_gic (const gsl_matrix* model, vis_modelfit_model_t model_type,
                  const gsl_matrix* uv1, const gsl_matrix* uv2,
                  double lambda1, double lambda2)
{
    double gic = GSL_POSINF;

    gsl_vector *err = vis_modelfit_reprojection_error (uv1, uv2, model, model_type);
    gic = _modelfit_compute_gic (err, model_type, lambda1, lambda2);
    gslu_vector_free (err);

    return gic;
}

gsl_vector *
vis_modelfit_reprojection_error (const gsl_matrix *uv1, const gsl_matrix *uv2,
                                 const gsl_matrix *model, vis_modelfit_model_t model_type)
{
    gsl_vector *err_ret = NULL;
    if (model_type == VIS_MODELFIT_H)
    {
        err_ret = _modelfit_reprojection_error_H (uv1, uv2, model);
    }
    else if (model_type == VIS_MODELFIT_F)
    {
        err_ret = _modelfit_reprojection_error_F (uv1, uv2, model);
    }
    else if (model_type == VIS_MODELFIT_E)
    {
        ERROR ("VIS_MODELFIT_E reprojection error not implemented.");
        abort ();
    }
    else
    {
        ERROR ("Unknown model type to compute reprojection error.");
        abort ();
    }

    return err_ret;
}

/*
 * computes the [Nx9] measurement matrix D used for SVD calculation
 * of the fundamental matrix f or essential matrix E
 * D = [xj.*xi, xj.*yi, xj, yj.*xi, yj.*yi, yj, xi, yi, ones(n,1)];
 *
 * xy1 2xN matrix with normalized coordinates
 * xy2 2xN matrix with normalized coordinates
 */

gsl_matrix *
_F_meas_matrix_alloc (const gsl_matrix *xy1, const gsl_matrix *xy2)
{
    gsl_vector_const_view x1 = gsl_matrix_const_row (xy1, 0);
    gsl_vector_const_view y1 = gsl_matrix_const_row (xy1, 1);
    gsl_vector_const_view x2 = gsl_matrix_const_row (xy2, 0);
    gsl_vector_const_view y2 = gsl_matrix_const_row (xy2, 1);

    size_t N = xy1->size2;
    gsl_matrix *D = gsl_matrix_alloc (N, 9);

    // column 1 = xj.*xi
    gsl_vector_view D1 = gsl_matrix_column (D,0);
    gsl_vector_memcpy (&D1.vector, &x2.vector);
    gsl_vector_mul (&D1.vector, &x1.vector);

    // column 2 = xj.*yi
    gsl_vector_view D2 = gsl_matrix_column (D,1);
    gsl_vector_memcpy (&D2.vector, &x2.vector);
    gsl_vector_mul (&D2.vector, &y1.vector);

    // column 3 = xj
    gsl_vector_view D3 = gsl_matrix_column (D,2);
    gsl_vector_memcpy (&D3.vector, &x2.vector);

    // column 4 = yj.*xi
    gsl_vector_view D4 = gsl_matrix_column (D,3);
    gsl_vector_memcpy (&D4.vector, &y2.vector);
    gsl_vector_mul (&D4.vector, &x1.vector);

    // column 5 = yj.*yi
    gsl_vector_view D5 = gsl_matrix_column (D,4);
    gsl_vector_memcpy (&D5.vector, &y2.vector);
    gsl_vector_mul (&D5.vector, &y1.vector);

    // column 6 = yj
    gsl_vector_view D6 = gsl_matrix_column (D,5);
    gsl_vector_memcpy (&D6.vector, &y2.vector);

    // column 7 = xi
    gsl_vector_view D7 = gsl_matrix_column (D,6);
    gsl_vector_memcpy (&D7.vector, &x1.vector);

    // column 8 = yi
    gsl_vector_view D8 = gsl_matrix_column (D,7);
    gsl_vector_memcpy (&D8.vector, &y1.vector);

    // column 9 = ones(n,1)
    gsl_vector_view D9 = gsl_matrix_column (D,8);
    gsl_vector_set_all (&D9.vector, 1.0);

    return D;
}

/* Construct demazure_constraints matrix from Ea, Eb, Ec using Matlab symb
 */
gsl_matrix *
_demazure_constraints_triggs_alloc (gsl_matrix *Ea, gsl_matrix *Eb, gsl_matrix *Ec)
{
    GSLU_MATRIX_VIEW (Eac, 3,3);
    gsl_matrix_memcpy (&Eac.matrix, Ea);
    gsl_matrix_sub (&Eac.matrix, Ec);       // Ex = Ea-Ec, Ey = Eb, Ez = Ec

    // pointers
    gsl_matrix *Ex = &Eac.matrix;
    gsl_matrix *Ey = Eb;
    gsl_matrix *Ez = Ec;

    GSLU_MATRIX_VIEW (I, 3, 3, {1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0
                               });

    GSLU_MATRIX_VIEW (WORK, 3, 3);
    GSLU_MATRIX_VIEW (TRANS_TEMP, 3, 3);

    // Construct matrices
    gsl_matrix *Nxx = gslu_blas_mmT_alloc (Ex, Ex);
    gsl_matrix_memcpy (&WORK.matrix, &I.matrix);
    gsl_matrix_scale (&WORK.matrix, 0.5*gslu_matrix_trace (Nxx));
    gsl_matrix_sub (Nxx, &WORK.matrix);

    gsl_matrix *Nyy = gslu_blas_mmT_alloc (Ey, Ey);
    gsl_matrix_memcpy (&WORK.matrix, &I.matrix);
    gsl_matrix_scale (&WORK.matrix, 0.5*gslu_matrix_trace (Nyy));
    gsl_matrix_sub (Nyy, &WORK.matrix);

    gsl_matrix *Nzz = gslu_blas_mmT_alloc (Ez, Ez);
    gsl_matrix_memcpy (&WORK.matrix, &I.matrix);
    gsl_matrix_scale (&WORK.matrix, 0.5*gslu_matrix_trace (Nzz));
    gsl_matrix_sub (Nzz, &WORK.matrix);

    gsl_matrix *Nxy = gslu_blas_mmT_alloc (Ex, Ey);
    gsl_matrix_memcpy (&WORK.matrix, &I.matrix);
    gsl_matrix_scale (&WORK.matrix, gslu_matrix_trace (Nxy));
    gsl_matrix_transpose_memcpy (&TRANS_TEMP.matrix, Nxy);
    gsl_matrix_add (Nxy, &TRANS_TEMP.matrix);
    gsl_matrix_sub (Nxy, &WORK.matrix);

    gsl_matrix *Nxz = gslu_blas_mmT_alloc (Ex, Ez);
    gsl_matrix_memcpy (&WORK.matrix, &I.matrix);
    gsl_matrix_scale (&WORK.matrix, gslu_matrix_trace (Nxz));
    gsl_matrix_transpose_memcpy (&TRANS_TEMP.matrix, Nxz);
    gsl_matrix_add (Nxz, &TRANS_TEMP.matrix);
    gsl_matrix_sub (Nxz, &WORK.matrix);

    gsl_matrix *Nyz = gslu_blas_mmT_alloc (Ey, Ez);
    gsl_matrix_memcpy (&WORK.matrix, &I.matrix);
    gsl_matrix_scale (&WORK.matrix, gslu_matrix_trace (Nyz));
    gsl_matrix_transpose_memcpy (&TRANS_TEMP.matrix, Nyz);
    gsl_matrix_add (Nyz, &TRANS_TEMP.matrix);
    gsl_matrix_sub (Nyz, &WORK.matrix);

    gsl_matrix *Nxxx_mat = gslu_blas_mm_alloc (Nxx, Ex);
    gsl_matrix *Nyyy_mat = gslu_blas_mm_alloc (Nyy, Ey);
    gsl_matrix *Nzzz_mat = gslu_blas_mm_alloc (Nzz, Ez);

    // Nxxy = Nxx*Ey+Nxy*Ex;
    gsl_matrix *Nxxy_mat = gslu_blas_mm_alloc (Nxx, Ey);
    gslu_blas_mm (&WORK.matrix, Nxy, Ex);
    gsl_matrix_add (Nxxy_mat, &WORK.matrix);

    // Nxxz = Nxx*Ez+Nxz*Ex;
    gsl_matrix *Nxxz_mat = gslu_blas_mm_alloc (Nxx, Ez);
    gslu_blas_mm (&WORK.matrix, Nxz, Ex);
    gsl_matrix_add (Nxxz_mat, &WORK.matrix);

    // Nxyy = Nyy*Ex+Nxy*Ey;
    gsl_matrix *Nxyy_mat = gslu_blas_mm_alloc (Nyy, Ex);
    gslu_blas_mm (&WORK.matrix, Nxy, Ey);
    gsl_matrix_add (Nxyy_mat, &WORK.matrix);

    // Nxzz = Nzz*Ex+Nxz*Ez;
    gsl_matrix *Nxzz_mat = gslu_blas_mm_alloc (Nzz, Ex);
    gslu_blas_mm (&WORK.matrix, Nxz, Ez);
    gsl_matrix_add (Nxzz_mat, &WORK.matrix);

    // Nyyz = Nyy*Ez+Nyz*Ey;
    gsl_matrix *Nyyz_mat = gslu_blas_mm_alloc (Nyy, Ez);
    gslu_blas_mm (&WORK.matrix, Nyz, Ey);
    gsl_matrix_add (Nyyz_mat, &WORK.matrix);

    // Nyzz = Nzz*Ey+Nyz*Ez;
    gsl_matrix *Nyzz_mat = gslu_blas_mm_alloc (Nzz, Ey);
    gslu_blas_mm (&WORK.matrix, Nyz, Ez);
    gsl_matrix_add (Nyzz_mat, &WORK.matrix);

    // Nxyz = Nxy*Ez+Nyz*Ex+Nxz*Ey;
    gsl_matrix *Nxyz_mat = gslu_blas_mm_alloc (Nxy, Ez);
    gslu_blas_mm (&WORK.matrix, Nyz, Ex);
    gsl_matrix_add (Nxyz_mat, &WORK.matrix);
    gslu_blas_mm (&WORK.matrix, Nxz, Ey);
    gsl_matrix_add (Nxyz_mat, &WORK.matrix);

    // allocation N matrix to return
    // N = reshape([Nxxx,Nxxy,Nxxz,Nxyy,Nxyz,Nxzz,Nyyy,Nyyz,Nyzz,Nzzz],9,10);
    gsl_matrix *N = gsl_matrix_alloc (9, 10);
    gsl_vector_view Nxxx_vec = gsl_matrix_column (N, 0);
    gslu_matrix_stack (&Nxxx_vec.vector, Nxxx_mat, CblasNoTrans);
    gsl_vector_view Nxxy_vec = gsl_matrix_column (N, 1);
    gslu_matrix_stack (&Nxxy_vec.vector, Nxxy_mat, CblasNoTrans);
    gsl_vector_view Nxxz_vec = gsl_matrix_column (N, 2);
    gslu_matrix_stack (&Nxxz_vec.vector, Nxxz_mat, CblasNoTrans);
    gsl_vector_view Nxyy_vec = gsl_matrix_column (N, 3);
    gslu_matrix_stack (&Nxyy_vec.vector, Nxyy_mat, CblasNoTrans);
    gsl_vector_view Nxyz_vec = gsl_matrix_column (N, 4);
    gslu_matrix_stack (&Nxyz_vec.vector, Nxyz_mat, CblasNoTrans);
    gsl_vector_view Nxzz_vec = gsl_matrix_column (N, 5);
    gslu_matrix_stack (&Nxzz_vec.vector, Nxzz_mat, CblasNoTrans);
    gsl_vector_view Nyyy_vec = gsl_matrix_column (N, 6);
    gslu_matrix_stack (&Nyyy_vec.vector, Nyyy_mat, CblasNoTrans);
    gsl_vector_view Nyyz_vec = gsl_matrix_column (N, 7);
    gslu_matrix_stack (&Nyyz_vec.vector, Nyyz_mat, CblasNoTrans);
    gsl_vector_view Nyzz_vec = gsl_matrix_column (N, 8);
    gslu_matrix_stack (&Nyzz_vec.vector, Nyzz_mat, CblasNoTrans);
    gsl_vector_view Nzzz_vec = gsl_matrix_column (N, 9);
    gslu_matrix_stack (&Nzzz_vec.vector, Nzzz_mat, CblasNoTrans);

    // clean up
    gslu_matrix_free (Nxx);
    gslu_matrix_free (Nyy);
    gslu_matrix_free (Nzz);
    gslu_matrix_free (Nxy);
    gslu_matrix_free (Nxz);
    gslu_matrix_free (Nyz);
    gslu_matrix_free (Nxxx_mat);
    gslu_matrix_free (Nxxy_mat);
    gslu_matrix_free (Nxxz_mat);
    gslu_matrix_free (Nxyy_mat);
    gslu_matrix_free (Nxyz_mat);
    gslu_matrix_free (Nxzz_mat);
    gslu_matrix_free (Nyyy_mat);
    gslu_matrix_free (Nyyz_mat);
    gslu_matrix_free (Nyzz_mat);
    gslu_matrix_free (Nzzz_mat);

    return N;
}

size_t
vis_modelfit_weight_6p (const gsl_matrix *A, gsl_vector *xroots, gsl_vector *yroots)
{
    size_t n_realroot = 0;
    assert (A->size1 == 9 && A->size2 == 10); // 9 X 10 N matrix

    gslu_linalg_SV *SVD = gslu_linalg_SV_decomp_full_alloc (A);

    if (SVD)
    {
        gsl_matrix_view Ast = gsl_matrix_submatrix (SVD->V, 0, 0, 10, 4);
        gsl_matrix *As = gslu_matrix_transpose_alloc (&Ast.matrix);

        gslu_linalg_rref (As, 4); // Ar = rref2(As,4);

        // last 3 equations x^2*y x^2 x*y^2 x*y x y^3 y^2 y 1 B = Ar(2:4,2:10);
        gsl_matrix_view B = gsl_matrix_submatrix (As, 1, 1, 3, 9);

        // expand to x^2*y x^2 x*y^2 x*y x y^6 y^5 y^4 y^3 y^2 y 1
        gsl_matrix *C = gsl_matrix_alloc (3, 12);
        gsl_matrix_set_zero (C);
        gsl_matrix_view C_sub = gsl_matrix_submatrix (C, 0, 0, 3, 5);
        gsl_matrix_view B_sub = gsl_matrix_submatrix (&B.matrix, 0, 0, 3, 5);
        gsl_matrix_memcpy (&C_sub.matrix, &B_sub.matrix);
        C_sub = gsl_matrix_submatrix (C, 0, 8, 3, 4);
        B_sub = gsl_matrix_submatrix (&B.matrix, 0, 5, 3, 4);
        gsl_matrix_memcpy (&C_sub.matrix, &B_sub.matrix);

        // eliminate x^2*y term of first row by shifting second row and substracting
        // keep rows one and three x*y^2 x*y x y^6 y^5 y^4 y^3 y^2 y 1
        gsl_matrix_view D = gsl_matrix_submatrix (C, 0, 2, 2, 10);
        gsl_matrix *M = &D.matrix; // just a pointer no mem alloc !

        for (size_t j=2; j<12; j++)
        {
            if (j<11)
                gsl_matrix_set (M, 0, j-2, gsl_matrix_get (C, 0, j) - gsl_matrix_get (C, 1, j+1));
            gsl_matrix_set (M, 1, j-2, gsl_matrix_get (C, 2, j));
        }

        // make first equation a function of x*y^2  0   x y^6 y^5 y^4 y^3 y^2 y 1
        // and second equation a function of   0   x*y  x y^6 y^5 y^4 y^3 y^2 y 1
        gslu_linalg_rref (M, 0);

        // eliminate x*y^2 term by shifting and substracting second equation
        for (size_t j=0; j<9; j++)
            gsl_matrix_set (M, 0, j, gsl_matrix_get (M, 0, j) - gsl_matrix_get (M, 1, j+1));


        // make first equation a function of x^y 0 y^6 y^5 y^4 y^3 y^2 y 1
        // and second equation a function of  0  x y^6 y^5 y^4 y^3 y^2 y 1
        gslu_linalg_rref (M, 0);

        // elimante x*y term and x term by shifting and substracting second equation
        for (size_t j=0; j<9; j++)
            gsl_matrix_set (M, 0, j, gsl_matrix_get (M, 0, j) - gsl_matrix_get (M, 1, j+1));

#ifdef MODELFIT_VERBOSE
        gslu_matrix_printf (M,"F");
#endif

        // extract polynomial of order 6 from first row and solve for roots
        gsl_poly_complex_workspace * w = gsl_poly_complex_workspace_alloc (7);

        double ypoly[7];
        double yroots_data[12]; // (7-1)*2

        for (size_t i=0; i<7; i++)
            ypoly[6-i] = gsl_matrix_get (M, 0, i+3);

        gsl_poly_complex_solve (ypoly, 7, w, yroots_data);
        gsl_poly_complex_workspace_free (w);

        // store y roots and to return
        for (size_t i=0; i<6; i++)
        {
            if (fabs(yroots_data[2*i+1]) < GSL_DBL_EPSILON)
            {
                gsl_vector_set (yroots, n_realroot, yroots_data[2*i]);
                n_realroot ++;
            }
            // printf ("root = %g\n", yroots_data[2*i]);
        }

        // equation 2 determines x as a function of y. xpoly = F(2,5:10);
        double xpoly[6];

        for (size_t i=0; i<6; i++)
            xpoly[5-i] = gsl_matrix_get (M, 1, i+4);

        // store x roots and to return
        for (size_t i=0; i<n_realroot; i++)
            gsl_vector_set (xroots, i, -gsl_poly_eval (xpoly, 6, gsl_vector_get (yroots, i)));


        gslu_matrix_free (As);
        gslu_matrix_free (C);
    }
    else
        printf ("ERROR in weight_6p svd\n");

    if (SVD)
        gslu_linalg_SV_free (SVD);

    return n_realroot;
}

size_t
vis_modelfit_E_6p (const gsl_matrix *D, gsl_matrix *E_set)
{
    assert (D->size1 == VIS_MODELFIT_E_RANSAC_SAMPLE_SIZE && D->size2 == 9); // nsamp X 9

    // Do SVD
    gslu_linalg_SV *SVD = gslu_linalg_SV_decomp_full_alloc (D);

    size_t n_realroot = 0;

    if (SVD)
    {
        //gslu_matrix_printf (SVD->U, "U");
        //gslu_matrix_printf (SVD->V, "V");
        //gslu_vector_printf (SVD->S, "S");
        gsl_vector_view ea = gsl_matrix_column (SVD->V, 6);
        gsl_vector_view eb = gsl_matrix_column (SVD->V, 7);
        gsl_vector_view ec = gsl_matrix_column (SVD->V, 8);
        GSLU_MATRIX_VIEW (Ea, 3, 3);
        GSLU_MATRIX_VIEW (Eb, 3, 3);
        GSLU_MATRIX_VIEW (Ec, 3, 3);
        gslu_vector_reshape (&Ea.matrix, &ea.vector, CblasTrans);
        gslu_vector_reshape (&Eb.matrix, &eb.vector, CblasTrans);
        gslu_vector_reshape (&Ec.matrix, &ec.vector, CblasTrans);

        gsl_matrix *N = _demazure_constraints_triggs_alloc (&Ea.matrix, &Eb.matrix, &Ec.matrix);

#ifdef MODELFIT_VERBOSE
        gslu_matrix_printf (N,"N");
#endif

        GSLU_VECTOR_VIEW (xroots, 6);
        GSLU_VECTOR_VIEW (yroots, 6);

        n_realroot = vis_modelfit_weight_6p (N, &xroots.vector, &yroots.vector);

        GSLU_MATRIX_VIEW (E, 3, 3);
        GSLU_MATRIX_VIEW (work, 3, 3);

        // the above solution for alpha and beta can have up to six roots
        for (size_t k=0; k<n_realroot; k++)
        {
            //E = alpha(k)*(Ea-Ec) + beta(k)*Eb + Ec;
            gsl_matrix_memcpy (&E.matrix, &Ec.matrix);
            gsl_matrix_memcpy (&work.matrix, &Ea.matrix);
            gsl_matrix_sub (&work.matrix, &Ec.matrix);
            double alpha = gsl_vector_get (&xroots.vector, k);
            double beta = gsl_vector_get (&yroots.vector, k);

            gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, alpha, &work.matrix, &Eb.matrix, beta, &E.matrix);
            gsl_vector_view E_set_col = gsl_matrix_column (E_set, k);
            gslu_matrix_stack (&E_set_col.vector, &E.matrix, CblasNoTrans);
        }
    }
    else
        printf ("ERROR in estim_E_svd\n");

    if (SVD)
        gslu_linalg_SV_free (SVD);

    return n_realroot;
}


double
vis_modelfit_E_ransac (const gsl_matrix *xy1, const gsl_matrix *xy2,
                       gsl_matrix *E, gslu_index **isel, gsl_vector **error)
{
    // check the input size first
    assert (xy1->size1 == 2 && xy2->size1 >= 2 && E->size1 == 3 && E->size2 == 3);

    // set parameters: threshold, iteration min/max
    const double thresh = 1.0E-5;           // value copied from matlab
    const double percent_inliers = 0.5;
    const double p = 0.999;                 // confidence level of picking an inlier set
    const size_t n_total  = xy1->size2;     // size of the set = N
    size_t itr_min = 50;
    size_t itr_max = ransac_adapt_trials (percent_inliers*n_total, n_total, p, VIS_MODELFIT_E_RANSAC_SAMPLE_SIZE);

    size_t n_inlier = 0;

    size_t *sel = malloc (n_total * sizeof (*sel));
    size_t *sel_in = malloc (n_total * sizeof (*sel_in));
    gsl_vector *error_i = gsl_vector_alloc (n_total);
    if (error)
        *error = gsl_vector_alloc (n_total);

    // prepare measurement matrix for E
    gsl_matrix *D = _F_meas_matrix_alloc (xy1, xy2);

    GSLU_INDEX_VIEW (rsel, 6);
    GSLU_MATRIX_VIEW (D_samples, 6, 9);
    gsl_rng *r = gslu_rand_rng_alloc ();
    GSLU_MATRIX_VIEW (E_set, 9, 6);
    GSLU_MATRIX_VIEW (E_temp, 3, 3);

    for (size_t itr=0; (itr < itr_min) || (itr < itr_max); itr++)
    {
        // draw samples
        gslu_rand_index (r, &rsel.vector, n_total);
        gslu_matrix_selrow (&D_samples.matrix, D, &rsel.vector);

        // fit essential matrix model
        size_t n_E = vis_modelfit_E_6p (&D_samples.matrix, &E_set.matrix);

        for (size_t k=0; k<n_E; k++)
        {
            gsl_vector_view E_col = gsl_matrix_column (&E_set.matrix, k);
            gslu_vector_reshape (&E_temp.matrix, &E_col.vector, CblasNoTrans);


            // sampson error
            //error_i = _modelfit_reprojection_error_F (xy1, xy2, &E_temp.matrix);
            error_i = _modelfit_sampson_error_F (xy1, xy2, D, &E_temp.matrix);

            size_t n_in = 0;
            for (size_t i=0; i<n_total; i++)
            {
                if (fabs(gsl_vector_get(error_i,i)) < thresh)
                {
                    //printf ("found smaller %g\b", fabs(gsl_vector_get(error_i,i)));
                    sel_in[n_in++] = i;
                }
            }

            // check for consensus
            if (n_in > n_inlier)
            {
                n_inlier = n_in;
                gsl_matrix_memcpy (E, &E_temp.matrix);
                itr_max = GSL_MIN (itr_max, ransac_adapt_trials (n_inlier, n_total, p, VIS_MODELFIT_H_RANSAC_SAMPLE_SIZE));

                // only stuff if the user requests the following outputs
                if (error)
                    gsl_vector_memcpy (*error, error_i);

                if (isel)
                {
                    for (size_t i=0; i<n_inlier; i++)
                        sel[i] = sel_in[i];
                }
            }
        } // for each E candidate (upto 6 E)
    }

    // assign inlier set only if user requests it
    if (isel && n_inlier)
    {
        *isel = gslu_index_alloc (n_inlier);
        for (size_t i=0; i<n_inlier; i++)
            gslu_index_set (*isel, i, sel[i]);
    }

    // clean up
    gslu_matrix_free (D);
    gsl_rng_free (r);
    gsl_vector_free (error_i);
    free (sel);
    free (sel_in);

    return n_inlier;
}

double
vis_modelfit_F_ocv (const gsl_matrix *uv1_gsl, const gsl_matrix *uv2_gsl,
                    gsl_matrix *F_gsl, gslu_index **sel,
                    vis_modelfit_method_t method)
{
    size_t N = uv1_gsl->size2;

    if (N < 6)
    {
        ERROR ("need more than 6 points.");
        return 0;
    }

    // determine method
    int cv_method;
    if(method == VIS_MODELFIT_RANSAC)
        cv_method = CV_FM_RANSAC;
    else
        cv_method = CV_FM_LMEDS;

    // convert to cvmat type
    CvMat uv1_cv = vis_cvu_gsl_matrix_to_cvmat_view (uv1_gsl);
    CvMat uv2_cv = vis_cvu_gsl_matrix_to_cvmat_view (uv2_gsl);
    CvMat F_cv = vis_cvu_gsl_matrix_to_cvmat_view (F_gsl);

    /* estim_F_RANSAC
     * cvFindFundamentalMat(const CvMat* points1, const CvMat* points2, CvMat* fundamentalMatrix,
     *                      int method=CV_FM_RANSAC, double param1=1., double param2=0.99, CvMat* status=NULL)
     */
    double param1 = 0.1;    // maximum pixel distance from point to epipolar line
    double param2 = 0.99; // desirable level of confidense the matrix is the correct
    CvMat *inliers_cv = cvCreateMat (1, N, CV_8U);
    int fm_count = cvFindFundamentalMat (&uv1_cv, &uv2_cv, &F_cv, cv_method, param1, param2, inliers_cv);

    size_t n_inliers = 0;
    if (fm_count)
    {
        // find indices of inlier uv1 and uv2
        gslu_index *sel_tmp = gslu_index_alloc (N);
        for (size_t i=0; i<N; i++)
        {
            if (cvGetReal2D (inliers_cv, 0, i) > 0)
                gslu_index_set (sel_tmp, n_inliers++, i);
        }
        (*sel) = gslu_index_alloc (n_inliers);
        gslu_index_view sel_tmp_in = gslu_index_subvector (sel_tmp, 0, n_inliers);
        gslu_index_memcpy (*sel, &sel_tmp_in.vector);
        gslu_index_free (sel_tmp);
    }

    //clean up
    cvReleaseMat (&inliers_cv);

    return n_inliers;
}


gsl_matrix *
_H_meas_matrix_alloc (const gsl_matrix *xy1, const gsl_matrix *xy2)
{
    size_t n = xy1->size2;

    //D = [xi, yi, ones(n,1),                    zeros(n,1), zeros(n,1), zeros(n,1),  -xi.*xj, -yi.*xj, -xj;
    //     zeros(n,1), zeros(n,1), zeros(n,1),   xi, yi, ones(n,1),                   -xi.*yj, -yi.*yj, -yj];

    gsl_vector_const_view x1 = gsl_matrix_const_row (xy1, 0);
    gsl_vector_const_view y1 = gsl_matrix_const_row (xy1, 1);
    gsl_vector_const_view x2 = gsl_matrix_const_row (xy2, 0);
    gsl_vector_const_view y2 = gsl_matrix_const_row (xy2, 1);

    gsl_matrix *D = gsl_matrix_alloc (2*n,9);
    gsl_matrix_set_zero (D);

    gsl_matrix_view D_up = gsl_matrix_submatrix (D, 0, 0, n, 9);
    gsl_matrix_view D_down = gsl_matrix_submatrix (D, n, 0, n, 9);

    // column1
    gsl_vector_view D_up1 = gsl_matrix_column (&D_up.matrix, 0);
    gsl_vector_memcpy (&D_up1.vector, &x1.vector);

    // column2
    gsl_vector_view D_up2 = gsl_matrix_column (&D_up.matrix, 1);
    gsl_vector_memcpy (&D_up2.vector, &y1.vector);

    // column3
    gsl_vector_view D_up3 = gsl_matrix_column (&D_up.matrix, 2);
    gsl_vector_set_all (&D_up3.vector, 1.0);

    // column4
    gsl_vector_view D_down4 = gsl_matrix_column (&D_down.matrix, 3);
    gsl_vector_memcpy (&D_down4.vector, &x1.vector);

    // column5
    gsl_vector_view D_down5 = gsl_matrix_column (&D_down.matrix, 4);
    gsl_vector_memcpy (&D_down5.vector, &y1.vector);

    // column6
    gsl_vector_view D_down6 = gsl_matrix_column (&D_down.matrix, 5);
    gsl_vector_set_all (&D_down6.vector, 1.0);

    // column7
    gsl_vector_view D_up7 = gsl_matrix_column (&D_up.matrix, 6);
    gsl_vector_memcpy (&D_up7.vector, &x1.vector);
    gsl_vector_scale (&D_up7.vector, -1.0);
    gsl_vector_mul (&D_up7.vector, &x2.vector);
    gsl_vector_view D_down7 = gsl_matrix_column (&D_down.matrix, 6);
    gsl_vector_memcpy (&D_down7.vector, &x1.vector);
    gsl_vector_scale (&D_down7.vector, -1.0);
    gsl_vector_mul (&D_down7.vector, &y2.vector);

    // column8
    gsl_vector_view D_up8 = gsl_matrix_column (&D_up.matrix, 7);
    gsl_vector_memcpy (&D_up8.vector, &y1.vector);
    gsl_vector_scale (&D_up8.vector, -1.0);
    gsl_vector_mul (&D_up8.vector, &x2.vector);
    gsl_vector_view D_down8 = gsl_matrix_column (&D_down.matrix, 7);
    gsl_vector_memcpy (&D_down8.vector, &y1.vector);
    gsl_vector_scale (&D_down8.vector, -1.0);
    gsl_vector_mul (&D_down8.vector, &y2.vector);

    // column9
    gsl_vector_view D_up9 = gsl_matrix_column (&D_up.matrix, 8);
    gsl_vector_memcpy (&D_up9.vector, &x2.vector);
    gsl_vector_scale (&D_up9.vector, -1.0);
    gsl_vector_view D_down9 = gsl_matrix_column (&D_down.matrix, 8);
    gsl_vector_memcpy (&D_down9.vector, &y2.vector);
    gsl_vector_scale (&D_down9.vector, -1.0);

    return D;
}

/**
 * @brief Calculate H based upon DLT point algorithm.
 * D is a [8x9] measurement matrix with rows of the form:
 * D = [xi, yi, 1,  0, 0, 0,  -xi*xj, -yi*xj, -xj;
 *      0, 0, 0,   xi, yi, 1, -xi*yj, -yi*yj, -yj];
 *
 */
void
vis_modelfit_H_svd (const gsl_matrix *D, gsl_matrix *H)
{
    assert (D->size1 == 2*VIS_MODELFIT_H_RANSAC_SAMPLE_SIZE && D->size2 == 9); // 2*nsamp X 9

    gslu_linalg_SV *SVD = gslu_linalg_SV_decomp_econ_alloc (D);

    if (SVD)
    {
        gsl_vector_view v = gsl_matrix_column (SVD->V, 8);
        gslu_vector_reshape (H, &v.vector, CblasTrans);
        gsl_matrix_scale (H, 1.0/gsl_matrix_get(H,2,2));
    }
    else
    {
        printf ("ERROR in estim_H_svd\n");
#ifdef MODELFIT_VERBOSE
        gslu_matrix_printf (D, "D");
#endif
    }
    gslu_linalg_SV_free (SVD);
}

double
vis_modelfit_H_ransac (const gsl_matrix *xy1, const gsl_matrix *xy2,
                       gsl_matrix *H, gslu_index **isel, gsl_vector **error)
{
    // check the input size first
    assert (xy1->size1 == 2 && xy2->size1 >= 2 && H->size1 == 3 && H->size2 == 3);

    // set parameters: threshold, iteration min/max
    const double thresh = 1.0E-5;           // value copied from matlab
    const double percent_inliers = 0.5;
    const double p = 0.999;                 // confidence level of picking an inlier set
    const size_t n_total  = xy1->size2;     // size of the set = N
    size_t itr_min = 50;
    size_t itr_max = ransac_adapt_trials (percent_inliers*n_total, n_total, p, VIS_MODELFIT_H_RANSAC_SAMPLE_SIZE);

    size_t n_inlier = 0;

    size_t *sel = malloc (n_total * sizeof (*sel));
    size_t *sel_in = malloc (n_total * sizeof (*sel_in));
    gsl_vector *error_i = gsl_vector_alloc (n_total);
    if (error)
        *error = gsl_vector_alloc (n_total);

    // prepare measurement matrix for E
    gsl_matrix *D = _H_meas_matrix_alloc (xy1, xy2);

    GSLU_INDEX_VIEW (rsel, 2*VIS_MODELFIT_H_RANSAC_SAMPLE_SIZE);
    GSLU_MATRIX_VIEW (D_samples, 2*VIS_MODELFIT_H_RANSAC_SAMPLE_SIZE, 9);
    GSLU_MATRIX_VIEW (H_temp, 3, 3);

    gsl_rng *r = gslu_rand_rng_alloc ();

    for (size_t itr=0; (itr < itr_min) || (itr < itr_max); itr++)
    {
        // draw samples
        gslu_rand_index (r, &rsel.vector, n_total);
        for (size_t i=0; i<VIS_MODELFIT_H_RANSAC_SAMPLE_SIZE; i++)
            gslu_index_set (&rsel.vector, i+VIS_MODELFIT_H_RANSAC_SAMPLE_SIZE, gslu_index_get (&rsel.vector,i)+n_total);

        gslu_matrix_selrow (&D_samples.matrix, D, &rsel.vector);
        //gslu_index_printf (&rsel.vector, "rsel");

        // fit homography model
        vis_modelfit_H_svd (&D_samples.matrix, &H_temp.matrix);
        //gslu_matrix_printf (&H_temp.matrix, "H");

        // geometric error
        size_t n_in = 0;
        error_i = _modelfit_reprojection_error_H (xy1, xy2, &H_temp.matrix);

        for (size_t i=0; i<n_total; i++)
        {
            if (fabs(gsl_vector_get(error_i,i)) < thresh)
            {
                //printf ("found smaller %g\b", fabs(gsl_vector_get(error_i,i)));
                sel_in[n_in++] = i;
            }
        }

        // check for consensus
        if (n_in > n_inlier)
        {
            n_inlier = n_in;
            gsl_matrix_memcpy (H, &H_temp.matrix);
            itr_max = GSL_MIN (itr_max, ransac_adapt_trials (n_inlier, n_total, p, VIS_MODELFIT_H_RANSAC_SAMPLE_SIZE));

            // only stuff if the user requests the following outputs
            if (error)
                gsl_vector_memcpy (*error, error_i);

            if (isel)
            {
                for (size_t i=0; i<n_inlier; i++)
                    sel[i] = sel_in[i];
            }
        }
    }

    // assign inlier set only if user requests it
    if (isel && n_inlier)
    {
        *isel = gslu_index_alloc (n_inlier);
        for (size_t i=0; i<n_inlier; i++)
            gslu_index_set (*isel, i, sel[i]);
    }

    // clean up
    gslu_matrix_free (D);
    gsl_rng_free (r);
    gsl_vector_free (error_i);
    free (sel);
    free (sel_in);

    return n_inlier;
}

double
vis_modelfit_H_ocv (const gsl_matrix *uv1_gsl, const gsl_matrix *uv2_gsl,
                    gsl_matrix *H_gsl,
                    gslu_index **sel,
                    vis_modelfit_method_t method)
{

    int N = uv1_gsl->size2;        // uv = [2xN]

    if (N < 4)
    {
        ERROR ("need more than 4 points.");
        return 0;
    }

    // determine method
    int H_method;
    if (method == VIS_MODELFIT_RANSAC)
        H_method = CV_FM_RANSAC;
    else
        H_method = CV_FM_LMEDS;

    // copy points to cvmat type
    CvMat uv1_cv = vis_cvu_gsl_matrix_to_cvmat_view (uv1_gsl);
    CvMat uv2_cv = vis_cvu_gsl_matrix_to_cvmat_view (uv2_gsl);
    CvMat H_cv = vis_cvu_gsl_matrix_to_cvmat_view (H_gsl);

    /* estim_H_RANSAC
     * cvFindHomography(const CvMat* srcPoints, const CvMat* dstPoints, CvMat* H
     *                  int method=0, double ransacReprojThreshold=0, CvMat* status=NULL)
     */
    double ransacReprojThreshold= 1.0;            // RANSAC threshold
    CvMat* inliers_h_cv = cvCreateMat (1,N,CV_8U);
    cvFindHomography (&uv1_cv, &uv2_cv, &H_cv, H_method, ransacReprojThreshold, inliers_h_cv);


    // check if H is zero matrix
    int h_valid = 0;
    for (size_t i=0; i<3; i++)
        for (size_t j=0; j<3; j++)
            h_valid = h_valid + gsl_matrix_get (H_gsl, i,j);

    size_t n_inliers = 0;
    if (h_valid)
    {
        gslu_index *sel_tmp = gslu_index_alloc (N);
        for (size_t i=0; i<N; i++)
        {
            if (cvGetReal2D (inliers_h_cv, 0, i) > 0)
                gslu_index_set (sel_tmp, n_inliers++, i);
        }

        // check number of inliers
        if (n_inliers > 4)
        {
            (*sel) = gslu_index_alloc (n_inliers);
            gslu_index_view sel_tmp_in = gslu_index_subvector (sel_tmp, 0, n_inliers);
            gslu_index_memcpy (*sel, &sel_tmp_in.vector);
        }

        // clean up
        gslu_index_free (sel_tmp);
    }
    cvReleaseMat (&inliers_h_cv);

    return n_inliers;
}

/**
 * @brief Fit a 3D model (E or F) to find inliers, model, and gic score.
 *
 * @param uvi_sel
 *
 * @return gic score
 */
double
vis_modelfit_inliers_gic_3D (gsl_matrix *uvi_sel, gsl_matrix *uvj_sel,
                             gslu_index **sel_f, gsl_matrix **uvi_f, gsl_matrix **uvj_f,
                             gsl_matrix *F_or_E,
                             vis_modelfit_method_t method)
{
    // set the default value
    double lambda1 = VIS_MODELFIT_TORR_LAMBDA1;
    double lambda2 = VIS_MODELFIT_TORR_LAMBDA2;

    double gic_f = GSL_POSINF;
    int n_in_f = 0;

    if (method == VIS_MODELFIT_OCV_RANSAC || method == VIS_MODELFIT_OCV_LMEDS)
        n_in_f = vis_modelfit_F_ocv (uvi_sel, uvj_sel, F_or_E, sel_f, method);
    else if (method == VIS_MODELFIT_RANSAC)
        n_in_f = vis_modelfit_E_ransac (uvi_sel, uvj_sel, F_or_E, sel_f, NULL);
    else
        printf ("ERROR (model fit) Unknown method");

    if (n_in_f > 0)
    {
        (*uvi_f) = gslu_matrix_selcol_alloc (uvi_sel, (*sel_f));
        (*uvj_f) = gslu_matrix_selcol_alloc (uvj_sel, (*sel_f));
        gic_f = vis_modelfit_gic (F_or_E, VIS_MODELFIT_F, uvi_sel, uvj_sel, lambda1, lambda2);
    }

    return gic_f;
}

double
vis_modelfit_inliers_gic_2D (gsl_matrix *uvi_sel, gsl_matrix *uvj_sel,
                             gslu_index **sel_h, gsl_matrix **uvi_h, gsl_matrix **uvj_h,
                             gsl_matrix *H,
                             vis_modelfit_method_t method)
{
    // set the default value
    double lambda1 = VIS_MODELFIT_TORR_LAMBDA1;
    double lambda2 = VIS_MODELFIT_TORR_LAMBDA2;

    double gic_h = GSL_POSINF;
    int n_in_h = 0;

    if (method == VIS_MODELFIT_OCV_RANSAC || method == VIS_MODELFIT_OCV_LMEDS)
        n_in_h = vis_modelfit_H_ocv (uvi_sel, uvj_sel, H, sel_h, method);
    else if (method == VIS_MODELFIT_RANSAC)
        n_in_h = vis_modelfit_H_ransac (uvi_sel, uvj_sel, H, sel_h, NULL);
    else
        printf ("ERROR (model fit) Unknown method");

    if (n_in_h > 0)
    {
        (*uvi_h) = gslu_matrix_selcol_alloc (uvi_sel, (*sel_h));
        (*uvj_h) = gslu_matrix_selcol_alloc (uvj_sel, (*sel_h));
        n_in_h = (*sel_h)->size;
        gic_h = vis_modelfit_gic (H, VIS_MODELFIT_H, uvi_sel, uvj_sel, lambda1, lambda2);
    }

    return gic_h;
}

#define F77_FUNC(func)    func ## _

// SVD
extern int F77_FUNC(dgesvd)(char *jobu, char *jobvt, int *m, int *n,
                            double *a, int *lda, double *s, double *u, int *ldu,
                            double *vt, int *ldvt, double *work, int *lwork,
                            int *info);

// lapack 3.0 routine, faster than dgesvd()
extern int F77_FUNC(dgesdd)(char *jobz, int *m, int *n, double *a, int *lda,
                            double *s, double *u, int *ldu, double *vt, int *ldvt,
                            double *work, int *lwork, int *iwork, int *info);

gslu_linalg_SV *
_SV_decomp_full_alloc_fortran (const gsl_matrix *A)
{
    // this code has been ported and modified from sba_Axb_SVD
    if (A == NULL) return NULL;

    size_t m = A->size1;
    size_t n = A->size2;
    size_t min_dim = m < n ? m : n;
    size_t lda = m;             // The leading dimension of the array A.  LDA >= max(1,M)
    size_t ldu = m;             // The leading dimension of the array U.  LDU >= 1; if JOBU = 'S' or 'A', LDU >= M.
    size_t ldvt = n;            // The leading dimension of the array VT.  LDVT >= 1; if JOBVT = 'A', LDVT >= N; if JOBVT = 'S', LDVT >= min(M,N).

    // allocate memory for a, u, s, vt
    int a_sz = lda*n, u_sz = ldu*m, s_sz = min_dim, vt_sz = ldvt*n;
    double *a = malloc (a_sz*sizeof(double));   // (LDA, n)
    double *u = malloc (u_sz*sizeof(double));   // (LDU,M)  if  JOBU = 'A'
    double *s = malloc (s_sz*sizeof(double));   // dimension (min(M,N))
    double *vt = malloc (vt_sz*sizeof(double));  // (LDVT,N)
    double *work = NULL;

    // calculate required memory size
    // -------------------------------------------------------
    double wkopt;
    int info = 0, lwork = -1;
    F77_FUNC (dgesvd) ("A", "A", (int*) &m, (int*) &n, a, (int*) &lda, s, u, (int*) &ldu, vt, (int*) &ldvt, &wkopt, &lwork, &info);
    lwork = (int) wkopt;
    work = (double*) malloc (lwork * sizeof (double));

    if(!work)
    {
        printf ("ERROR: memory allocation in svd failed!\n");
        if (a) free(a);
        if (u) free(u);
        if (s) free(s);
        if (vt) free(vt);
        return NULL;
    }

    // gsl matrix is NOT column major. Convert!
    // TODO: better way to do this?
    // -------------------------------------------------------
    // store A (row major) into a (column major)
    for(size_t i=0; i<m; ++i)
        for(size_t j=0; j<n; ++j)
            a[i+j*m]= gsl_matrix_get (A, i, j);

    // SVD decomposition of A
    //F77_FUNC(dgesdd)("A", (int *)&m, (int *)&m, a, (int *)&m, s, u, (int *)&m, vt, (int *)&m, work, (int *)&worksz, iwork, &info);
    F77_FUNC (dgesvd) ("A", "A", (int*) &m, (int*) &n, a, (int*) &lda, s, u, (int*) &ldu, vt, (int*) &ldvt, work, &lwork, &info );

    // error treatment
    if (info!=0)
    {
        if (info<0)
        {
            fprintf (stderr, "LAPACK error: illegal value for argument %d of dgesdd/dgesvd in sba_Axb_SVD()\n", -info);
            return NULL;
        }
        else
        {
            fprintf (stderr, "LAPACK error: dgesdd (dbdsdc)/dgesvd (dbdsqr) failed to converge in sba_Axb_SVD() [info=%d]\n", info);

            return NULL;
        }
    }

    // prepare SVD struct to be returned
    gslu_linalg_SV *sv = (gslu_linalg_SV*) malloc (sizeof (gslu_linalg_SV));
    gsl_vector_view s_view = gsl_vector_view_array (s, s_sz);
    sv->S = gslu_vector_clone (&s_view.vector);
    sv->U = gsl_matrix_alloc (m, m);
    sv->V = gsl_matrix_alloc (n, n);

    for(size_t i=0; i<m; ++i)
        for(size_t j=0; j<m; ++j)
            gsl_matrix_set (sv->U, i, j, u[i+j*m]);

    for(size_t i=0; i<n; ++i)
        for(size_t j=0; j<n; ++j)
            gsl_matrix_set (sv->V, j, i, vt[i+j*n]);

    // clean up
    if (a) free (a);
    if (u) free (u);
    if (s) free (s);
    if (vt) free (vt);
    if (work) free (work);
    return sv;
}

