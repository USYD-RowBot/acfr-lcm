#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include <math.h>
#include <gsl/gsl_linalg.h>

#include "perls-common/timestamp.h"
#include "perls-common/units.h"

#include "perls-math/so3.h"
#include "perls-math/gsl_util.h"

int
main (int argc, char *argv[])
{

    /*
     * gsl_util_vector.h
     */
    // example of how to use gslu macros to create vector elements
    // on the stack with vector views.  x, and b each have subfields
    // .data and .vector.  The advantage is that elements created on the stack
    // do not have to be manually freed by the programmer, they automatically
    // have limited scope
    GSLU_VECTOR_VIEW (x, 4);
    // these are equivlent
    x.data[0] = 0;  gsl_vector_set (&x.vector, 0, 0);
    x.data[1] = 0;  gsl_vector_set (&x.vector, 1, 0);
    x.data[2] = 1;  gsl_vector_set (&x.vector, 2, 1);
    x.data[3] = 0;  gsl_vector_set (&x.vector, 3, 0);
    gslu_vector_printf (&x.vector, "x");

    // and here's another syntax that allows you to create and simultaneously initialize
    GSLU_VECTOR_VIEW (xx, 4, {0, 0, 1, 0});
    gslu_vector_printf (&xx.vector, "xx");


    // clone a vector
    gsl_vector *y = gslu_vector_clone (&x.vector);
    gslu_vector_printf (y, "y");

    // select a subset of a vector
    GSLU_INDEX_VIEW (isel, 2, {2, 3});
    gsl_vector *x_sub = gslu_vector_sel_alloc (&x.vector, &isel.vector);
    gslu_vector_printf (x_sub, "x_sub");

    // set a subvector
    gslu_vector_set_subvector (y, 0, x_sub);
    gslu_vector_printf (y, "y");


    // vector sum
    double sum = gslu_vector_sum (y);
    printf ("sum=%g\n", sum);

    // dist
    GSLU_VECTOR_VIEW (xi, 5, {1, 2, 3*M_PI, 4*M_PI, 5});
    GSLU_VECTOR_VIEW (yi, 5, {0, 0, 0, 0, 0});
    GSLU_MATRIX_VIEW (Si, 5, 5, 
                      { 1, 0,  0,  0, 0,
                        0, 1,  0,  0, 0,
                        0, 0,  1,  0, 0,
                        0, 0,  0,  1, 0,
                        0, 0,  0,  0, 1 });
    GSLU_INDEX_VIEW (ci, 2, {2, 3});
    double dist = gslu_vector_dist (&xi.vector, &yi.vector);
    double distc = gslu_vector_circ_dist (&xi.vector, &yi.vector, &ci.vector);
    printf ("dist=%g\tdistc=%g\n", dist, distc);

    // mahal dist
    double mahal_dist = gslu_vector_mahal_dist (&xi.vector, &yi.vector, &Si.matrix);
    double mahal_distc = gslu_vector_mahal_circ_dist (&xi.vector, &yi.vector, &Si.matrix, &ci.vector);
    printf ("\nmahal_dist = %g\tmahal_distc=%g\n", mahal_dist, mahal_distc);

    
    // cross
    {
        GSLU_VECTOR_VIEW (a, 3, {1, 2, 3});
        GSLU_VECTOR_VIEW (b, 3, {6, 5, 4});

        gsl_vector *c = gslu_vector_cross_alloc (&a.vector, &b.vector);
        gslu_vector_printf (c, "c");
        
        // scalar triple prod
        double stp = gslu_vector_scalar_triple_prod (&a.vector, &b.vector, c);
        printf ("\nstp=%g\n", stp);

        // vector triple prod
        gsl_vector *vtp = gslu_vector_triple_prod_alloc (&a.vector, &b.vector, c);
        gslu_vector_printf (vtp, "vtp");

        gsl_vector_free (c);
        gsl_vector_free (vtp);
    }

    /*
     * gsl_util_matrix.h
     */
    GSLU_MATRIX_VIEW (B_static, 4, 4, 
                      {  9, 1, 8, 2,
                        10, 3, 2, 3,
                         5, 8, 9, 6,
                         1, 3, 3, 5 });
    gsl_matrix *B = &B_static.matrix;


    GSLU_MATRIX_VIEW (C_static, 4, 4, 
                      {0,   1,  2,  3,
                       4,   5,  6,  7,
                       8,   9, 10, 11,
                       12, 13, 14, 15}
        );
    gsl_matrix *C = &C_static.matrix;
    gslu_matrix_printf (C, "C");

    // dynamic allocation
    GSLU_MATRIX_VIEW (A_static, 4, 4, 
                      { 4, 10, 7, 4,
                        0, 1,  7, 6,
                        7, 5,  7, 1,
                        8, 6,  7, 1 });
    gsl_matrix *A = &A_static.matrix;
    gslu_matrix_printf (A, "A");
    gslu_matrix_printfc (A, "A", NULL, CblasTrans);


    // selrow
    gsl_matrix *D = gslu_matrix_selrow_alloc (C, &isel.vector);
    gslu_matrix_printf (D, "D");


    // selcol
    gsl_matrix *E = gslu_matrix_selcol_alloc (C, &isel.vector);
    gslu_matrix_printf (E, "E");

    // equal
    printf ("D==D ? %d\n", gslu_matrix_is_equal (D, D));

    gslu_matrix_set_submatrix (C, 0, 2, E);
    gslu_matrix_printf (C, "C");

    // submarix
    gslu_matrix_set_submatrix (C, 2, 0, D);
    gslu_matrix_printf (C, "C");


    // determinant
    double det = gslu_matrix_det (A);
    printf ("\ndet=%g\n", det);

    // trace
    double trace = gslu_matrix_trace (A);
    printf ("\ntrace=%g\n", trace);

    // inverse
    gsl_matrix *Ainv = gsl_matrix_alloc (A->size1, A->size2);
    gslu_matrix_inv (Ainv, A);
    gslu_matrix_printf (Ainv, "\nAinv");

    // reshaping a matrix into a new size
    GSLU_MATRIX_VIEW (C_2x8, 2, 8);
    gslu_matrix_reshape (&C_2x8.matrix, C, CblasNoTrans);
    gslu_matrix_printf (&C_2x8.matrix, "C_2x8");

    // reshaping the transpose of matrix
    GSLU_MATRIX_VIEW (C_2x8T, 2, 8);
    gslu_matrix_reshape (&C_2x8T.matrix, C, CblasTrans);
    gslu_matrix_printf (&C_2x8T.matrix, "C_2x8T");

    // stacking a matrix into vector form
    GSLU_VECTOR_VIEW (c, 16);
    gslu_matrix_stack (&c.vector, C, CblasNoTrans);
    gslu_vector_printf (&c.vector, "c");

    // reshaping a vector back into a matrix
    GSLU_MATRIX_VIEW (Cprime, 4, 4);
    gslu_vector_reshape (&Cprime.matrix, &c.vector, CblasNoTrans);
    gslu_matrix_printf (&Cprime.matrix, "Cprime");

    // skew symmetric matrix
    GSLU_VECTOR_VIEW (t, 3, {1, 2, 3});
    gsl_matrix *skew = gslu_matrix_skewsym_alloc (&t.vector);
    gslu_matrix_printf (skew, "skew");

    /*
     * gsl_index_blas.h
     */

    // matrix * vector
    GSLU_VECTOR_VIEW (b, 4);
    gslu_blas_mv (&b.vector, A, &x.vector); // A*x = b
    gslu_vector_printf (&b.vector, "b");    

    // vector^T * matrix
    gsl_vector *bT = gslu_blas_vTm_alloc (A, &x.vector);
    gslu_vector_printfc (bT, "b", NULL, CblasTrans);

    
    // vectorT * matrix * vector
    double scalar = gslu_blas_vTmv (&x.vector, A, y);
    printf ("scalar=%g\n", scalar);

    // computes C = A*B
    gsl_matrix *C1 = gslu_blas_mm_alloc (A, B);
    gslu_matrix_printf (C1, "C1");

    // computes C = A*B^T
    gsl_matrix *C2 = gslu_blas_mmT_alloc (A, B);
    gslu_matrix_printf (C2, "C2");


    // computes C = A^T*B
    gsl_matrix *C3 = gslu_blas_mTm_alloc (A, B);
    gslu_matrix_printf (C3, "C3");


    // computes C = A^T*B^T
    gsl_matrix *C4 = gslu_blas_mTmT_alloc (A, B);
    gslu_matrix_printf (C4, "C4");


    // computes D = A*B*C
    gsl_matrix *D1 = gslu_blas_mmm_alloc (A, B, C, NULL);
    gslu_matrix_printf (D1, "D1");

    // computes D = A*B*C^T
    gsl_matrix *D2 = gslu_blas_mmmT_alloc (A, B, C, NULL);
    gslu_matrix_printf (D2, "D2");

    // computes D = A*B^T*C
    gsl_matrix *D3 = gslu_blas_mmTm_alloc (A, B, C, NULL);
    gslu_matrix_printf (D3, "D3");

    // computes D = A^T*B*C
    gsl_matrix *D4 = gslu_blas_mTmm_alloc (A, B, C, NULL);
    gslu_matrix_printf (D4, "D4");

    // computes D = A*B'*C'
    gsl_matrix *D5 = gslu_blas_mmTmT_alloc (A, B, C, NULL);
    gslu_matrix_printf (D5, "D5");

    // computes D = A'*B'*C
    gsl_matrix *D6 = gslu_blas_mTmTm_alloc (A, B, C, NULL);
    gslu_matrix_printf (D6, "D6");

    // computes D = A'*B*C'
    gsl_matrix *D7 = gslu_blas_mTmmT_alloc (A, B, C, NULL);
    gslu_matrix_printf (D7, "D7");

    // computes D = A'*B'*C'
    gsl_matrix *D8 = gslu_blas_mTmTmT_alloc (A, B, C, NULL);
    gslu_matrix_printf (D8, "D8");



    /*
     * gsl_index_linalg.h
     */

    printf ("\nQR factorization of A\n");
    gslu_linalg_QR *qr = gslu_linalg_QR_decomp_alloc (A);
    gsl_matrix *Q = gsl_matrix_alloc (A->size1, A->size1);
    gsl_matrix *R = gsl_matrix_alloc (A->size1, A->size2);
    gsl_linalg_QR_unpack (qr->QR, qr->tau, Q, R);
    gslu_matrix_printf (Q, "Q");
    gslu_matrix_printf (R, "R");
    gslu_linalg_QR_free (qr);
    gsl_matrix_free (Q);
    gsl_matrix_free (R);


    printf ("\nSVD decomposition of A (econ) for thin or square matrices\n");
    gslu_linalg_SV *sv = gslu_linalg_SV_decomp_econ_alloc (A);
    gslu_matrix_printf (sv->U, "U");
    gslu_vector_printf (sv->S, "S");
    gslu_matrix_printf (sv->V, "V");
    gslu_linalg_SV_free (sv);


    printf ("\nSVD decomposition of A (full)\n");
    gsl_matrix_view A_fat = gsl_matrix_submatrix (&A_static.matrix, 0, 0, 3, 4);
    gslu_matrix_printf (&A_fat.matrix, "A fat");
    gslu_linalg_SV *sv2 = gslu_linalg_SV_decomp_full_alloc (&A_fat.matrix);
    gslu_matrix_printf (sv2->U, "U");
    gslu_vector_printf (sv2->S, "S");
    gslu_matrix_printf (sv2->V, "V");
    gslu_linalg_SV_free (sv2);

    printf ("\nReduced row echelon form of A\n");
    gsl_matrix *A_rref = gslu_linalg_rref_alloc (A, 0);
    gslu_matrix_printf (A_rref, "A rref");
    gslu_matrix_free (A_rref);
    
    /*
     * gsl_util_rand.h
     */


    const double rho = 0.9999999999999;
    GSLU_VECTOR_VIEW (mu, 5, {0, 0, 200, 300, 400});
    GSLU_MATRIX_VIEW (Sigma, 5, 5, 
                      { 1,   -rho,  0,  0, 0,
                       -rho,    1,  0,  0, 0,
                        0,      0, .1,  0, 0,
                        0,      0,  0,  1, 0,
                        0,      0,  0,  0, .00001 });
    gsl_rng *rng = gslu_rand_rng_alloc ();

    double s;
    GSLU_VECTOR_VIEW (w_static, 5, {0.0});
    GSLU_MATRIX_VIEW (W_static, 5, 10, {0.0});
    gsl_vector *w = &w_static.vector;
    gsl_matrix *W = &W_static.matrix;

    // uniform
    printf ("\nuniform\n");
    s = gslu_rand_uniform (rng);
    gslu_rand_uniform_vector (rng, w);
    gslu_rand_uniform_matrix (rng, W);
    printf ("s=%g\n", s);
    gslu_vector_printf (w, "w");
    gslu_matrix_printf (W, "W");

    // uniform-pos
    printf ("\nuniform-pos\n");
    s = gslu_rand_uniform_pos (rng);
    gslu_rand_uniform_pos_vector (rng, w);
    gslu_rand_uniform_pos_matrix (rng, W);
    printf ("s=%g\n", s);
    gslu_vector_printf (w, "w");
    gslu_matrix_printf (W, "W");

    // normal
    printf ("\nnormal\n");
    s = gslu_rand_normal (rng);
    gslu_rand_normal_vector (rng, w);
    gslu_rand_normal_matrix (rng, W);
    printf ("s=%g\n", s);
    gslu_vector_printf (w, "w");
    gslu_matrix_printf (W, "W");

    // guassian
    printf ("\ngaussian\n");
    s = gslu_rand_gaussian (rng, 1.0, 3.0);
    gslu_rand_gaussian_vector (rng, w, &mu.vector, &Sigma.matrix, NULL);
    gslu_rand_gaussian_matrix (rng, W, &mu.vector, &Sigma.matrix, NULL);
    printf ("s=%g\n", s);
    gslu_vector_printf (w, "w");
    gslu_matrix_printf (W, "W");


    // clean up
    gsl_rng_free (rng);
    gsl_vector_free (bT);
    gsl_vector_free (y);
    gsl_vector_free (x_sub);
    gsl_matrix_free (Ainv);
    gsl_matrix_free (C1);
    gsl_matrix_free (C2);
    gsl_matrix_free (C3);
    gsl_matrix_free (C4);
    gsl_matrix_free (D);
    gsl_matrix_free (D1);
    gsl_matrix_free (D2);
    gsl_matrix_free (D3);
    gsl_matrix_free (D4);
    gsl_matrix_free (D5);
    gsl_matrix_free (D6);
    gsl_matrix_free (D7);
    gsl_matrix_free (D8);
    gsl_matrix_free (E);
}
