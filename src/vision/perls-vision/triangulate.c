#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/gsl_util.h"
#include "perls-math/homogenous.h"

#include "triangulate.h"

void
vis_triangulate (const gsl_matrix *K, const gsl_matrix *R, const gsl_vector *t, const gsl_matrix *uv1, const gsl_matrix *uv2,    // inputs
                 gsl_matrix *_X1, gsl_vector *_alpha, gsl_vector *_beta, gsl_vector *_gamma)                                     // outputs
{
    size_t Np = uv1->size2;

    assert (gslu_matrix_is_same_size (uv1, uv2) && (uv1->size1 == 2) && (uv1->size2 == Np) &&
            (_X1->size1 == 3) && (_X1->size2 == Np));
    assert (_alpha ? (_alpha->size == Np) : (_alpha == NULL));
    assert (_beta  ? (_beta->size  == Np) : (_beta  == NULL));
    assert (_gamma ? (_gamma->size == Np) : (_gamma == NULL));


    // determine the Euclidian ray direction vectors for each image point
    gsl_matrix *invK = gslu_matrix_inv_alloc (K);
    gsl_matrix *uv1_h = homogenize_alloc (uv1);
    gsl_matrix *uv2_h = homogenize_alloc (uv2);

    gsl_matrix *r1_mat = gslu_blas_mm_alloc (invK, uv1_h);
    gsl_matrix *r2_mat = gslu_blas_mm_alloc (invK, uv2_h);

    // following Horn's notation
    GSLU_VECTOR_VIEW (b, 3);
    GSLU_VECTOR_VIEW (c, 3);
    GSLU_VECTOR_VIEW (r1_prime, 3);
    GSLU_VECTOR_VIEW (X2, 3);

    gsl_vector_memcpy (&b.vector, t);
    gsl_vector_scale (&b.vector, -1.0); // b = -t

    for (size_t j=0; j<Np; j++)
    {
        // normalize the rays to unit magnitude
        gsl_vector_view r1 = gsl_matrix_column (r1_mat, j);
        double r1_norm = gslu_vector_norm (&r1.vector);
        gsl_vector_scale (&r1.vector, 1.0/r1_norm); // r1_hat

        gsl_vector_view r2 = gsl_matrix_column (r2_mat, j);
        double r2_norm = gslu_vector_norm (&r2.vector);
        gsl_vector_scale (&r2.vector, 1.0/r2_norm); // r2_hat

        // rotate the ray from camera 1 into orientation of camera 2 frame
        gslu_blas_mv (&r1_prime.vector, R, &r1.vector); // r1_prime = R*r1

        // c = r1_prime x r2
        gslu_vector_cross (&c.vector, &r1_prime.vector, &r2.vector);
        double c_norm = gslu_vector_norm (&c.vector);


        // magnitudes along rays r1_prime and r2 where closest intersection occurs
        double alpha = gslu_vector_scalar_triple_prod (&c.vector, &b.vector, &r2.vector); // alpha = c' * (b x r2)
        alpha /= c_norm*c_norm;
        if (_alpha)
            gsl_vector_set (_alpha, j, alpha);

        double beta = gslu_vector_scalar_triple_prod (&c.vector, &b.vector, &r1_prime.vector); // gamma = c' * (b x r1_prime)
        beta /= c_norm*c_norm;
        if (_beta)
            gsl_vector_set (_beta, j, beta);

        // perpendicular distance at closest intersection
        double gamma = gslu_vector_dot (&b.vector, &c.vector); // gamma = b'*c
        gamma /= c_norm*c_norm;
        if (_gamma)
            gsl_vector_set (_gamma, j, gamma);

        // scene as represented in camera frame 2 where scene features are defined
        // to lie at midpoint of ray intersection
        gsl_vector_memcpy (&X2.vector, &r2.vector);
        gsl_vector_scale (&X2.vector, beta);
        gsl_vector_scale (&c.vector, 0.5*gamma);
        gsl_vector_sub (&X2.vector, &c.vector); // X2 = beta*r2 - gamma/2*c

        // scene as represented in camera frame 1
        gsl_vector_view X1 = gsl_matrix_column (_X1, j);
        gsl_vector_sub (&X2.vector, t);
        gsl_blas_dgemv (CblasTrans, 1.0, R, &X2.vector, 0.0, &X1.vector); // X1 = R' * (X2-t)
    }

    // clean up
    gsl_matrix_free (invK);
    gsl_matrix_free (uv1_h);
    gsl_matrix_free (uv2_h);
    gsl_matrix_free (r1_mat);
    gsl_matrix_free (r2_mat);
}

vis_triangulate_t *
vis_triangulate_alloc (const gsl_matrix *K, const gsl_matrix *R, const gsl_vector *t, const gsl_matrix *uv1, const gsl_matrix *uv2)
{
    size_t Np = uv1->size2;

    vis_triangulate_t *tri = malloc (sizeof (vis_triangulate_t));
    tri->X1 = gsl_matrix_alloc (3, Np);
    tri->alpha = gsl_vector_alloc (Np);
    tri->beta  = gsl_vector_alloc (Np);
    tri->gamma = gsl_vector_alloc (Np);

    vis_triangulate (K, R, t, uv1, uv2, tri->X1, tri->alpha, tri->beta, tri->gamma);

    return tri;
}

void
vis_triangulate_free (vis_triangulate_t *tri)
{
    gsl_matrix_free (tri->X1);
    gsl_vector_free (tri->alpha);
    gsl_vector_free (tri->beta);
    gsl_vector_free (tri->gamma);
    free (tri);
}

gslu_index *
vis_triangulate_constraint_alloc (const gsl_matrix *X, const int j, const double lb, const double ub)
{
    int n = X->size2;   // X = 3 x N matrix
    gslu_index *tri_const_idx = NULL;

    int n_accepted = 0;
    for (int i=0; i<n; i++)
    {
        if (gsl_matrix_get (X,j,i) > lb && gsl_matrix_get (X,j,i) < ub)
        {
            n_accepted++;
        }
    }

    if (n_accepted > 0)
    {
        tri_const_idx = gslu_index_alloc (n_accepted);
        int idx = 0;
        for (int i=0; i<n; i++)
        {
            if (gsl_matrix_get(X,j,i) > lb && gsl_matrix_get(X,j,i) < ub)
            {
                gslu_index_set(tri_const_idx, idx, i);
                idx++;
            }
        }
    }

    return tri_const_idx;
}

