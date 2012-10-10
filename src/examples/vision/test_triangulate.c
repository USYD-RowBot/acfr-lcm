#include <stdio.h>
#include <stdlib.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/gsl_util.h"
#include "perls-vision/triangulate.h"

#define NPOINTS  10

int
main (int argc, char *argv[])
{
    printf ("Hello World!\n");
    FILE *fd;

    // R
    gsl_matrix *R = gsl_matrix_alloc (3, 3);
    fd = fopen ("R.txt", "r");
    gsl_matrix_fscanf (fd, R);
    fclose (fd);
    gslu_matrix_printf (R, "R");

    // t
    gsl_vector *t = gsl_vector_alloc (3);
    fd = fopen ("t.txt", "r");
    gsl_vector_fscanf (fd, t);
    fclose (fd);
    gslu_vector_printf (t, "t");

    // K
    gsl_matrix *K = gsl_matrix_alloc (3, 3);
    fd = fopen ("K.txt", "r");
    gsl_matrix_fscanf (fd, K);
    fclose (fd);
    gslu_matrix_printf (K, "K");
    
    // uv1
    gsl_matrix *uv1 = gsl_matrix_alloc (2, NPOINTS);
    fd = fopen ("uv1.txt", "r");
    gsl_matrix_fscanf (fd, uv1);
    fclose (fd);
    gslu_matrix_printf (uv1, "uv1");

    // uv2
    gsl_matrix *uv2 = gsl_matrix_alloc (2, NPOINTS);
    fd = fopen ("uv2.txt", "r");
    gsl_matrix_fscanf (fd, uv2);
    fclose (fd);
    gslu_matrix_printf (uv2, "uv2");

    // X1
    gsl_matrix *X1 = gsl_matrix_alloc (3, NPOINTS);
    fd = fopen ("X1.txt", "r");
    gsl_matrix_fscanf (fd, X1);
    fclose (fd);

    // alpha
    gsl_vector *alpha = gsl_vector_alloc (NPOINTS);
    fd = fopen ("alpha.txt", "r");
    gsl_vector_fscanf (fd, alpha);
    fclose (fd);

    // beta
    gsl_vector *beta = gsl_vector_alloc (NPOINTS);
    fd = fopen ("beta.txt", "r");
    gsl_vector_fscanf (fd, beta);
    fclose (fd);

    // gamma
    gsl_vector *gamma = gsl_vector_alloc (NPOINTS);
    fd = fopen ("gamma.txt", "r");
    gsl_vector_fscanf (fd, gamma);
    fclose (fd);


    vis_triangulate_t *tri = vis_triangulate_alloc (K, R, t, uv1, uv2);

    gsl_matrix_sub (X1, tri->X1);
    gslu_matrix_printf (X1, "X1 diff");

    gsl_vector_sub (alpha, tri->alpha);
    gslu_vector_printfc (alpha, "alpha diff", NULL, CblasTrans);

    gsl_vector_sub (beta, tri->beta);
    gslu_vector_printfc (beta, "beta diff", NULL, CblasTrans);

    gsl_vector_sub (gamma, tri->gamma);
    gslu_vector_printfc (gamma, "gamma diff", NULL, CblasTrans);

    vis_triangulate_free (tri);

    return 0;
}
