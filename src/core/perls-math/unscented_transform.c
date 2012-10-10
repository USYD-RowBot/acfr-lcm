#include "unscented_transform.h"

#include "gsl_util_matrix.h"
#include "gsl_util_blas.h"
#include "gsl_util_linalg.h"

unscented_transform_opts_t * 
unscented_transform_def_opts ()
{
    unscented_transform_opts_t *opts = calloc (1, sizeof(*opts));
    opts->alpha = 1.0;
    opts->beta = 2.0;
    opts->kappa = 0.0;
    return opts;
}

void
unscented_transform_alloc (const gsl_vector *mu, const gsl_matrix *R, const unscented_transform_opts_t *opts,
                           gsl_matrix **sigmaPoints, gsl_vector **meanWeights, gsl_vector **covWeights)
{

    int n = mu->size;
    int numSigmaPoints = 2*n+1;

    *sigmaPoints = gsl_matrix_calloc(n, numSigmaPoints);
    *meanWeights = gsl_vector_calloc(numSigmaPoints);
    *covWeights  = gsl_vector_calloc(numSigmaPoints);

    if (opts)
        unscented_transform (mu, R, opts, *sigmaPoints, *meanWeights, *covWeights);
    else {
        unscented_transform_opts_t *defaultOpts = unscented_transform_def_opts();
        unscented_transform (mu, R, defaultOpts, *sigmaPoints, *meanWeights, *covWeights);
        free (defaultOpts);
    }

}

int
unscented_transform (const gsl_vector *mu, const gsl_matrix *R, const unscented_transform_opts_t *opts,
                     gsl_matrix *sigmaPoints, gsl_vector *meanWeights, gsl_vector *covWeights)
{

    int n = sigmaPoints->size1;
    int numSigmaPoints = 2*n+1;

    assert(sigmaPoints->size1 == n && sigmaPoints->size2 == numSigmaPoints);


    double lambda = pow(opts->alpha, 2) * (n+opts->kappa) - n;

    gsl_matrix *RCopy = gsl_matrix_alloc(R->size1, R->size2);
	gsl_matrix_memcpy(RCopy, R);
    gsl_matrix_scale(RCopy, n+lambda);
    gsl_matrix *U = gslu_linalg_sqrtm_alloc(RCopy);

    int i=0;
    gsl_vector_view UColumn;
    gsl_vector_view ithSigmaPoint;

    //set the 0th sigma point to be the mean
    ithSigmaPoint = gsl_matrix_column(sigmaPoints, 0);
    gsl_vector_memcpy(&(ithSigmaPoint.vector), mu);

    gsl_vector_set (meanWeights, 0, lambda / (n+lambda));
    gsl_vector_set (covWeights, 0, lambda / (n+lambda) + (1 - opts->alpha*opts->alpha + opts->beta));

    for (i=1; i<=n; i++) {
        /* Compute sigma point */
        UColumn = gsl_matrix_column(U, i-1);
        ithSigmaPoint = gsl_matrix_column(sigmaPoints, i);
        gsl_vector_memcpy(&(ithSigmaPoint.vector), mu);
        gsl_vector_add(&(ithSigmaPoint.vector), &(UColumn.vector));

        /* Compute mean weight */
        gsl_vector_set (meanWeights, i, 1/(2*(n+lambda)));

        /* Compute covariance weight */
        gsl_vector_set (covWeights, i, 1/(2*(n+lambda)));
    }

    for (i=n+1; i<=2*n; i++) {
        /* Compute sigma point */
        UColumn = gsl_matrix_column(U, i-n-1);
        ithSigmaPoint = gsl_matrix_column(sigmaPoints, i);
        gsl_vector_memcpy(&(ithSigmaPoint.vector), mu);
        gsl_vector_sub(&(ithSigmaPoint.vector), &(UColumn.vector));

        /* Compute mean weight */
        gsl_vector_set (meanWeights, i, 1/(2*(n+lambda)));

        /* Compute covariance weight */
        gsl_vector_set (covWeights, i, 1/(2*(n+lambda)));
    }

    //clean up
    gsl_matrix_free(RCopy);
    gsl_matrix_free(U);
    
    return EXIT_SUCCESS;

}
