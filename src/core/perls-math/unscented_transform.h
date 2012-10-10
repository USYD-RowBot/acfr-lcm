#ifndef __PERLS_MATH_UNSCENTED_TRANSFORM_H__
#define __PERLS_MATH_UNSCENTED_TRANSFORM_H__

#include <math.h>
#include <assert.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_eigen.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct unscented_transform_opts_t {
    double alpha;
    double beta;
    double kappa;
} unscented_transform_opts_t;

unscented_transform_opts_t * 
unscented_transform_def_opts ();

void
unscented_transform_alloc (const gsl_vector *mu, const gsl_matrix *R, const unscented_transform_opts_t *opts,
                           gsl_matrix **sigmaPoints, gsl_vector **meanWeights, gsl_vector **covWeights);

int
unscented_transform (const gsl_vector *mu, const gsl_matrix *R, const unscented_transform_opts_t *opts,
                     gsl_matrix *sigmaPoints, gsl_vector *meanWeights, gsl_vector *covWeights);

#ifdef __cplusplus
}
#endif

#endif //__PERLS_MATH_UNSCENTED_TRANSFORM_H__
