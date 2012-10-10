#ifndef __PERLS_VISION_ZERNIKE_H__
#define __PERLS_VISION_ZERNIKE_H__

#include <stddef.h>
#include <opencv/cv.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector_complex_double.h>
#include <gsl/gsl_matrix_complex_double.h>

#include "perls-math/gsl_util_index.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct vis_zernike_params vis_zernike_params_t;

struct vis_zernike_params
{
    size_t window_size;              /* half of size of feature window 
                                      * [(2w+1)x(2w+1)]
                                      */

    size_t r_nsamp;                  /* number of samples in r direction
                                      */

    size_t t_nsamp;                  /* number of samples in theta direction
                                      */

    size_t moment_order;             /* zernike moment order (=keylen)
                                      */

    size_t nsamp;                    /* number of sample points in a patch
                                      */

    gsl_matrix *sampler;             /* grid contain sampler points
                                      */

    size_t repetition;

    gslu_index *m_idx;               /* storage for m indeces */

    gsl_matrix_complex *V_nm;        /* order n, repetition m
                                      */
    
    gsl_vector *darea;               /* dArea of sampled patch */

};

vis_zernike_params_t
vis_zernike_init (size_t window_size, size_t r_nsamp, size_t t_nsamp, size_t order);

/* evaluates the Zernike polynomial V_nm
 * for the given order n and repetition m over points within the unit circle
 * defined by polar coordinates rsamp and tsamp.  
 * This implementation should be accurate up to order n=43.
 */
void
vis_zernike_polynomial (const size_t n, const size_t m, 
                        const gsl_vector *rsamp, const gsl_vector *tsamp, 
                        vis_zernike_params_t params,
                        gsl_vector_complex *V_nm_col);

/* normalizes the polar coordinate image patch by demeaning and  diving by the patch energy.  
 * This normalization yields a normalized correlation score in the range [-1,1].
 */
void 
zernike_detrend_polar (gsl_vector *polarpatch, vis_zernike_params_t params);

void
vis_zernike_polarpatch (const CvArr* image, const double u, const double v, gsl_matrix *Hinf,
                        vis_zernike_params_t params, gsl_vector *workspace, 
                        gsl_vector_complex *patch_col);

void
vis_zernike_destroy (vis_zernike_params_t params);


#ifdef __cplusplus
}
#endif

#endif // __PERLS_VISION_ZERNIKE_H__
