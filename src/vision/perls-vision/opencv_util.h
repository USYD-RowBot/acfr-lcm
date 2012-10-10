#ifndef __PERLS_VISION_OPENCV_UTIL_H__
#define __PERLS_VISION_OPENCV_UTIL_H__

#include <opencv/cv.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>

#include "perls-common/cache.h"

#ifdef __cplusplus
extern "C" {
#endif


/************************
 * IplImage Utils
 ************************/
// data struct to be used with cvRemap ()
typedef struct vis_cvu_map vis_cvu_map_t;
struct vis_cvu_map
{
    CvMat *mapu; // pixels
    CvMat *mapv; // pixels
};

/*
 * Saves IplImage (for plotting) to van-processed dir
 */
void
vis_cvu_iplimg_save (const char *logdir, int64_t utime, const IplImage *img);

/*
 * Loads IplImage (for plotting) from van-processed dir
 */
IplImage *
vis_cvu_iplimg_load (const char *logdir, int64_t utime);

/*
 * Pops a copy of an IplImage off the shared memory stack and moves it to the top of the stack
 */
IplImage *
vis_cvu_iplimg_pop (cache_t *cache, const char *logdir, int64_t utime);

void
vis_cvu_map_free (vis_cvu_map_t *map);

void
vis_cvu_warp_image (const IplImage *src, IplImage *dst, const vis_cvu_map_t *map);

/*  get pixel value from a gray scale image (1 channel image), 
 *  if x and y are not integer, interpolate pixel values linearly
 */
double
vis_cvu_get_pixel_value_1c (const IplImage *image, double x, double y);

/************************
 * CvMat Utils
 ************************/

// CvMat to gsl_matrix - use gsl_matrix_free()
gsl_matrix *
vis_cvu_cvmat_to_gsl_matrix_copy (const CvMat *cmat);

// CvMat to gsl_vector - use gsl_matrix_free()
gsl_vector *
vis_cvu_cvmat_to_gsl_vector_copy (const CvMat *cvec);

// gsl_matrix to CvMat - use cvReleaseMat()
CvMat *
vis_cvu_gsl_matrix_to_cvmat_copy (const gsl_matrix *gmat);

// gsl_vector to CvMat - must use CvReleaseMat()
CvMat *
vis_cvu_gsl_vector_to_cvmat_copy (const gsl_vector *gvec);

// CvMat to gsl_matrix_view - scope defined by CvMat - CvMat must be 64F
gsl_matrix_view
vis_cvu_cvmat_to_gsl_matrix_view (const CvMat *cmat);

// CvMat to gsl_vector_view - scope defined by CvMat - CvMat must be 64F
gsl_vector_view
vis_cvu_cvmat_to_gsl_vector_view (const CvMat *cvec);

// gsl_matrix to CvMat - scope defined by gsl_matrix
CvMat
vis_cvu_gsl_matrix_to_cvmat_view (const gsl_matrix *gmat);

// gsl_matrix to CvMat - scope defined by gsl_vector
CvMat
vis_cvu_gsl_vector_to_cvmat_view (const gsl_vector *gvec);


/* prints the contents of a CvMat to stdout.  each element is formatted
 * using the printf-style format specifier fmt.  If fmt is NULL, then it
 * defaults to "%f", Trans is one of either CblasNoTrans, CblasTrans, CblasConjTrans
 */
void
vis_cvu_matrix_printf (const CvMat *cmat, const char *name);

void
vis_cvu_matrix_printfc (const CvMat *cmat, const char *name, const char *fmt, CBLAS_TRANSPOSE_t Trans); // custom

void
vis_cvu_vector_printf (const CvMat *cvec, const char *name);

void
vis_cvu_vector_printfc (const CvMat *cvec, const char *name, const char *fmt, CBLAS_TRANSPOSE_t Trans);

#ifdef __cplusplus
}
#endif

#endif //__PERLS_VISION_OPENCV_UTIL_H__
