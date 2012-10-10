#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <inttypes.h> // needed for PRId64 macros
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include "perls-common/error.h"
#include "perls-math/gsl_util.h"

#include "opencv_util.h"

void
vis_cvu_iplimg_save (const char *logdir, int64_t utime, const IplImage *img)
{
    char filename[PATH_MAX];
    snprintf (filename, sizeof filename, "%s/%"PRId64".ppm", logdir, utime);
    cvSaveImage (filename, img, 0);
}

IplImage *
vis_cvu_iplimg_load (const char *logdir, int64_t utime)
{
    char filename[PATH_MAX];
    snprintf (filename, sizeof filename, "%s/%"PRId64".ppm", logdir, utime);
    return cvLoadImage (filename, CV_LOAD_IMAGE_UNCHANGED);
}

IplImage *
vis_cvu_iplimg_pop (cache_t *imgcache, const char *logdir, int64_t utime)
{
    IplImage *img_warp = cache_pop (imgcache, utime);
    if (!img_warp) {
        IplImage *img_warp = vis_cvu_iplimg_load (logdir, utime);
        if (!img_warp) {
            ERROR ("unable to load img_warp %"PRId64" from disk!\n", utime);
            return NULL;
        } 
        else
            cache_push (imgcache, utime, cvCloneImage (img_warp));
    }
    return img_warp;
}

void
vis_cvu_map_free (vis_cvu_map_t *map)
{
/* printf ("1\n"); */
    cvReleaseMat (&map->mapu);
/* printf ("2\n"); */
    cvReleaseMat (&map->mapv);
/* printf ("3\n"); */
    free (map);
/* printf ("4\n"); */
}

void
vis_cvu_warp_image (const IplImage *src, IplImage *dst, const vis_cvu_map_t *map)
{
    cvRemap (src, dst, map->mapu, map->mapv,
             CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll (0));
}

double
vis_cvu_get_pixel_value_1c (const IplImage *image, double x, double y)
{
    /* Following discussion of "FAQ Technical Questions on Library Use"
     * in opencv c reference (http://opencv.willowgarage.com/documentation/index.html)
     * it is faster than cvGet2D()
     */

    double val = 0.0;

    int xl, xu, yl, yu;
    xl = floor (x); xu = ceil (x);
    yl = floor (y); yu = ceil (y);

    if (image->depth > 8)
        ERROR ("vis_cvu_get_pixel_value for > 8 bit is not implemented yet.");
    else {
        // if the value is out of image size, return 0.0
        if (x < 0.0 || y < 0.0 || x > image->width || y > image->height)
            return 0.0;

        // 8 bit, I(x,y) ~ ((uchar*)(img->imageData + img->widthStep*y))[x]
        int xpt, ypt;
        if (xl == xu && yl == yu) {  // x and y are integers
            xpt = xl; ypt = yl;
            val = ((uchar*)(image->imageData + image->widthStep*ypt))[xpt];
            val = (double) val;
        }
        else if (xl == xu) { // only x is integer
            xpt = xl;
            uchar val_yl = ((uchar*)(image->imageData + image->widthStep*yl))[xpt];
            uchar val_yu = ((uchar*)(image->imageData + image->widthStep*yu))[xpt];
        
            val = (double) val_yl + (double) (val_yu - val_yl) * (double) (y - yl) / (double) (yu - yl);
        }
        else if (yl == yu) { // only y is integer
            ypt = yl;

            uchar val_xl = ((uchar*)(image->imageData + image->widthStep*ypt))[xl];
            uchar val_xu = ((uchar*)(image->imageData + image->widthStep*ypt))[xu];

            val = (double) val_xl + (double) (val_xu - val_xl) * (double) (x - xl) / (double) (xu - xl);
        }
        else { // either x or y are integer
            uchar val_xlyl = ((uchar*)(image->imageData + image->widthStep*yl))[xl];
            uchar val_xlyu = ((uchar*)(image->imageData + image->widthStep*yu))[xl];
            uchar val_xuyl = ((uchar*)(image->imageData + image->widthStep*yl))[xu];
            uchar val_xuyu = ((uchar*)(image->imageData + image->widthStep*yu))[xu];

            double val_yu = (double) val_xlyu + (double) (val_xuyu - val_xlyu) * (double) (x - xl) / (double) (xu - xl);
            double val_yl = (double) val_xlyl + (double) (val_xuyl - val_xlyl) * (double) (x - xl) / (double) (xu - xl);

            val = (double) val_yl + (double) (val_yu - val_yl) * (double) (y - yl) / (double) (yu - yl);
            
            //printf ("(%d~%d),(%d~%d),%d,%d,%d,%d ", xl,xu, yl,yu, val_xlyl, val_xuyl, val_xlyu, val_xuyu);
            //printf ("then %g,%g==> %g\n", val_yl, val_yu, val);
        }
    }

    return val;
}


/*-------------------MATRIX COPY---------------------------------*/
gsl_matrix *
vis_cvu_cvmat_to_gsl_matrix_copy (const CvMat *cmat)
{
    gsl_matrix *gmat_copy = gsl_matrix_alloc (cmat->rows, cmat->cols);
    assert (gmat_copy!=NULL);

    gsl_matrix_view gmat_view = vis_cvu_cvmat_to_gsl_matrix_view (cmat);
    gsl_matrix_memcpy (gmat_copy, &gmat_view.matrix);
    
    return gmat_copy;
}

CvMat *
vis_cvu_gsl_matrix_to_cvmat_copy (const gsl_matrix *gmat)
{
    CvMat *cmat_copy = cvCreateMat (gmat->size1, gmat->size2, CV_64FC1);
    assert (cmat_copy!=NULL);

    CvMat cmat_view = vis_cvu_gsl_matrix_to_cvmat_view (gmat);
    cvCopy (&cmat_view, cmat_copy, NULL);
    
    return cmat_copy;
}

/*-------------------MATRIX VIEW---------------------------*/
gsl_matrix_view
vis_cvu_cvmat_to_gsl_matrix_view (const CvMat *cmat)
{
    int type = CV_MAT_TYPE (cmat->type);
    assert (type == CV_64FC1);
    return gsl_matrix_view_array (cmat->data.db, cmat->rows, cmat->cols);
}

CvMat
vis_cvu_gsl_matrix_to_cvmat_view (const gsl_matrix *gmat)
{
    CvMat cmat;
    cvInitMatHeader (&cmat, gmat->size1, gmat->size2, CV_64FC1, gmat->data, gmat->tda*sizeof(double));
    return cmat;
}


/*-------------------VECTOR COPY--------------------------------*/
gsl_vector *
vis_cvu_cvmat_to_gsl_vector_copy (const CvMat *cvec)
{
    assert (cvec->rows==1 || cvec->cols==1);

    gsl_vector *gvec_copy = gsl_vector_alloc (cvec->rows*cvec->cols);
    assert (gvec_copy!=NULL);

    gsl_vector_view gvec_view = vis_cvu_cvmat_to_gsl_vector_view (cvec);
    gsl_vector_memcpy (gvec_copy, &gvec_view.vector);

    return gvec_copy;
}

CvMat *
vis_cvu_gsl_vector_to_cvmat_copy (const gsl_vector *gvec)
{
    CvMat *cvec_copy = cvCreateMat (gvec->size, 1, CV_64FC1);
    assert (cvec_copy!=NULL);

    CvMat cvec_view = vis_cvu_gsl_vector_to_cvmat_view (gvec);
    cvCopy (&cvec_view, cvec_copy, NULL);

    return cvec_copy;
}


/*-------------------VECTOR VIEW---------------------------*/
gsl_vector_view
vis_cvu_cvmat_to_gsl_vector_view (const CvMat *cvec)
{    
    assert (cvec->rows==1 || cvec->cols==1);
    int type = CV_MAT_TYPE (cvec->type);
    assert (type == CV_64FC1);
    return gsl_vector_view_array (cvec->data.db, cvec->rows*cvec->cols);
}

CvMat
vis_cvu_gsl_vector_to_cvmat_view (const gsl_vector *gvec)
{
    CvMat cvec;
    cvInitMatHeader (&cvec, gvec->size, 1, CV_64FC1, gvec->data, gvec->stride*sizeof(double));
    return cvec;
}


/*********************************PRINTING**************************************/
void
vis_cvu_matrix_printf (const CvMat *cmat, const char *name)
{
    gsl_matrix_view gmat = vis_cvu_cvmat_to_gsl_matrix_view (cmat);
    gslu_matrix_printf (&gmat.matrix, name);
}


void
vis_cvu_matrix_printfc (const CvMat *cmat, const char *name, const char *fmt, CBLAS_TRANSPOSE_t Trans)
{
    gsl_matrix_view gmat = vis_cvu_cvmat_to_gsl_matrix_view (cmat);
    gslu_matrix_printfc (&gmat.matrix, name, fmt, Trans);    
}

void
vis_cvu_vector_printf (const CvMat *cvec, const char *name)
{
    gsl_vector_view gvec = vis_cvu_cvmat_to_gsl_vector_view (cvec);
    gslu_vector_printf (&gvec.vector, name);
}


void
vis_cvu_vector_printfc (const CvMat *cvec, const char *name, const char *fmt, CBLAS_TRANSPOSE_t Trans)
{
    gsl_vector_view gvec = vis_cvu_cvmat_to_gsl_vector_view (cvec);
    gslu_vector_printfc (&gvec.vector, name, fmt, Trans);    
}
