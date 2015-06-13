#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <opencv/cv.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-common/error.h"
#include "perls-math/gsl_util.h"
#include "perls-math/homogenous.h"

#include "opencv_util.h"
#include "calib.h"
#include "distortion.h"

/*
function [xd,yd] = oulu_forward_distortion(kr,kt,xn,yn)
%OULU_FORWARD_DISTORTION  Applies the Heikkila forward distortion model.
%  [XD,YD] = OULU_FORWARD_DISTORTION(KR,KT,XN,YN) applies the
%  distortion model to undistorted normalized image coordinates
%  represented by the [Nx1] vectors XN,YN.  KR is the [3x1] vector of
%  radial distortion coefficients and KT is the [2x1] vector of
%  tangential distortion.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-14-2002      rme         Created and written.
%    03-14-2010      rme         Ported to libvision.
*/
void
vis_distort_pts_radial (const gsl_matrix *src, gsl_matrix *dst, const gsl_matrix *_K, const gsl_vector *distCoeffs)
{

    gsl_matrix *K;
    if (_K == NULL)
    {
        K = gsl_matrix_alloc (3, 3);
        gsl_matrix_set_identity (K);
    }
    else
        K = gslu_matrix_clone (_K);


    assert (src->size1 == 2 && gslu_matrix_is_same_size (src, dst) &&
            K->size1 == 3 && K->size2 == 3 && distCoeffs->size == 5);

    double k1 = gsl_vector_get (distCoeffs, 0);
    double k2 = gsl_vector_get (distCoeffs, 1);
    double p1 = gsl_vector_get (distCoeffs, 2);
    double p2 = gsl_vector_get (distCoeffs, 3);
    double k3 = gsl_vector_get (distCoeffs, 4);

    // normalized coordinates
    gsl_matrix *tmp_h = homogenize_alloc (src);

    gsl_matrix *Kinv = gslu_matrix_inv_alloc (K);
    //gslu_matrix_printf(tmp_h,"tmp_h");
    //gslu_matrix_printf(K,"K");
    //gslu_matrix_printf(Kinv,"Kinv");
    gsl_matrix *xy_h = gslu_blas_mm_alloc (Kinv, tmp_h);


    for (size_t n=0; n<src->size2; n++)
    {
        double x = gsl_matrix_get (xy_h, 0, n);
        double y = gsl_matrix_get (xy_h, 1, n);

        double r2 = x*x + y*y;
        double r4 = r2 * r2;
        double r6 = r4 * r2;

        // apply radial distortion model to undistorted normalized points
        double dr = k1*r2 + k2*r4 + k3*r6;
        double xd = x * (1.0+dr);
        double yd = y * (1.0+dr);

        // apply tangential distortion model
        xd += 2*p1*x*y + p2*(r2+2*x*x);
        yd += p1*(r2+2*y*y) + 2*p2*x*y;

        gsl_matrix_set (xy_h, 0, n, xd);
        gsl_matrix_set (xy_h, 1, n, yd);
    }

    // pixel coordinates
    gslu_blas_mm (tmp_h, K, xy_h);
    dehomogenize (tmp_h, dst);

    // clean up
    gsl_matrix_free (Kinv);
    gsl_matrix_free (xy_h);
    gsl_matrix_free (tmp_h);
    gsl_matrix_free (K);
}

void
vis_undistort_pts_radial (const gsl_matrix *src, gsl_matrix *dst, const gsl_matrix *_K, const gsl_vector *distCoeffs)
{
    gsl_matrix *K;
    if (_K == NULL)
    {
        K = gsl_matrix_alloc (3, 3);
        gsl_matrix_set_identity (K);
    }
    else
        K = gslu_matrix_clone (_K);

    assert (src->size1 == 2 && gslu_matrix_is_same_size (src, dst) &&
            K->size1 == 3 && K->size2 == 3 && distCoeffs->size == 5);

    /* Note: cvUndistortPoints only seems to work with points stored as a Nx1 2-channel array.
     */
    gsl_matrix *srcT = gslu_matrix_transpose_alloc (src); // [N x 2]
    gsl_matrix *dstT = gslu_matrix_transpose_alloc (dst); // [N x 2]

    CvMat srcT_2NC1 = vis_cvu_gsl_matrix_to_cvmat_view (srcT); // [N x 2]
    CvMat srcT_1NC2_header, *srcT_1NC2;
    srcT_1NC2 = cvReshape(&srcT_2NC1, &srcT_1NC2_header, 2, 1); // [N x 1] 2-channel

    CvMat dstT_2NC1 = vis_cvu_gsl_matrix_to_cvmat_view (dstT); // [N x 2]
    CvMat dstT_1NC2_header, *dstT_1NC2;
    dstT_1NC2 = cvReshape(&dstT_2NC1, &dstT_1NC2_header, 2, 1); // [N x 1] 2-channel

    // undistort
    CvMat K_cv = vis_cvu_gsl_matrix_to_cvmat_view (K);
    CvMat distCoeffs_cv = vis_cvu_gsl_vector_to_cvmat_view (distCoeffs);
    cvUndistortPoints (srcT_1NC2, dstT_1NC2, &K_cv, &distCoeffs_cv, NULL, &K_cv);

    // copy back
    gsl_matrix_transpose_memcpy (dst, dstT);

    // clean up
    gsl_matrix_free (srcT);
    gsl_matrix_free (dstT);
    gsl_matrix_free (K);
}


IplImage *
vis_undistort_mask (const perllcm_van_calib_t *calib)
{
    vis_calib_const_view_cv_t cv = vis_calib_const_view_cv (calib);
    IplImage *Imask = cvCreateImage (cv.imageSize, IPL_DEPTH_8U, 1);
    IplImage *I = cvCreateImage (cv.imageSize, IPL_DEPTH_8U, 1);

    cvSet (I, cvScalarAll (255), NULL);
    vis_cvu_map_t *map = vis_undistort_map (calib);
    cvRemap (I, Imask, map->mapu, map->mapv,
             CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll (0));

    // This would be the preferred way to generate the mask if black fill-in was working in cvUndistort2
    //cvUndistort2 (Iall, Imask, calib->cv.K, calib->cv.distCoeffs);

    cvReleaseImage (&I);

    return Imask;
}

#define VIS_DISTORTION_FORWARD  0
#define VIS_DISTORTION_BACKWARD 1

static vis_cvu_map_t *
_vis_distortion_map (const perllcm_van_calib_t *calib, int direction)
{
    vis_cvu_map_t *map = malloc (sizeof (*map));
    map->mapu = cvCreateMat (calib->height, calib->width, CV_32FC1);
    map->mapv = cvCreateMat (calib->height, calib->width, CV_32FC1);

    // fill with pixel coordinates
    gsl_matrix *uv = gsl_matrix_alloc (2, calib->height * calib->width);
    for (size_t v = 0, k = 0; v < calib->height; v++)
    {
        for (size_t u = 0; u < calib->width; u++, k++)
        {
            gsl_matrix_set (uv, 0, k, u);
            gsl_matrix_set (uv, 1, k, v);
        }
    }

    // distort points
    vis_calib_const_view_gsl_t gsl = vis_calib_const_view_gsl (calib);
    switch (calib->kc_model)
    {
    case PERLLCM_VAN_CALIB_T_KC_MODEL_RADTAN:
        if (direction == VIS_DISTORTION_FORWARD)
            vis_distort_pts_radial (uv, uv, &gsl.K.matrix, &gsl.kc.vector);
        else
            vis_undistort_pts_radial (uv, uv, &gsl.K.matrix, &gsl.kc.vector);
        break;

    default:
        ERROR ("unsupported distortion model, kc_model=%d", calib->kc_model);
        exit (EXIT_FAILURE);
    }

    // map to opencv
    for (size_t v = 0, k = 0; v < calib->height; v++)
    {
        for (size_t u = 0; u < calib->width; u++, k++)
        {
            cvmSet (map->mapu, v, u, gsl_matrix_get (uv, 0, k));
            cvmSet (map->mapv, v, u, gsl_matrix_get (uv, 1, k));
        }
    }

    // clean up
    gsl_matrix_free (uv);

    return map;
}


vis_cvu_map_t *
vis_undistort_map (const perllcm_van_calib_t *calib)
{
    return _vis_distortion_map (calib, VIS_DISTORTION_FORWARD);
}

vis_cvu_map_t *
vis_distort_map (const perllcm_van_calib_t *calib)
{
    return _vis_distortion_map (calib, VIS_DISTORTION_BACKWARD);
}
