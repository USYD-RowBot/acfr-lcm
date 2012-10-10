#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <opencv/cv.h>

#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_sort.h>
#include <gsl/gsl_sort_vector.h>
#include <gsl/gsl_statistics.h>

#include "perls-common/error.h"
#include "perls-common/magic.h"
#include "perls-common/units.h"

#include "perls-math/gsl_util.h"
#include "perls-math/gsl_util_index.h"
#include "perls-math/homogenous.h"
#include "perls-math/plane.h"
#include "perls-math/ssc.h"

#include "calib.h"
#include "camera.h"
#include "feature.h"
#include "homography.h"
#include "opencv_util.h"
#include "featuregpu.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define MIN_DVL_NPTS 4

perllcm_van_feature_collection_t *
vis_feature_collection_alloc (int64_t utime, const char *channel)
{
    perllcm_van_feature_collection_t *fc = calloc (1, sizeof (*fc));
    fc->utime = utime;
    fc->channel = strdup (channel);
    return fc;
}

void
vis_feature_collection_add (perllcm_van_feature_collection_t *fc, perllcm_van_feature_t *f)
{
    perllcm_van_feature_t *tmp = malloc ((fc->ntypes+1) * sizeof (*f));
    for (size_t n=0; n < fc->ntypes; n++)
        tmp[n] = fc->f[n];
    tmp[fc->ntypes++] = *f;
    free (fc->f);
    fc->f = tmp;
}

void
vis_feature_collection_scene_prior (perllcm_van_feature_collection_t *fc, 
                                    const gsl_matrix *XYZ_l,
                                    const gsl_vector *bid,
                                    const gsl_vector *x_lc,
                                    const perllcm_van_calib_t *calib)
{
    assert (XYZ_l->size1==3);

    /***************************************************
     * 1) project bathymetry prior into image space
     ***************************************************/

    // init scene prior to zero
    perllcm_van_scene_prior_t sp0 = {0};
    fc->scene_prior = sp0;

    // assign default beam id if none provided
    gsl_vector *tmpbid = NULL;
    if (bid) {
        assert (XYZ_l->size2 == bid->size);
        tmpbid = gslu_vector_clone (bid);
    } else {
        tmpbid = gsl_vector_alloc (XYZ_l->size2);
        gsl_vector_set_all (tmpbid, -1);
    }

    // transform our bathy prior from local-level to the camera frame
    GSLU_VECTOR_VIEW (x_cl, 6, {0, 0, 0, 0, 0, 0});
    if (x_lc)
        ssc_inverse_gsl (&x_cl.vector, NULL, x_lc);
    GSLU_MATRIX_VIEW (H_cl, 4, 4);
    ssc_homo4x4 (H_cl.data, x_cl.data);
    gsl_matrix *tmpXYZ_l_h = homogenize_alloc (XYZ_l);
    gsl_matrix *tmpXYZ_c_h = gslu_blas_mm_alloc (&H_cl.matrix, tmpXYZ_l_h);

    // find points in front of the camera
    size_t npts = 0;
    for (size_t i=0, j=0; i<tmpXYZ_c_h->size2; i++) {
        const double z = gsl_matrix_get (tmpXYZ_c_h, 2, i);
        if (z > 0.0) {
            gsl_matrix_swap_columns (tmpXYZ_c_h, j, i);
            gsl_vector_swap_elements (tmpbid, j, i);
            npts = ++j;
        }
    }
    if (npts < MIN_DVL_NPTS) {
        for (size_t n=0; n<fc->ntypes; n++) {
            perllcm_van_feature_t *f = &fc->f[n];
            perllcm_van_feature_user_depth_t *user = calloc (1, sizeof (*user));
            user->npts = 0;
            int32_t max_user_size = perllcm_van_feature_user_depth_t_encoded_size (user);
            f->usertype = PERLLCM_VAN_FEATURE_T_USERTYPE_DEPTH;
            f->user = malloc (max_user_size);
            f->usersize = perllcm_van_feature_user_depth_t_encode (f->user, 0, max_user_size, user);
            perllcm_van_feature_user_depth_t_destroy (user);
        }

        gsl_vector_free (tmpbid);
        gsl_matrix_free (tmpXYZ_l_h);
        gsl_matrix_free (tmpXYZ_c_h);
        return;
    }

    // project points into pixel space
    vis_calib_const_view_gsl_t calib_gsl = vis_calib_const_view_gsl (calib);
    GSLU_MATRIX_VIEW (P, 3, 4);
    vis_camera_matrix (&P.matrix, &calib_gsl.K.matrix, NULL, NULL);
    gsl_matrix *tmpuv = gsl_matrix_alloc (2, npts);
    gsl_matrix_view XYZ_c_h_front = gsl_matrix_submatrix (tmpXYZ_c_h, 0, 0, 4, npts);
    vis_camera_project (&P.matrix, &XYZ_c_h_front.matrix, tmpuv);

    // find projections within bounding box
    npts = 0;
    const double umin = -1.0*calib->width;
    const double umax =  2.0*calib->width;
    const double vmin = -1.0*calib->height;
    const double vmax =  2.0*calib->height;
    for (size_t i=0, j=0; i<tmpuv->size2; i++) {
        const double u = gsl_matrix_get (tmpuv, 0, i);
        const double v = gsl_matrix_get (tmpuv, 1, i);
        if (umin < u && u < umax &&
            vmin < v && v < vmax) {
            gsl_matrix_swap_columns (tmpXYZ_c_h, j, i);
            gsl_matrix_swap_columns (tmpuv, j, i);
            gsl_vector_swap_elements (tmpbid, j, i);
            npts = ++j;
        }
    }
    if (npts < MIN_DVL_NPTS) {
        for (size_t n=0; n<fc->ntypes; n++) {
            perllcm_van_feature_t *f = &fc->f[n];
            perllcm_van_feature_user_depth_t *user = calloc (1, sizeof (*user));
            user->npts = 0;
            int32_t max_user_size = perllcm_van_feature_user_depth_t_encoded_size (user);
            f->usertype = PERLLCM_VAN_FEATURE_T_USERTYPE_DEPTH;
            f->user = malloc (max_user_size);
            f->usersize = perllcm_van_feature_user_depth_t_encode (f->user, 0, max_user_size, user);
            perllcm_van_feature_user_depth_t_destroy (user);
        }

        gsl_vector_free (tmpbid);
        gsl_matrix_free (tmpXYZ_l_h);
        gsl_matrix_free (tmpXYZ_c_h);
        return;
    }
    gsl_matrix_view XYZ_c_h_valid = gsl_matrix_submatrix (tmpXYZ_c_h, 0, 0, 4, npts);
    gsl_matrix_view uv_valid = gsl_matrix_submatrix (tmpuv, 0, 0, 2, npts);
    gsl_vector_view bid_valid = gsl_vector_subvector (tmpbid, 0, npts);


    // stuff a scene prior data struct
    gsl_vector *tmpZ = gsl_vector_alloc (npts);
    fc->scene_prior.npts = npts;
    fc->scene_prior.X = malloc (npts*sizeof (*fc->scene_prior.X));
    fc->scene_prior.Y = malloc (npts*sizeof (*fc->scene_prior.Y));
    fc->scene_prior.Z = malloc (npts*sizeof (*fc->scene_prior.Z));
    fc->scene_prior.u = malloc (npts*sizeof (*fc->scene_prior.u));
    fc->scene_prior.v = malloc (npts*sizeof (*fc->scene_prior.v));
    fc->scene_prior.id = malloc (npts*sizeof (*fc->scene_prior.id));
    for (size_t n=0; n<npts; n++) {
        fc->scene_prior.X[n] = gsl_matrix_get (&XYZ_c_h_valid.matrix, 0, n);
        fc->scene_prior.Y[n] = gsl_matrix_get (&XYZ_c_h_valid.matrix, 1, n);
        fc->scene_prior.Z[n] = gsl_matrix_get (&XYZ_c_h_valid.matrix, 2, n);
        fc->scene_prior.u[n] = gsl_matrix_get (&uv_valid.matrix, 0, n);
        fc->scene_prior.v[n] = gsl_matrix_get (&uv_valid.matrix, 1, n);
        fc->scene_prior.id[n] = gsl_vector_get (&bid_valid.vector, n);
        
        gsl_vector_set (tmpZ, n, fc->scene_prior.Z[n]);
    }
    
    // compute Z statistics
    gsl_sort_vector (tmpZ);
    fc->scene_prior.Zmin = gsl_vector_get (tmpZ, 0);
    fc->scene_prior.Zmax = gsl_vector_get (tmpZ, tmpZ->size-1);
    fc->scene_prior.Z10  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.10);
    fc->scene_prior.Z20  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.20);
    fc->scene_prior.Z30  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.30);
    fc->scene_prior.Z40  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.40);
    fc->scene_prior.Z50  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.50);
    fc->scene_prior.Z60  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.60);
    fc->scene_prior.Z70  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.70);
    fc->scene_prior.Z80  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.80);
    fc->scene_prior.Z90  = gsl_stats_quantile_from_sorted_data (tmpZ->data, tmpZ->stride, tmpZ->size, 0.90);

    fc->scene_prior.Zmean = gsl_stats_mean (tmpZ->data, tmpZ->stride, tmpZ->size);
    fc->scene_prior.Zstd = gsl_stats_sd (tmpZ->data, tmpZ->stride, tmpZ->size);
    fc->scene_prior.Zabsdev = gsl_stats_absdev (tmpZ->data, tmpZ->stride, tmpZ->size);

    // clean up
    gsl_vector_free (tmpbid);
    gsl_vector_free (tmpZ);
    gsl_matrix_free (tmpuv);
    gsl_matrix_free (tmpXYZ_l_h);
    gsl_matrix_free (tmpXYZ_c_h);


    /***************************************************
     * 2) compute depth prior for each feature pixel
     ***************************************************/

    // test if the scene is locally planar
    gsl_matrix_view XYZ_c_valid = gsl_matrix_submatrix (&XYZ_c_h_valid.matrix, 0, 0, 3, npts);
    gsl_vector_view coeff = gsl_vector_view_array (fc->scene_prior.lp_coeff, 4);
    if (XYZ_c_valid.matrix.size2 >= 3) {
        gsl_vector *tmperror = NULL;
        plane_estim_svd (&XYZ_c_valid.matrix, &coeff.vector, &tmperror);
        fc->scene_prior.lp_absdev = gsl_stats_absdev (tmperror->data, tmperror->stride, tmperror->size);
        //fc->scene_prior.lp_absdev = 0.1;    // 10 cm
        const double THRESH = 0.25; // [m]
        if (fc->scene_prior.lp_absdev < THRESH)
            fc->scene_prior.locally_planar = 1;
        else
            fc->scene_prior.locally_planar = 0;
        gsl_vector_free (tmperror);
    }


    // assign feature depth prior
    for (size_t n=0; n<fc->ntypes; n++) {
        perllcm_van_feature_t *f = &fc->f[n];
        perllcm_van_feature_user_depth_t *user = malloc (sizeof (*user));
        user->npts = f->npts;
        user->mu_Z = malloc (f->npts*sizeof (*user->mu_Z));
        user->Sigma_Z = malloc (f->npts*sizeof (*user->Sigma_Z));

        if (fc->scene_prior.locally_planar) {
            // construct uv_h homogenous point array
            gsl_vector_float_const_view uf = gsl_vector_float_const_view_array (f->u, f->npts);
            gsl_vector_float_const_view vf = gsl_vector_float_const_view_array (f->v, f->npts);
            gsl_matrix *tmpuv_h = gsl_matrix_alloc (3, f->npts);
            gsl_vector_view u = gsl_matrix_row (tmpuv_h, 0);
            gsl_vector_view v = gsl_matrix_row (tmpuv_h, 1);
            gsl_vector_view one = gsl_matrix_row (tmpuv_h, 2);
            GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector_float, &uf.vector, gsl_vector, &u.vector);
            GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector_float, &vf.vector, gsl_vector, &v.vector);
            gsl_vector_set_all (&one.vector, 1.0);

            // normalized rays
            gsl_matrix *tmpxy_h = gslu_blas_mm_alloc (&calib_gsl.Kinv.matrix, tmpuv_h);

            // find ideal scene depth based upon plane-ray intersection
            gsl_matrix *tmpxyz = gsl_matrix_alloc (3, f->npts);
            plane_ray_intersection (&coeff.vector, tmpxy_h, tmpxyz);

            // assign scene depth to user field
            for (size_t i=0; i<f->npts; i++) {
                user->mu_Z[i] = gsl_matrix_get (tmpxyz, 2, i);
                user->Sigma_Z[i] = fc->scene_prior.lp_absdev * fc->scene_prior.lp_absdev;
            }

            // clean up
            gslu_matrix_free (tmpuv_h);
            gslu_matrix_free (tmpxy_h);
            gslu_matrix_free (tmpxyz);

        }
        else { // 3d structure    
            // find closest bathymetry point and assign it's depth
            float rclosest = GSL_POSINF;
            float Zclosest = fc->scene_prior.Z50;
            for (size_t n=0; n < f->npts; n++) {
                for (size_t i=0; i < fc->scene_prior.npts; i++) {
                    float du = f->u[n] - fc->scene_prior.u[i];
                    float dv = f->v[n] - fc->scene_prior.v[i];
                    float r  = sqrtf (du*du + dv*dv);
                    if (r < rclosest) {
                        rclosest = r;
                        Zclosest = fc->scene_prior.Z[i];
                    }
                }
                user->mu_Z[n] = Zclosest;
                user->Sigma_Z[n] = fc->scene_prior.Zabsdev*fc->scene_prior.Zabsdev;
            }
        }

        int32_t max_user_size = perllcm_van_feature_user_depth_t_encoded_size (user);
        f->usertype = PERLLCM_VAN_FEATURE_T_USERTYPE_DEPTH;
        f->user = malloc (max_user_size);
        f->usersize = perllcm_van_feature_user_depth_t_encode (f->user, 0, max_user_size, user);
        perllcm_van_feature_user_depth_t_destroy (user);
    }
}


perllcm_van_feature_t *
vis_feature_cvsurf (const IplImage *image, const IplImage *mask, CvSURFParams params, int npts_max)
{
    CvSeq *keypoints = NULL, *descriptors = NULL;
    CvMemStorage *storage = cvCreateMemStorage (0);

    cvExtractSURF (image, mask, &keypoints, &descriptors, storage, params, 0);
    if (!keypoints) {
        cvReleaseMemStorage (&storage);
        return NULL;
    }
    
    // sort keypoints by hessian (if necessary)
    gsl_permutation *perm=NULL;
    if (npts_max < 0) // keep all keypoints found
        npts_max = keypoints->total;
    else if (keypoints->total > npts_max) { // sort points by hessian for pruning
        gsl_vector_float *hessian = gsl_vector_float_alloc (keypoints->total);
        for (int n=0; n<keypoints->total; n++) {
            const CvSURFPoint *keypoint = (CvSURFPoint *) cvGetSeqElem (keypoints, n);
            gsl_vector_float_set (hessian, n, keypoint->hessian);
        }
        
        perm = gsl_permutation_alloc (keypoints->total);
        gsl_sort_vector_float_index (perm, hessian); // ascending order
        gsl_vector_float_free (hessian);
    }

    // allocate feature_t
    perllcm_van_feature_t *f = malloc (sizeof (*f));
    f->npts  = GSL_MIN (keypoints->total, npts_max);

    f->u = malloc (f->npts * sizeof (*f->u));
    f->v = malloc (f->npts * sizeof (*f->v));

    f->keylen = params.extended ? 128 : 64;
    f->keys = malloc (f->npts * sizeof (*f->keys));


    perllcm_van_feature_attr_cvsurf_t *attr = malloc (sizeof (*attr));
    attr->npts = f->npts;
    attr->laplacian = malloc (f->npts * sizeof (*attr->laplacian));
    attr->size = malloc (f->npts * sizeof (*attr->size));
    attr->dir = malloc (f->npts * sizeof (*attr->dir));
    attr->hessian = malloc (f->npts * sizeof (*attr->hessian));


    // populate feature_t
    for (int n=0; n < f->npts; n++) {
        // if perm exists, use it to select points from strongest to weakest,
        // o/w just keep all points
        int i = perm ? perm->data[f->npts-1-n] : n;
        const CvSURFPoint *keypoint = (CvSURFPoint *) cvGetSeqElem (keypoints, i);
        const float *descriptor = (float *) cvGetSeqElem (descriptors, i);

        // store keypoint pixel location
        f->u[n] = keypoint->pt.x;
        f->v[n] = keypoint->pt.y;

        // store keypoint key
        f->keys[n] = malloc (f->keylen * sizeof (*f->keys[n]));
        memcpy (f->keys[n], descriptor, f->keylen * sizeof (*f->keys[n]));

        // store keypoint attributes
        attr->laplacian[n] = keypoint->laplacian;
        attr->size[n] = keypoint->size;
        attr->dir[n] = keypoint->dir * DTOR;
        attr->hessian[n] = keypoint->hessian;
    }

    // assign attr to feature_t
    int32_t max_attr_size = perllcm_van_feature_attr_cvsurf_t_encoded_size (attr);
    f->attrtype = PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF;
    f->attr = malloc (max_attr_size);
    f->attrsize = perllcm_van_feature_attr_cvsurf_t_encode (f->attr, 0, max_attr_size, attr);

    // assign null user to feature_t
    f->usertype = PERLLCM_VAN_FEATURE_T_USERTYPE_NONE;
    f->usersize = 0;
    f->user = NULL;

    // clean up
    perllcm_van_feature_attr_cvsurf_t_destroy (attr);
    cvRelease((void **) &keypoints);
    cvRelease((void **) &descriptors);
    cvReleaseMemStorage (&storage);
    if (perm) gsl_permutation_free (perm);

    return f;
}


perllcm_van_feature_t *
vis_feature_siftgpu (const IplImage *image, const IplImage *mask, const char *server_ipaddr, int server_port, int npts_max)
{
    return vis_siftgpu_client (image, mask, server_ipaddr, server_port, npts_max);
}

perllcm_van_feature_t *
vis_feature_surfgpu (const IplImage *image, const IplImage *mask, const char *server_ipaddr, int server_port, int npts_max)
{
    return vis_surfgpu_client (image, mask, server_ipaddr, server_port, npts_max);
}

gsl_matrix *
vis_feature_patch_sampler_alloc (size_t w)
{

    int nsamp = (2*w+1)*(2*w+1);
    gsl_matrix *sampler = NULL;

    if (w>0)
        sampler = gsl_matrix_alloc (2, nsamp);

    // computes x samples and y samples and gen sampler
    size_t idx = 0;
    for (int x=-(int)w; x<=(int)w; x++) {
        for (int y=-(int)w; y<=(int)w; y++) {
            gsl_matrix_set (sampler, 0, idx, x);
            gsl_matrix_set (sampler, 1, idx, y);

            idx++;
        }
    }

    return sampler;
}

void
vis_feature_harris_patch (const IplImage *image, const double u, const double v, gsl_matrix *Hinf,
                          gsl_matrix *sampler, gsl_vector *sampled_pixel_col)
{
    // project via Hinf and warp to N-E coordinate
    double uw, vw, xo, yo;

    vis_homog_single_pt_project (Hinf, u, v, &uw, &vw);

    GSLU_MATRIX_VIEW (invHinf, 3, 3);
    gslu_matrix_inv (&invHinf.matrix, Hinf);

    size_t n_samp = sampler->size2;

    for (size_t i=0; i<n_samp; i++) {
        double xsamp = gsl_matrix_get (sampler, 0, i) + uw;
        double ysamp = gsl_matrix_get (sampler, 1, i) + vw;

        vis_homog_single_pt_project (&invHinf.matrix, xsamp, ysamp, &xo, &yo);

        // get pixel value at xy_o
        double val = vis_cvu_get_pixel_value_1c (image, xo, yo);

        // weight with area
        gsl_vector_set (sampled_pixel_col, i, val);
    }

    // normalize with mean and variance
    double demean, variance;
    demean = - gsl_stats_mean (sampled_pixel_col->data, 1, sampled_pixel_col->size);
    variance = gsl_stats_variance (sampled_pixel_col->data, 1, sampled_pixel_col->size);
    
    gsl_vector_add_constant (sampled_pixel_col, demean);
    gsl_vector_scale (sampled_pixel_col, 1/sqrt (variance));

}

perllcm_van_feature_t *
vis_feature_cvharris (const IplImage *image, const IplImage *mask, 
                      vis_feature_harris_params_t *hp, 
                      gsl_matrix *featpatch_sampler,
                      perllcm_pose3d_t x_lc, gsl_matrix *K,
                      int npts_max)
{
    if (npts_max < 0)
        npts_max = 2000;

    IplImage *corner_img = cvCreateImage (cvGetSize (image), IPL_DEPTH_8U, 1);
    IplImage *storage = cvCreateImage (cvGetSize (image), IPL_DEPTH_8U, 1);
    CvPoint2D32f corners[npts_max];
    int corner_count = npts_max;

    cvGoodFeaturesToTrack (image,            // input
                           corner_img,       // output
                           storage,          // temp storage
                           corners,          // cornerss
                           &corner_count,    // # feat pts
                           hp->qualityLevel,
                           hp->minDistance,
                           mask,
                           hp->blockSize,
                           1,
                           hp->k);

    if (!corner_count) {
        cvReleaseImage (&corner_img);
        cvReleaseImage (&storage);
        return NULL;
    }


    // allocate feature_t
    perllcm_van_feature_t *f = malloc (sizeof (*f));
    f->npts  = GSL_MIN (corner_count, npts_max);
    f->u = malloc (f->npts * sizeof (*f->u));
    f->v = malloc (f->npts * sizeof (*f->v));

    // construct H inf
    GSLU_MATRIX_VIEW (Hinf, 3, 3);
    GSLU_MATRIX_VIEW (R, 3, 3);
    gsl_vector_const_view mu = gsl_vector_const_view_array (x_lc.mu, 6);
    gsl_vector_const_view rph = gsl_vector_const_subvector (&mu.vector, 3, 3);
    so3_rotxyz_gsl (&R.matrix, &rph.vector);
    vis_homog_matrix_infinite (&Hinf.matrix, K, &R.matrix);

    // populate feature_t
    int nsamp = featpatch_sampler->size2;
    gsl_vector *patch_col = gsl_vector_alloc (nsamp);   // sampled_pixel stacked in col

    f->keylen = nsamp;
    f->keys = malloc (f->npts * sizeof (*f->keys));

    for (int n=0; n < f->npts; n++) {
        // store keypoint pixel location
        f->u[n] = corners[n].x;
        f->v[n] = corners[n].y;

        vis_feature_harris_patch (image, f->u[n], f->v[n],
                                  &Hinf.matrix, featpatch_sampler, patch_col);

        // store keypoint key
        f->keys[n] = malloc (f->keylen * sizeof (*f->keys[n]));
        gsl_vector_float_view fkeyn_f = gsl_vector_float_view_array (f->keys[n], f->keylen);
        GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector, patch_col, gsl_vector_float, &fkeyn_f.vector);

    }

    // assign null attr to feature_t
    f->attrtype = PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVHARRIS;
    f->attrsize = 0;
    f->attr = NULL;

    // assign null user to feature_t
    f->usertype = PERLLCM_VAN_FEATURE_T_USERTYPE_NONE;
    f->usersize = 0;
    f->user = NULL;

    // clean up
    cvReleaseImage (&corner_img);
    cvReleaseImage (&storage);
    gsl_vector_free (patch_col);

    return f;
}

double
_vis_feature_get_simscore_euclidean (const perllcm_van_feature_t *fi, const perllcm_van_feature_t *fj, int i, int j)
{
    // calculate similarity score
    // the squared Eucldiean distance between two vectors x & y can be written as:
    // (x-y)'*(x-y) = x'*x - 2*x'*y + y'*y
    // ---------------------------------------------------- //
    double xTx, yTy, xTy;
    gsl_vector_float_const_view keyi = gsl_vector_float_const_view_array (fi->keys[i], fi->keylen);
    gsl_vector_float_const_view keyj = gsl_vector_float_const_view_array (fj->keys[j], fj->keylen);
    gsl_blas_dsdot (&keyi.vector, &keyi.vector, &xTx);
    gsl_blas_dsdot (&keyj.vector, &keyj.vector, &yTy);
    gsl_blas_dsdot (&keyi.vector, &keyj.vector, &xTy);

    return xTx - 2.0*xTy + yTy;
}

double
vis_feature_get_simscore (const perllcm_van_feature_t *fi, const perllcm_van_feature_t *fj, int i, int j)
{
    assert (fi->attrtype==fj->attrtype && fi->keylen==fj->keylen && i<fi->npts && j<fj->npts);

    double sij = GSL_POSINF;
    switch (fi->attrtype) {
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF:
        sij = _vis_feature_get_simscore_euclidean (fi, fj, i, j);
        break;
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_SIFTGPU:
        sij = _vis_feature_get_simscore_euclidean (fi, fj, i, j);
        break;
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_SURFGPU:
        sij = _vis_feature_get_simscore_euclidean (fi, fj, i, j);
        break;
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVHARRIS:
        sij = _vis_feature_get_simscore_euclidean (fi, fj, i, j);
        break;
    default:
        ERROR ("unknown attrtype %d", fi->attrtype);
    }
    return sij;
}

void 
_vis_feature_get_smatrix_euclidean (const perllcm_van_feature_t *fi, const perllcm_van_feature_t *fj, gsl_matrix *smatrix)
{
    if (fi->attrtype == PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF) {
        // use laplacian information: compare the sign
        // ---------------------------------------------------- //
        // laplacian = -1 0 +1, check this sign for robust match
        perllcm_van_feature_attr_cvsurf_t *attri = malloc (sizeof (*attri));
        if (perllcm_van_feature_attr_cvsurf_t_decode (fi->attr, 0, fi->attrsize, attri) < 0) {
            ERROR ("perllcm_van_feature_attr_cvsurf_t_decode() failed");
            free (attri);
            return;
        }

        perllcm_van_feature_attr_cvsurf_t *attrj = malloc (sizeof (*attrj));
        if (perllcm_van_feature_attr_cvsurf_t_decode (fj->attr, 0, fj->attrsize, attrj) < 0) {
            ERROR ("perllcm_van_feature_attr_cvsurf_t_decode() failed");
            free (attri);
            free (attrj);
            return;
        }

        // if two signs are diff, give inf dist in feature vec space
        for (size_t i=0; i<fi->npts; i++)
            for (size_t j=0; j<fj->npts; j++)
                if (attri->laplacian[i] != attrj->laplacian[j])
                    gsl_matrix_set (smatrix, i, j, GSL_POSINF);
                else
                    gsl_matrix_set (smatrix, i, j, _vis_feature_get_simscore_euclidean (fi, fj, i, j));

        // clean up
        perllcm_van_feature_attr_cvsurf_t_destroy (attri);
        perllcm_van_feature_attr_cvsurf_t_destroy (attrj);
    } else {
        for (size_t i=0; i<fi->npts; i++)
            for (size_t j=0; j<fj->npts; j++)
                gsl_matrix_set (smatrix, i, j, _vis_feature_get_simscore_euclidean (fi, fj, i, j));
    }
}

gsl_matrix *
vis_feature_get_smatrix (const perllcm_van_feature_t *fi, const perllcm_van_feature_t *fj)
{
    assert (fi->attrtype==fj->attrtype && fi->keylen==fj->keylen);

    gsl_matrix *smatrix = gsl_matrix_alloc (fi->npts, fj->npts);
    switch (fi->attrtype) {
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF:
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_SIFTGPU:
        _vis_feature_get_smatrix_euclidean (fi, fj, smatrix);
        break;
    default:
        ERROR ("unknown attrtype %d", fi->attrtype);
    }
    return smatrix;
}

double 
vis_feature_get_simscore_gsl (const gsl_vector *key1, const gsl_vector *key2)
{
    // calculate similarity score
    // the squared Eucldiean distance between two vectors x & y can be written as:
    // (x-y)'*(x-y) = x'*x - 2*x'*y + y'*y
    // ---------------------------------------------------- //
    double xTx = gslu_vector_dot (key1, key1);
    double yTy = gslu_vector_dot (key2, key2);
    double xTy = gslu_vector_dot (key1, key2);
    return xTx - 2.0*xTy + yTy;
}

double
vis_feature_get_simscore_gsl_float (const gsl_vector_float *key1, const gsl_vector_float *key2)
{
    // calculate similarity score
    // the squared Eucldiean distance between two vectors x & y can be written as:
    // (x-y)'*(x-y) = x'*x - 2*x'*y + y'*y
    // ---------------------------------------------------- //
    float xTx = gslu_vector_float_dot (key1, key1);
    float yTy = gslu_vector_float_dot (key2, key2);
    float xTy = gslu_vector_float_dot (key1, key2);
    return xTx - 2.0*xTy + yTy;
}

void
vis_feature_get_smatrix_gsl (gsl_matrix *key1, gsl_matrix *key2, gsl_matrix* smatrix)
{
    // the squared Eucldiean distance between two vectors x & y can be written as:
    // (x-y)'*(x-y) = x'*x - 2*x'*y + y'*y
    // compute Euclidean distance between all keypairs

    int n1 = key1->size2;
    int n2 = key2->size2;

    /* calculate similarity score
     * SURF has normalized key vector xTx = yTy = 1
     * smatrix = 2*ones(n1,n2) - 2xTy                       */
    // ---------------------------------------------------- //
    gsl_matrix* xTy =  gsl_matrix_alloc (n1,n2);
    gsl_blas_dgemm (CblasTrans, CblasNoTrans, 1.0, key1, key2, 0.0, xTy);

    gsl_matrix_set_all (smatrix, 2.0);
    gsl_matrix_scale (xTy,-2.0);
    gsl_matrix_add (smatrix,xTy);      

    // clean up
    gsl_matrix_free (xTy);

}

int
vis_feature_ubc_match_alloc (const perllcm_van_feature_t *f1, const perllcm_van_feature_t *f2,
                             gsl_matrix **uv1, gsl_matrix **uv2, double simAB_thresh, int simscore_minmax)
{
    int n1 = f1->npts;
    int n2 = f2->npts;
    int nomatch = n1 > n2 ? n1 + 1 : n2 + 1;
    int ret = 0;
    double simscore_thresh = simAB_thresh;

    /* original indeces */
    gslu_index *sel1_temp = gslu_index_alloc (f1->npts);
    gslu_index *sel2_temp = gslu_index_alloc (f2->npts);

    /* init forward index and backward index */
    gslu_index *fwd12_sel = gslu_index_alloc (n1); 
    gslu_index_set_all (fwd12_sel, nomatch);
    gslu_index *fwd21_sel = gslu_index_alloc (n2); 
    gslu_index_set_all (fwd21_sel, nomatch);
    gsl_vector *fwd21_score = gsl_vector_alloc (n2);

    if (simscore_minmax == VIS_UBC_SIMSCORE_MIN) /* this should check the type of feature */
        gsl_vector_set_all (fwd21_score, GSL_POSINF);
    else
        gsl_vector_set_all (fwd21_score, GSL_NEGINF);

    gsl_matrix *uv1All = vis_feature_get_uv_alloc (f1);
    gsl_matrix *uv2All = vis_feature_get_uv_alloc (f2);

    /* foreach feature in image1 */
    for (int i=0; i<n1; i++) {

        /* reset min distances */
        double key_dist_min = GSL_POSINF, key_dist_2nd_min = GSL_POSINF;

        /* foreach feature in image2 */
        for (int j=0; j<n2; j++) {

            /* compute sim score */
            double key_dist = GSL_POSINF;
            key_dist = vis_feature_get_simscore (f1, f2, i, j);

            if (simscore_minmax == VIS_UBC_SIMSCORE_MIN) { /* this should check the type of feature */
                if (key_dist < key_dist_min) {
                    key_dist_2nd_min = key_dist_min;
                    key_dist_min = key_dist;

                    /* check simA simB */
                    if (key_dist_min < simscore_thresh*key_dist_2nd_min)
                        gslu_index_set (fwd12_sel, i, j);
                    else
                        gslu_index_set (fwd12_sel, i, nomatch);

                }
                if (key_dist_min < key_dist && key_dist < key_dist_2nd_min) {
                    key_dist_2nd_min = key_dist;
                    if (key_dist_min > simscore_thresh*key_dist_2nd_min) /* check simA simB */
                        gslu_index_set (fwd12_sel, i, nomatch);
                }

            }
            else {
                /* this is case when feature key is harris */
            }

            /* simultaneously.. find the best score (min/max) & update fwd21_sel         */
            if (key_dist < gsl_vector_get (fwd21_score, j)) {
                gslu_index_set (fwd21_sel, j, i);
                gsl_vector_set (fwd21_score, j, key_dist);
            }

        }

    }

    /* Done. Now check fwd and bwd to populate sel1 sel2 */
    int idx_sel = 0;
    for (int j=0; j<n2; ++j) {
        if (gslu_index_get (fwd21_sel,j) != nomatch) {
            if ( gslu_index_get (fwd12_sel, gslu_index_get (fwd21_sel,j)) == j) {
                gslu_index_set (sel2_temp, idx_sel, j);
                idx_sel++;
            }
        }
    }
    int n_corr = idx_sel; ret = n_corr;

    for (int i=0; i<n_corr; ++i) {
        int j = gslu_index_get (fwd21_sel, gslu_index_get (sel2_temp, i));
        gslu_index_set (sel1_temp, i, j);
    }

    /* Get the non-zero indeces */
    gslu_index_view sel1_sub = gslu_index_subvector (sel1_temp, 0, n_corr);
    gslu_index_view sel2_sub = gslu_index_subvector (sel2_temp, 0, n_corr);

    /* Assign correspondence to output */
    *uv1 = gslu_matrix_selcol_alloc (uv1All, &sel1_sub.vector);
    *uv2 = gslu_matrix_selcol_alloc (uv2All, &sel2_sub.vector);

    /* clean up */
    gsl_matrix_free (uv1All);
    gsl_matrix_free (uv2All);
    gslu_index_free (sel1_temp);
    gslu_index_free (sel2_temp);
    gslu_index_free (fwd12_sel);
    gslu_index_free (fwd21_sel);
    gsl_vector_free (fwd21_score);

    return ret;
}

void
vis_feature_get_uv (const perllcm_van_feature_t *f, gsl_matrix *uv)
{

    // check if uv is allocated properly
    if (uv->size1 != 2 || uv->size2 != f->npts) {
        printf("Error in vis_feature_get_uv [feature]: uv need to be 2 x N matrix\n");
    }
    else {
        gsl_vector_float_const_view uf = gsl_vector_float_const_view_array (f->u, f->npts);
        gsl_vector_float_const_view vf = gsl_vector_float_const_view_array (f->v, f->npts);
        gsl_vector_view u = gsl_matrix_row (uv, 0);
        gsl_vector_view v = gsl_matrix_row (uv, 1);
        GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector_float, &uf.vector, gsl_vector, &u.vector);
        GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector_float, &vf.vector, gsl_vector, &v.vector);
    }
}

gsl_matrix *
vis_feature_get_uv_alloc (const perllcm_van_feature_t *f)
{
    // uv will be 2 x N matrix
    gsl_matrix *uv = gsl_matrix_alloc (2, f->npts);

    gsl_vector_float_const_view uf = gsl_vector_float_const_view_array (f->u, f->npts);
    gsl_vector_float_const_view vf = gsl_vector_float_const_view_array (f->v, f->npts);
    gsl_vector_view u = gsl_matrix_row (uv, 0);
    gsl_vector_view v = gsl_matrix_row (uv, 1);
    GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector_float, &uf.vector, gsl_vector, &u.vector);
    GSLU_VECTOR_TYPEA_TO_TYPEB (gsl_vector_float, &vf.vector, gsl_vector, &v.vector);

    return uv;
}

void
vis_feature_uv_collection_add (const gsl_matrix *uv, gslu_index *sel, gsl_matrix **uvc)
{
    if (!(*uvc)) { // null
        *uvc = gslu_matrix_selcol_alloc (uv, sel);
    }
    else { // there exist non zero uv collection
        size_t n_before = (*uvc)->size2;   // 2 x N matrix
        size_t n_add = sel->size;

        gsl_matrix *tmp = gsl_matrix_alloc (2, n_before+n_add);
        gsl_matrix_view uvc_prev = gsl_matrix_submatrix (tmp, 0, 0, (*uvc)->size1, (*uvc)->size2);
        gsl_matrix_memcpy (&uvc_prev.matrix, *uvc);
        gsl_matrix_free (*uvc);

        gsl_matrix_view uvc_add = gsl_matrix_submatrix (tmp, 0, n_before, 2, n_add);
        gslu_matrix_selcol (&uvc_add.matrix, uv, sel);

        *uvc = tmp;
    }

}


void
vis_feature_isel_collection_add (gslu_index *isel, gslu_index **iselc)
{
    if (!(*iselc)) { // null
        *iselc = gslu_index_alloc (isel->size);
        gslu_index_memcpy ((*iselc), isel);
    }
    else { // there exist non zero uv collection
        size_t n_before = (*iselc)->size;
        size_t n_add = isel->size;

        // concatanate sel
        gslu_index *tmp = gslu_index_alloc (n_before+n_add);
        gslu_index_view sel_prev = gslu_index_subvector (tmp, 0, (*iselc)->size);
        gslu_index_memcpy (&sel_prev.vector, *iselc);
        gslu_index_free (*iselc);

        gslu_index_view sel_add = gslu_index_subvector (tmp, n_before, n_add);
        gslu_index_memcpy (&sel_add.vector, isel);

        *iselc = tmp;
    }
}


void
vis_feature_uv_isel_collection_add (const gsl_matrix *uv, gslu_index *isel,
                                    gsl_matrix **uvc, gslu_index **iselc, size_t offset)
{
    vis_feature_uv_collection_add (uv, isel, uvc);
    gslu_index_add_constant (isel, (double) offset);
    vis_feature_isel_collection_add (isel, iselc);
}

void
vis_feature_get_uvc (const perllcm_van_feature_collection_t *fc, gsl_matrix *uvc)
{
    size_t total_npts = 0;

    for (size_t n=0; n<fc->ntypes; n++)
        total_npts += (&fc->f[n])->npts;

    if ((uvc->size1 == 2) && (uvc->size2 == total_npts)) {
        for (size_t n=0; n<fc->ntypes; n++) {
            perllcm_van_feature_t *f = &fc->f[n];
            size_t from = n == 0 ? 0 : (&fc->f[n-1])->npts;
            gsl_matrix_view uvc_view = gsl_matrix_submatrix (uvc, 0, from, 2, (&fc->f[n])->npts);
            vis_feature_get_uv (f, &uvc_view.matrix);
        }
    }
    else {
        printf ("check uvc size\n");
    }
}

gsl_matrix *
vis_feature_get_uvc_alloc (const perllcm_van_feature_collection_t *fc)
{
    size_t total_npts = 0;

    for (size_t n=0; n<fc->ntypes; n++)
        total_npts += (&fc->f[n])->npts;

    gsl_matrix *uvc = gsl_matrix_alloc (2, total_npts);


    size_t start_idx = 0;
    for (size_t n=0; n<fc->ntypes; n++) {
        perllcm_van_feature_t *f = &fc->f[n];    
        
        size_t length = (&fc->f[n])->npts;
        gsl_matrix_view uvc_view = gsl_matrix_submatrix (uvc, 0, start_idx, 2, length);
        vis_feature_get_uv (f, &uvc_view.matrix);

        start_idx += length;
    }

    return uvc;
}

double
vis_feature_get_scenedepth (perllcm_van_feature_t *f)
{
    perllcm_van_feature_user_depth_t *sp = malloc (sizeof (*sp));
    perllcm_van_feature_user_depth_t_decode (f->user, 0, f->usersize, sp);
    double z_mean = 1.0;

    if (sp->npts > 0) { // if scene prior found
        gsl_vector_float_const_view zf = gsl_vector_float_const_view_array (sp->mu_Z, sp->npts);
        z_mean = gsl_stats_float_mean (zf.vector.data, 1, sp->npts);
    }

    // clean up
    perllcm_van_feature_user_depth_t_destroy (sp);

    return z_mean;

}

void
_fprintf_surf (const perllcm_van_feature_t *f, const char *key_fullname)
{
    // prepare file operation
    FILE *file = fopen (key_fullname, "w");
    if (!file) {
        fprintf (stderr, "Warning: error opening %s, %s, line %d\n", key_fullname, __FILE__, __LINE__);
        return;
    }

    perllcm_van_feature_attr_cvsurf_t *attr = malloc (sizeof (*attr));
    perllcm_van_feature_attr_cvsurf_t_decode (f->attr, 0, f->attrsize, attr);

    fprintf (file, "%d %d\n", f->npts, f->keylen);
    for (int n=0; n<f->npts; n++) {
        fprintf (file, "%f %f %f %f\n", f->u[n], f->v[n], attr->size[n], attr->dir[n]);
        for (int j=0; j<f->keylen; j++ ) {
            float *key_i = f->keys[n];
            fprintf (file, " %f", key_i[j]);
        }
        fprintf (file, "\n");
    }

    perllcm_van_feature_attr_cvsurf_t_destroy (attr);

    if (fclose (file)!=0) {
        fprintf (stderr, "Warning: file close error, %s, line %d\n", __FILE__, __LINE__ );
        return;
    }
}

void
_fprintf_siftgpu (const perllcm_van_feature_t *f, const char *key_fullname)
{
    // prepare file operation
    FILE *file = fopen (key_fullname, "w");
    if (!file) {
        fprintf (stderr, "Warning: error opening %s, %s, line %d\n", key_fullname, __FILE__, __LINE__);
        return;
    }

    perllcm_van_feature_attr_siftgpu_t *attr = malloc (sizeof (*attr));
    perllcm_van_feature_attr_siftgpu_t_decode (f->attr, 0, f->attrsize, attr);

    fprintf (file, "%d %d\n", f->npts, f->keylen);
    for (int n=0; n<f->npts; n++) {
        fprintf (file, "%f %f %f %f\n", f->u[n], f->v[n], attr->s[n], attr->o[n]);
        for (int j=0; j<f->keylen; j++ ) {
            float *key_i = f->keys[n];
    	    fprintf (file, " %f", key_i[j]);
    	}
        fprintf (file, "\n");
    }

    perllcm_van_feature_attr_siftgpu_t_destroy (attr);

    if (fclose (file)!=0) {
        fprintf (stderr, "Warning: file close error, %s, line %d\n", __FILE__, __LINE__ );
        return;
    }
}

void
vis_feature_fprintf (const perllcm_van_feature_t *f, const char *key_fullname)
{
    switch (f->attrtype) {
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF:
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_SURFGPU:
        _fprintf_surf (f, key_fullname);
        break;
    case PERLLCM_VAN_FEATURE_T_ATTRTYPE_SIFTGPU:
        _fprintf_siftgpu (f, key_fullname);
        break;
    default:
        ERROR ("unknown attrtype %d", f->attrtype);
    }

}

void
vis_feature_2v_corrset_scale_alloc (const perllcm_van_feature_t *f1, const perllcm_van_feature_t *f2, 
                                    const gslu_index *sel1, const gslu_index *sel2, 
                                    gsl_vector **scale1, gsl_vector **scale2)
{
    assert (f1->attrtype == f2->attrtype);
    assert (sel1->size == sel2->size);

    int npts = sel1->size;
    *scale1 = gsl_vector_calloc (npts);
    *scale2 = gsl_vector_calloc (npts);

    if (f1->attrtype == PERLLCM_VAN_FEATURE_T_ATTRTYPE_SIFTGPU) {
        perllcm_van_feature_attr_siftgpu_t *attr1 = malloc (sizeof (*attr1));
        perllcm_van_feature_attr_siftgpu_t *attr2 = malloc (sizeof (*attr2));
        perllcm_van_feature_attr_siftgpu_t_decode (f1->attr, 0, f1->attrsize, attr1);
        perllcm_van_feature_attr_siftgpu_t_decode (f2->attr, 0, f2->attrsize, attr2);

        for (int i=0; i<npts; i++) {
            int ind1 = gslu_index_get (sel1, i); 
            int ind2 = gslu_index_get (sel2, i); 
            /* camera 1 */
            gsl_vector_set (*scale1, i, attr1->s[ind1]);
            /* camera 2 */
            gsl_vector_set (*scale2, i, attr2->s[ind2]);
       }

        /* clean up */
        perllcm_van_feature_attr_siftgpu_t_destroy (attr1);
        perllcm_van_feature_attr_siftgpu_t_destroy (attr2);
    }
    else {
        ERROR ("Unimplemented attrtype: %d", f1->attrtype);
    }
}

double *
vis_feature_2v_covimgpts_alloc (const gsl_vector *scale1, const gsl_vector *scale2)
{
    assert (scale1->size == scale2->size);

    int npts = scale1->size;
    int numprojs = 2*npts;      /* number of projections */
    int numcovelem = 4;         /* number of elements in covariance matrix */
    double *covimgpts = malloc (numprojs*numcovelem*sizeof(*covimgpts));

    for (int i=0; i<npts; i++) {
        covimgpts[i*8+0] = gsl_vector_get (scale1, i)*gsl_vector_get (scale1, i);
        covimgpts[i*8+1] = 0;
        covimgpts[i*8+2] = 0;
        covimgpts[i*8+3] = gsl_vector_get (scale1, i)*gsl_vector_get (scale1, i);
        covimgpts[i*8+4] = gsl_vector_get (scale2, i)*gsl_vector_get (scale2, i);
        covimgpts[i*8+5] = 0;
        covimgpts[i*8+6] = 0;
        covimgpts[i*8+7] = gsl_vector_get (scale2, i)*gsl_vector_get (scale2, i);
    }

    return covimgpts;
}
