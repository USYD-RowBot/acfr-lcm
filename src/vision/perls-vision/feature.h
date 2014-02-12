#ifndef __PERLS_VISION_FEATURE_H__
#define __PERLS_VISION_FEATURE_H__

#include <stdint.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <opencv/cv.h>

#include "perls-lcmtypes/perllcm_pose3d_t.h"
#include "perls-lcmtypes/perllcm_van_feature_t.h"
#include "perls-lcmtypes/perllcm_van_feature_collection_t.h"
#include "perls-lcmtypes/perllcm_van_feature_user_depth_t.h"
#include "perls-lcmtypes/perllcm_van_feature_attr_cvsurf_t.h"
#include "perls-lcmtypes/perllcm_van_feature_attr_siftgpu_t.h"

#include "perls-math/gsl_util_index.h"

/**
 * @defgroup PerlsVisionFeature feature
 * @brief perls-vision feature libraries
 * @ingroup PerlsVision
 * 
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#define VIS_UBC_SIMSCORE_MIN 0
#define VIS_UBC_SIMSCORE_MAX 1

/**
 * @breif  allocate a new feature_collection_t
 *
 * @param  utime timestamp of camera event
 * @param  channel LCM channel name of camera event
 *
 * @return a new feature_collection_t
 */
perllcm_van_feature_collection_t *
vis_feature_collection_alloc (int64_t utime, const char *channel);

/**
 * @brief adds a feature_t to the feature_collection_t
 * @note fc will point at f, and will therefore be owned by fc, and
 * subsequently destroyed when perllcm_van_feature_collection_t_destroy() is called.  
 * Do not call perllcm_van_feature_t_destroy() on f once it has been added to fc.
 * If you need to do so, then pass a copy of f using perllcm_van_feature_t_copy().
 */
void
vis_feature_collection_add (perllcm_van_feature_collection_t *fc, perllcm_van_feature_t *f);

/**
 * @brief Populates fc.scene_prior and fc.f[n].user fields with depth prior, should be called AFTER
 * all features are extracted and added via vis_feature_collection_add()
 *
 * @param XYZ_l 3xN array of points in local-level frame
 * @param bid   optional N-vector of beam id's (i.e., 1,2,3,4), otherwise set to NULL
 * @param x_lc  [x,y,z,r,p,h] pose of camera w.r.t. local-level (set to NULL to use identity transform)
 * @param calib camera calibration
 */
void
vis_feature_collection_scene_prior (perllcm_van_feature_collection_t *fc, 
                                    const gsl_matrix *XYZ_l,
                                    const gsl_vector *bid,
                                    const gsl_vector *x_lc, 
                                    const perllcm_van_calib_t *calib);

/**
 * @brief vis_feature wrapper around opencv's cvExtractSURF. npts_max limits the number of feature
 * points to the npts_max strongest based upon the hessian response.
 * to return all detected features, set ntps_max < 0;
 *
 * @return NULL on error;
 */
perllcm_van_feature_t *
vis_feature_cvsurf (const IplImage *image, const IplImage *mask, CvSURFParams params, int npts_max);


/**
 * @brief vis_feature wrapper around siftgpu client.
 * 
 * @param image          must be 8-bit gray scale
 * @param mask           image mask, otherwise specify NULL
 * @param server_ipaddr  dotted quad address of siftgpu-server host, specify NULL to use default address
 * @param server_port    port address of siftgpu-server, set to -1 to use default port
 * @param npts_max       limits the number of feature points returned
 *
 * @return NULL on error
 */
perllcm_van_feature_t *
vis_feature_siftgpu (const IplImage *image, const IplImage *mask, const char *server_ipaddr, int server_port, int npts_max);

/**
 * @brief vis_feature wrapper around surfgpu client.
 *
 * @param image           must be 8-bit gray scale
 * @param mask            image mask, otherwise specify NULL
 * @param server_ipaddr   dotted quad address of surfgpu-server host, specify NULL to use default address
 * @param server_port     port address of surfgpu-server, set to -1 to use default port
 * @param npts_max        limits the number of feature points returned
 *
 * @return NULL on error
 */
perllcm_van_feature_t *
vis_feature_surfgpu (const IplImage *image, const IplImage *mask, const char *server_ipaddr, int server_port, int npts_max);


typedef struct vis_feature_harris_params vis_feature_harris_params_t;
struct vis_feature_harris_params
{
    double qualityLevel;    /* Multiplier for the max/min eigenvalue; 
                             * specifies the minimal accepted quality of image corners 
                             */

    double minDistance;     /* Limit, specifying the minimum possible distance between the returned corners;
                             * Euclidian distance is used
                             */

    int blockSize;          /* Size of the averaging block, passed to the underlying 
                             * CornerMinEigenVal or CornerHarris used by the function
                             */

    double k;               /* Free parameter of Harris detector; 
                             * used only if ($\texttt{useHarris} != 0$)
                             * default = 0.04, matlab van also uses 0.04.
                             * det(M) = k trace(M)^2
                             */
};

/**
 * @brief detect harris feature and describe with zernike moments
 * @note This uses same args as in cvGoodFeaturesToTrack
 * 
 * @param image          input image
 * @param mask           image mask, otherwise specify NULL
 * @param harris_param   harris parameters as in vis_feature_harris_params_t
 * @param featpath       patch generated using vis_feature_harris_patch()
 * @param x_lc           camera pose in local coordinate
 * @param K              calibration matrix
 *
 * @return all detected features, set ntps_max < 0;
 */
perllcm_van_feature_t *
vis_feature_cvharris (const IplImage *image, const IplImage *mask, 
                      vis_feature_harris_params_t *harris_params, 
                      gsl_matrix *featpatch,
                      perllcm_pose3d_t x_lc, gsl_matrix *K,
                      int npts_max);

/** 
 * @brief apply patch around a feature point u,v
 * and store the pixel information at each patch pixels
 * Hinf to warp image in N-E coordinate
 *
 * @param image          input image
 * @uv                   feature point
 * @Hinf                 infinite homography to warp image in N-E coordinate
 */
void
vis_feature_harris_patch (const IplImage *image, const double u, const double v, gsl_matrix *Hinf,
                          gsl_matrix *featpatch, gsl_vector *patch_col);

/** 
 * initialize a retangular patch sampler centered at (0,0) with size of (2w+1) by (2w+1)
 * for the given window size
 * return 2xN matrix = [xsamples;
 *                      ysamples;];
 */
gsl_matrix *
vis_feature_patch_sampler_alloc (size_t window_size);


 
/** calculate smatrix (n1 x n2) matrix
 * sij = similarity score ith feature key 1 and jth feature key 2 (sij != sji)
 * matrix version of vis_feature_get_simscore()
 */
gsl_matrix *
vis_feature_get_smatrix (const perllcm_van_feature_t *fi, const perllcm_van_feature_t *fj);

/** calculate similarity score sij
 * sij = similarity score ith feature key of fi and jth feature key of fj (sij != sji)
 */
double 
vis_feature_get_simscore (const perllcm_van_feature_t *fi, const perllcm_van_feature_t *fj, int i, int j);


/** 
 * get smatrix from two set of feature keys
 */
void 
vis_feature_get_smatrix_gsl (gsl_matrix *key1, gsl_matrix *key2, gsl_matrix* smatrix);

/** 
 * get simscore from two feature keys
 */
double 
vis_feature_get_simscore_gsl (const gsl_vector* key1, const gsl_vector* key2);

double 
vis_feature_get_simscore_gsl_float (const gsl_vector_float *key1, const gsl_vector_float *key2);

/**
 * @brief Lowe's suggested appearance-based descriptor matching algorith.  See
 * http://www.vlfeat.org/mdoc/VL_UBCMATCH.html
 *
 * @param f1 Features from camera 1
 * @param f2 Feature from camera 2
 * @param uv1 Output pixel locations from camera 1.  Must be freed by user.
 * @param uv2 Output pixel locations from camera 2.  Must be freed by user.
 * @param simAB_thresh 2nd closest similarity score threshold.  For SIFT, Lowe recommends 0.6.
 * @param simscore_minmax Either VIS_UBC_SIMSCORE_MIN (for SIFT) or VIS_UBC_SIMSCORE_MAX (for Harris)
 *
 * @return Number of corresponding feature descriptors
 *
 * @note UBC supposedly stands for University of British Columbia
 */
int
vis_feature_ubc_match_alloc (const perllcm_van_feature_t *f1, const perllcm_van_feature_t *f2,
                             gsl_matrix **uv1, gsl_matrix **uv2, double simAB_thresh, int simscore_minmax);

/**  decode uv pixel points from feature structure
 */
void
vis_feature_get_uv (const perllcm_van_feature_t *fi, gsl_matrix *uv);

/**
 * allocation version of vis_feature_get_uv()
 */
gsl_matrix *
vis_feature_get_uv_alloc (const perllcm_van_feature_t *fi);

/** decode uv pixel points from each feature in feature collection
 * and stack up to return uvc
 */
void
vis_feature_get_uvc (const perllcm_van_feature_collection_t *fc, gsl_matrix *uvc);

/**
 * allocation version of vis_feature_get_uv()
 */
gsl_matrix *
vis_feature_get_uvc_alloc (const perllcm_van_feature_collection_t *fc);

/*  take a subset (sel) of uv and add into existing uvc
 *  if uvc is null, uvc = gslu_matrix_selcol_alloc (uvi, seli);
 */
void 
vis_feature_uv_collection_add (const gsl_matrix *uv, gslu_index *sel, gsl_matrix **uvc);

/*  add sel into existing sel collection
 *  if sel collection is null, memcpy the current sel into collection
 */
void
vis_feature_isel_collection_add (gslu_index *isel, gslu_index **iselc);

/* add uv, sel and collection number all at once
 */
void
vis_feature_uv_isel_collection_add (const gsl_matrix *uv, gslu_index *isel,
                                    gsl_matrix **uvc, gslu_index **iselc, size_t offset);

/* get avg of scene depth by decoding feature_t
 */
double
vis_feature_get_scenedepth (perllcm_van_feature_t *f);

/**
 * @brief fprintf feature to disk that is matlab van compatible
 * 
 * @param f         perllcm_van_feature_t to be written
 * @key_filename    full filename of the keyfile
 */
void
vis_feature_fprintf (const perllcm_van_feature_t *f, const char *key_filename);

/**
 * @brief get a vector of feature scales according to the inliner indeces
 * 
 * @param f1 feature from camera 1
 * @param f2 feature from camera 2
 * @param sel1 inliers from camera 1
 * @param sel2 inliers from camera 2
 * @param scale1 vector to assign scales from camera 1
 * @param scale2 vector to assign scales from camera 2
 */
void
vis_feature_2v_corrset_scale_alloc (const perllcm_van_feature_t *f1, const perllcm_van_feature_t *f2, 
                                    const gslu_index *sel1, const gslu_index *sel2, 
                                    gsl_vector **scale1, gsl_vector **scale2);

/**
 * @brief isotropic estimate of feature covariance of each feature, and places into
 * libsba-compatible double array
 *
 * @detail approximates the method from Zeisl et. al (Estimation of Location Uncertianty
 * for Scale Invariant Feature Points).  For a feature, simply square the scale of the
 * feature and multiply by eye(2).
 *
 * @param scale1 vector of scales from camera 1
 * @param scale2 vector of scales from camera 1
 */
double *
vis_feature_2v_covimgpts_alloc (const gsl_vector *scale1, const gsl_vector *scale2);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif //__PERLS_VISION_FEATURE_H__
