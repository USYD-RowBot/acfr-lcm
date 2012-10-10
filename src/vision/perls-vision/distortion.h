#ifndef __PERLS_VISION_DISTORTION_H__
#define __PERLS_VISION_DISTORTION_H__

#include <opencv/cv.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-lcmtypes/perllcm_van_calib_t.h"

#include "opencv_util.h"

#ifdef __cplusplus
extern "C" {
#endif

/* computes a forward radial distortion model on [2 x N] undistorted src points, the points dst are distorted
 * - src points represent uv_c pixel coordinates
 * - dst points represent uv pixel coordinates (src and dst can overlap in memory)
 * - K is the [3x3] camera calibration matrix
 * - distCoeffs [5x1] vector of radial and tangential distortion coefficients [k1,k2,p1,p2,k3]
 *
 * NOTE: to work with normalized xy src points, simply set the camera calibration matrix to identity or NULL
 */
void
vis_distort_pts_radial (const gsl_matrix *src, gsl_matrix *dst, const gsl_matrix *K, const gsl_vector *distCoeffs);

/* computes an inverse radial distortion model on [2 x N] distorted src points, the points dst are undistorted
 * - src points represent uv pixel coordinates
 * - dst points represent uv_c pixel coordinates (src and dst can overlap in memory)
 * - K is the [3x3] camera calibration matrix
 * - distCoeffs [5x1] vector of radial and tangential distortion coefficients [k1,k2,p1,p2,k3]
 *
 * NOTE: to work with normalized xy src points, simply set the camera calibration matrix to identity or NULL
 */
void
vis_undistort_pts_radial (const gsl_matrix *src, gsl_matrix *dst, const gsl_matrix *K, const gsl_vector *distCoeffs);

// returns a non-zero image mask associated with an unwarped image
IplImage *
vis_undistort_mask (const perllcm_van_calib_t *calib);


// returns a mapping from distorted to undistorted pixel coordinates used for warping an image with cvRemap()
vis_cvu_map_t *
vis_undistort_map (const perllcm_van_calib_t *calib);


// returns a mapping from undistored to distorted pixel coordinates used for warping an image with cvRemap()
vis_cvu_map_t *
vis_distort_map (const perllcm_van_calib_t *calib);


#ifdef __cplusplus
}
#endif

#endif //__PERLS_VISION_DISTORTION_H__
