#ifndef __PERLS_HAUV_TWOVIEW_REG_H__
#define __PERLS_HAUV_TWOVIEW_REG_H__

#include <opencv/cv.h>

#include "perls-lcmtypes/perllcm_van_calib_t.h"

#include "perls-vision/opencv_util.h"

/**
 * @defgroup PerlsHauvTwoviewReg 
 * @brief Homograpy related functions
 * @ingroup PerlsVision
 * @include: perls-vision/homography.h
 * @author Ayoung Kim
 * 
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/* methods to compute feature correspondence */
#define PERLS_HAUV_CORRESP_H 0
#define PERLS_HAUV_CORRESP_E 1 
#define PERLS_HAUV_CORRESP_F 2
#define PERLS_HAUV_CORRESP_MANUAL 3

typedef struct perls_hauv_tv_camera_params_t {
    perllcm_van_calib_t calib;
    vis_cvu_map_t *map;
    IplImage      *mask;
    IplImage      *mask_siftgpu;
} perls_hauv_tv_camera_params_t;

typedef struct perls_hauv_tv_reg_opts_t {
    int corresp_method;
    double sim_thresh;
} perls_hauv_tv_reg_opts_t;

/**
 * @brief Register two images with no prior guess of pose from odometry
 */
gsl_vector *
perls_hauv_tv_register_images (IplImage *img1, IplImage *img2, 
                               perls_hauv_tv_camera_params_t *camera,  
                               perls_hauv_tv_reg_opts_t *opts);

#ifdef __cplusplus
}
#endif

#endif // __PERLS_HAUV_TWOVIEW_REG_H__
