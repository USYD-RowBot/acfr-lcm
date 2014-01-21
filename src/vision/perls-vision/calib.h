#ifndef __PERLS_VISION_CALIB_H__
#define __PERLS_VISION_CALIB_H__

#include <opencv/cv.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-common/bot_util.h"
#include "perls-lcmtypes/perllcm_van_calib_t.h"

#ifdef __cplusplus
extern "C" {
#endif

// returns a perllcm_van_calib_t data structure as loaded from master.cfg
// using the specified key
// if cfg=NULL, it will call config_parse_default()
perllcm_van_calib_t
vis_calib_load_config (BotParam *param, const char *key);

// returns a perllcm_van_calib_t data structure as loaded from
// a matlab camera calibration toolbox m-file
perllcm_van_calib_t
vis_calib_load_matlab (const char *filename);

// returns an openCV view of the calibration structure
// note: calib must remain in scope
typedef struct vis_calib_view_cv vis_calib_view_cv_t;
struct vis_calib_view_cv {
    CvSize imageSize;
    CvMat K;
    CvMat Kinv;
    CvMat kc;
};

vis_calib_view_cv_t
vis_calib_view_cv (perllcm_van_calib_t *calib);

typedef struct vis_calib_const_view_cv vis_calib_const_view_cv_t;
struct vis_calib_const_view_cv {
    const CvSize imageSize;
    const CvMat K;
    const CvMat Kinv;
    const CvMat kc;
};

vis_calib_const_view_cv_t
vis_calib_const_view_cv (const perllcm_van_calib_t *calib);


// returns a gsl view of the calibration structure
// note: calib must remain in scope
typedef struct vis_calib_view_gsl vis_calib_view_gsl_t;
struct vis_calib_view_gsl {
    int imageSize[2];
    gsl_matrix_view K;
    gsl_matrix_view Kinv;
    gsl_vector_view kc;
};

vis_calib_view_gsl_t
vis_calib_view_gsl (perllcm_van_calib_t *calib);


typedef struct vis_calib_const_view_gsl vis_calib_const_view_gsl_t;
struct vis_calib_const_view_gsl {
    const int imageSize[2];
    gsl_matrix_const_view K;
    gsl_matrix_const_view Kinv;
    gsl_vector_const_view kc;
};

vis_calib_const_view_gsl_t
vis_calib_const_view_gsl (const perllcm_van_calib_t *calib);


#ifdef __cplusplus
}
#endif

#endif //__PERLS_VISION_CALIB_H__
