#ifndef __PERLS_VISION_HIGHGUI_UTIL_H__
#define __PERLS_VISION_HIGHGUI_UTIL_H__

#include <opencv/cv.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get manual feature correspondence between two images
 *
 * @param i1 Image 1
 * @param i2 Image 2
 * @param uv1 Output pixel locations in image 1(2 x n)
 * @param uv2 Output pixel locations in image 2 (2 x n)
 * @param n Number of desired corresponding points
 *
 * @note OpenCV HighGui callbacks only support integer click locations.  For highest
 * accuracy, there is no downsampling of i1 and i2
 *
 */
void
vis_hgu_manual_corresp (const IplImage *i1,
                        const IplImage *i2,
                        gsl_matrix *uv1,
                        gsl_matrix *uv2,
                        int n);


#ifdef __cplusplus
}
#endif

#endif //__PERLS_VISION_HIGHGUI_UTIL_H__
