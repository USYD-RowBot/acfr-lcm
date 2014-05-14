#ifndef __PERLS_VISION_FEATUREGPU_H__
#define __PERLS_VISION_FEATUREGPU_H__

#include <opencv/cv.h>

#include "perls-lcmtypes/perllcm_van_feature_t.h"

#include "perls-common/magic.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VIS_SIFTGPU_TCP_PORT     31001
#define VIS_SIFTGPU_SHM_KEY      magic32 ('S','I','F','T')

#define VIS_SURFGPU_TCP_PORT     0xBEEF
#define VIS_SURFGPU_SHM_KEY      magic32 ('S','U','R','F')

#define VIS_FEATUREGPU_TYPE_SIFT      1
#define VIS_FEATUREGPU_TYPE_SURF      2

#define VIS_SURFGPU_MAX_FEATURES      10000
#define VIS_SIFTGPU_MAX_FEATURES      1000

/* siftgpu client API for interacting with siftgpu-server
 * image = must be 8-bit gray scale
 * mask = image mask, otherwise specify NULL
 * server_ipaddr = dotted quad address of siftgpu-server host, specify NULL to use default address
 * server_port = port address of siftgpu-server, set to -1 to use default port
 * npts_max = limits the number of feature points returned
 *
 * returns NULL on error
 */
perllcm_van_feature_t *
vis_siftgpu_client (const IplImage *image, const IplImage *mask, const char *server_ipaddr, int server_port, int npts_max);

/* surfgpu client API for interacting with surfgpu-server
 * image = must be 8-bit gray scale
 * mask = image mask, otherwise specify NULL
 * server_ipaddr = dotted quad address of surfgpu-server host, specify NULL to use default address
 * server_port = port address of surfgpu-server, set to -1 to use default port
 * npts_max = limits the number of feature points returned
 *
 * returns NULL on error
 */
perllcm_van_feature_t *
vis_surfgpu_client (const IplImage *image, const IplImage *mask, const char *server_ipaddr, int server_port, int npts_max);

#ifdef __cplusplus
}
#endif

#endif //__PERLS_VISION_FEATUREGPU_H__
