#ifndef __BUMBLEBEEXB3_H__
#define __BUMBLEBEEXB3_H__

#define USE_JPEG

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <dc1394/dc1394.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <dc1394/log.h>
#include <dc1394/utils.h>
#include <dc1394/register.h>
#include <dc1394/control.h>


/*#include <jpeglib.h>
#include <jerror.h>
#include "memsrc.h"
#define BB3_REGISTER_FRAME_TIMESTAMP 0x12F8 
#define BB3_REGISTER_FRAME_RATE_CONTROL 0x83C 
#define BB3_REGISTER_JPEG_QUALITY_CONTROL 0x1E80 //0x1A20
#define BB3_REGISTER_ABS_FRAME_RATE_CONTROL 0x968
*/
#ifdef __cplusplus
extern "C" {
#endif

typedef struct _BB3camera
{
    dc1394_t *dc1394;
    dc1394camera_t *camera;
    dc1394color_filter_t bayerTile;
    int bColor;
    unsigned int nRows;
    unsigned int nCols;
    unsigned int nBytesPerPixel;
} BB3camera_t;



typedef struct _BB3StreamHeaderInfo_
{
  uint32_t versionNum;
  uint32_t framerate;
  uint32_t headSerialNum;
  uint32_t paddingBlock;
  //DataFormat_t dataFormat;
  //Resolution_t resolution;
  //BayerPattern_t bayerPattern;
  uint32_t configDataSize;
  uint32_t numImages;
  uint32_t numIndex_M;
  uint32_t interval_K;
  uint32_t streamDataOffset;
  uint32_t gpsDataOffset;
  uint32_t gpsDataSize;
} BB3StreamHeaderInfo_t;


dc1394error_t
getBayerTile (dc1394camera_t *camera, dc1394color_filter_t *bayerPattern);

dc1394error_t
getSensorInfo (dc1394camera_t *camera, int *pbColor, unsigned int *pnRows, unsigned int *pnCols);

void
dc1394_deinterlace_green (unsigned char *src, unsigned char *dest, unsigned int width, unsigned int height);

BB3camera_t *
bb3_init (void);

/*Sets the operational mode of the BB3*/
dc1394error_t 
bb3_set_operation_mode (BB3camera_t *bb3camera, dc1394operation_mode_t mode);

/*This function sets the isochronous speed */
dc1394error_t
bb3_set_iso_speed (BB3camera_t *bb3camera, dc1394speed_t speed);

/**
 *This function sets the jpeg quality of the camera. 
 *quality = 1 to 100
*/
dc1394error_t
bb3_set_jpeg_quality (BB3camera_t *bb3camera, uint8_t quality);

/*This function sets video mode */
dc1394error_t
bb3_set_video_mode (BB3camera_t *bb3camera);



/**
 * This function starts the video transmission, i.e. data starts transferring and image buffer
 * starts getting filled
 */
dc1394error_t
bb3_start_video_transmission (BB3camera_t *bb3camera);

/**
 * This function writes the bumblebeexb3 stream header info into the file
 */
void
bb3_write_stream_header_info (FILE *fp, BB3StreamHeaderInfo_t header);

/**
 * This function writes the calib data into the file stream
 */
void
bb3_write_calib_data (FILE *fp, const char *calib_file_name);

/**
 * This function frees the parameter
 */
void
bb3_camera_free (BB3camera_t *bb3camera);

/**
 * This function stops the video transmission
 */
dc1394error_t
bb3_stop_video_transmission (BB3camera_t *bb3camera);


void
extractImagesMono (BB3camera_t   *bb3camera, 
		   unsigned char *pucDeInterleaved,
		   unsigned char **ppucRightMono8,
		   unsigned char **ppucLeftMono8,
		   unsigned char **ppucCenterMono8);
void
extractImagesColor (BB3camera_t *bb3camera, 
		    dc1394bayer_method_t bayerMethod,
		    unsigned char *pucDeInterleaved,
		    unsigned char *pucRGB,
		    unsigned char *pucGreen,
		    unsigned char **ppucRightRGB,
		    unsigned char **ppucLeftRGB,
		    unsigned char **ppucCenterRGB); 

dc1394error_t
bb3_set_frame_rate (BB3camera_t *bb3camera, float framerate);

dc1394error_t
bb3_setup (BB3camera_t *bb3camera);

#ifdef __cplusplus
}
#endif

#endif //__BUMBLEBEEXB3_H__
