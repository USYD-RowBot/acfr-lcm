#ifndef __PERLS_DC1394_CAMERA_H__
#define __PERLS_DC1394_CAMERA_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <dc1394/dc1394.h>
#include "perls-common/error.h"

#define PERLS_DC1394_CAMERA_REGISTER_FRAME_TIMESTAMP 0x12F8 
#define PERLS_DC1394_CAMERA_REGISTER_FRAME_RATE_CONTROL 0x83C 
#define PERLS_DC1394_CAMERA_REGISTER_ABS_FRAME_RATE_CONTROL 0x968
#define PERLS_DC1394_CAMERA_REGISTER_BAYER_TILE_MAPPING 0x1040

#define CYCLE_TIMER_MAX_USEC  ((0x7f+1)*1000000)
#define CYCLE_TIMER_MAX_TICS  (0x7f+1)

#ifdef __cplusplus
extern "C" {
#endif

/** EXAMPLE CONFIG FILE
 *
 *  dc1394-camera {
 *      # Note: run with '-l' or '-q' to list cameras on bus or query individual cameras for modes.
 *      # Also, each field can be completed with an array (one for each cam in cam_id), or 
 *      # individually and the setting will be applied to all cameras.
 *
 *      lcm_channel = "IMAGE";                              # leave out/comment for IMAGEX
 *      cam_id = ["b09d0100a9ad98"];                        # camera id must be specified
 *      video_mode = "DC1394_VIDEO_MODE_1280x960_MONO8";    # leave out/comment for default
 *      #video_mode = "DC1394_VIDEO_MODE_FORMAT7_0";        # leave out/comment for default
 *      speed1394b = 1;     # using FireWire 800?           # leave out/comment for FireWire 400
 *
 *      framerate = 10;                                     # leave out/comment for maximum
 *      #brightness = 0;                                    # leave out/comment for auto
 *      #exposure = 0;                                      # leave out/comment for auto
 *      #gamma = 0;                                         # leave out/comment for off
 *      shutter = .006;                                     # leave out/comment for auto
 *      #gain = 0;                                          # leave out/comment for auto
 *      #whitebalance_blue = 850;                           # leave out/comment for auto
 *      #whitebalance_red = 500;                            # leave out/comment for auto
 *
 *      # FORMAT7 video mode only:
 *      #color_code = "DC1394_COLOR_CODING_MONO8";          # leave out/comment for default
 *      #width = 400;        # image width for ROI          # leave out/comment for max width
 *      #height = 516;       # image height for ROI         # leave out/comment for max height
 *      #startx = 0;         # starting x position of ROI   # leave out/comment for auto centering
 *      #starty = 0;         # starting y position of ROI   # leave out/comment for auto centering
 *  }
 */


typedef struct _PERLS_dc1394_camera PERLS_dc1394_camera_t;
struct _PERLS_dc1394_camera {
    dc1394_t *dc1394;
    dc1394camera_t *cam;
};

dc1394error_t
perls_dc1394_camera_parse_embedded_timestamp (PERLS_dc1394_camera_t *perls_cam, uint8_t *image, uint64_t *timestamp);

/**
 * Lists all cameras on the bus by camera_id
 */
dc1394error_t
perls_dc1394_camera_list (void);

/**
 * Queries the cam_id for supported video modes/framerates.
 */
dc1394error_t
perls_dc1394_camera_query (const char *cam_id);    

/**
 * Cleans and deallocates all bandwidth for all cameras
 */
dc1394error_t
perls_dc1394_camera_clean (void);

/**
 * Verifies that cam_id is on the bus and initializes it.
 */
PERLS_dc1394_camera_t * 
perls_dc1394_camera_init (const char *cam_id);

/**
 * Clears previously allocated isochronous bandwidth and channels.
 *     -this is essential if a dc1394 camera is not properly closed
 */
dc1394error_t
perls_dc1394_camera_cleanup_iso (dc1394camera_t *cam);
 
/**
 * typedef enum{
 *   DC1394_OPERATION_MODE_LEGACY = 480,
 *   DC1394_OPERATION_MODE_1394B
 * }dc1394operation_mode_t; 
 *  As the IEEE1394 speeds were increased with IEEE-1394b specifications, a new type
 *  of control is necessary when the camera is operating in iso speeds over 800Mbps. 
 *  If you wish to use a 1394b camera you may need to switch the operation mode to 1394b.
 *  Legacy mode refers to speeds less than 400Mbps. 
 */
dc1394error_t 
perls_dc1394_camera_set_operation_mode (PERLS_dc1394_camera_t *perls_cam, dc1394operation_mode_t opMode);

/**
 * typedef enum {
 *   DC1394_ISO_SPEED_100= 0,
 *   DC1394_ISO_SPEED_200,
 *   DC1394_ISO_SPEED_400,
 *   DC1394_ISO_SPEED_800,
 *   DC1394_ISO_SPEED_1600,
 *   DC1394_ISO_SPEED_3200
 * }dc1394speed_t;
 *
 *These definitions specify the isochronous speed at which the transmission should occur. 
 *100 stands for 100Mbps, 400 for 400Mbps,... Note that speeds of 1600 and 3200 Mbps are 
 *for future use only: they are not supported by any hardware or software at the moment.
 */
dc1394error_t 
perls_dc1394_camera_set_iso_speed (PERLS_dc1394_camera_t *perls_cam, dc1394speed_t speed);

/**
 * Sets the DC1394 video mode.
 */
dc1394error_t
perls_dc1394_camera_set_video_mode (PERLS_dc1394_camera_t *perls_cam, dc1394video_mode_t vidMode);    

/**
 * Checks the currently specified DC1394 color code.
 */
dc1394error_t
perls_dc1394_camera_get_color_coding (PERLS_dc1394_camera_t *perls_cam, dc1394color_coding_t *color);

/**
 * Checks the Bayer pattern currently in use
 */
dc1394error_t
perls_dc1394_camera_get_bayer_pattern (PERLS_dc1394_camera_t *perls_cam, dc1394color_filter_t *filter);

/**
 * Sets the framerate of the camera. 
 * If set frame rate is more than the transfer rate then the framerate is governed by the 
 * transfer rate. Note: only use this function on non-scalable video modes.
 */
dc1394error_t 
perls_dc1394_camera_set_frame_rate (PERLS_dc1394_camera_t *perls_cam, float framerate);

/**
 * Sets the ROI for scalable video modes (FORMAT7)
 */
dc1394error_t
perls_dc1394_camera_set_roi (PERLS_dc1394_camera_t *perls_cam, dc1394color_coding_t color,
        double framerate, int left, int top, int width, int height);

/**
 * Starts the video transmission, i.e. data starts transferring and image buffer
 * starts getting filled
 */
dc1394error_t
perls_dc1394_camera_start_video_transmission (PERLS_dc1394_camera_t *perls_cam);

/**
 * Stops the video transmission
 */
dc1394error_t
perls_dc1394_camera_stop_video_transmission (PERLS_dc1394_camera_t *perls_cam);

/**
 * Cleans up.
 */
void
perls_dc1394_camera_camera_free(PERLS_dc1394_camera_t *perls_cam);

/**
 * These functions modify the camera settings
 */
/***************************************************************************************/
dc1394error_t 
perls_dc1394_camera_set_brightness_auto (PERLS_dc1394_camera_t *perls_cam);

dc1394error_t
perls_dc1394_camera_set_brightness_manual (PERLS_dc1394_camera_t* perls_cam, double value);

dc1394error_t
perls_dc1394_camera_set_brightness (PERLS_dc1394_camera_t* perls_cam, int manual, double value);

dc1394error_t
perls_dc1394_camera_set_exposure_manual (PERLS_dc1394_camera_t *perls_cam, double value);

dc1394error_t
perls_dc1394_camera_set_exposure_mode (PERLS_dc1394_camera_t *perls_cam, char mode[64]);

dc1394error_t
perls_dc1394_camera_set_exposure (PERLS_dc1394_camera_t* perls_cam, int manual, double value);

dc1394error_t
perls_dc1394_camera_set_gamma_manual (PERLS_dc1394_camera_t *perls_cam, double value);

dc1394error_t
perls_dc1394_camera_set_gamma_mode (PERLS_dc1394_camera_t *perls_cam, char mode[64]);

dc1394error_t
perls_dc1394_camera_set_gamma (PERLS_dc1394_camera_t* perls_cam, int manual, double value);

dc1394error_t
perls_dc1394_camera_set_gain_manual (PERLS_dc1394_camera_t *perls_cam, int value);

dc1394error_t
perls_dc1394_camera_set_gain_mode (PERLS_dc1394_camera_t *perls_cam, char mode[64]);

dc1394error_t
perls_dc1394_camera_set_gain (PERLS_dc1394_camera_t* perls_cam, int manual, double value);

dc1394error_t
perls_dc1394_camera_set_shutter_manual (PERLS_dc1394_camera_t *perls_cam, double value);

dc1394error_t
perls_dc1394_camera_set_shutter_mode (PERLS_dc1394_camera_t *perls_cam, char mode[64]);

dc1394error_t
perls_dc1394_camera_set_shutter (PERLS_dc1394_camera_t* perls_cam, int manual, double value);

dc1394error_t
perls_dc1394_camera_set_whitebalance_auto (PERLS_dc1394_camera_t *perls_cam);

dc1394error_t
perls_dc1394_camera_set_whitebalance_manual (PERLS_dc1394_camera_t *perls_cam, int valueBlue, int valueRed);

dc1394error_t
perls_dc1394_camera_set_whitebalance (PERLS_dc1394_camera_t* perls_cam, int manual, int blue, int red);
/******************************************************************************************/

/**
 * Enables embedding of timestamp into first few bytes of the image data.
 */
dc1394error_t
perls_dc1394_camera_embed_timestamp_into_frame (PERLS_dc1394_camera_t *perls_cam);


/**
 * These functions are useful mode/enum conversions
 */
/***************************************************************************************/
char *
dc1394_videomode_int_to_string (int);
char *
dc1394_colorcode_int_to_string (int);
char *
dc1394_framerate_int_to_string (int);
double
dc1394_framerate_int_to_double (int);
int
dc1394_videomode_string_to_int (const char *);
int
dc1394_colorcode_string_to_int (const char *);
int
dc1394_framerate_string_to_int (const char *);
/******************************************************************************************/


#ifdef __cplusplus
}
#endif

#endif //__PERLS_DC1394_CAMERA_H__
