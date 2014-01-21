#ifndef __LADYBUG3_H__
#define __LADYBUG3_H__

#define USE_JPEG

#include <dc1394/dc1394.h>

#define LB3_CAM_MODEL "Ladybug3 LD3-20S4C"

#define LB3_REGISTER_FRAME_TIMESTAMP            0x12F8 
#define LB3_REGISTER_FRAME_RATE_CONTROL         0x83C 
#define LB3_REGISTER_JPEG_QUALITY_CONTROL       0x1E80 //0x1A20
#define LB3_REGISTER_ABS_FRAME_RATE_CONTROL     0x968
#define LB3_REGISTER_SHUTTER_CONTROL            0x81C 
#define LB3_REGISTER_ABS_SHUTTER_MIN_VAL        0x910
#define LB3_REGISTER_ABS_SHUTTER_MAX_VAL        0x914
#define LB3_REGISTER_ABS_SHUTTER_VAL            0x918
#define LB3_REGISTER_AUTO_SHUTTER_RANGE         0x1098
#define LB3_REGISTER_AE_STATS_MASK              0x1E90

#define CYCLE_TIMER_TO_USEC(cycle,secmask) (            \
        (((uint32_t)cycle >> 25) & secmask) * 1000000 + \
        (((uint32_t)cycle & 0x01fff000) >> 12) * 125 +  \
        ((uint32_t)cycle & 0x00000fff) * 125 / 3072)

#define CYCLE_TIMER_MAX_USEC(secmask)  ((secmask+1)*1000000)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * This struct contains the structures from libdc1394 library
 * used to operate the camera
 */
typedef struct LB3camera LB3camera_t;
struct LB3camera {
    dc1394_t *dc1394;
    dc1394camera_t *cam;
};

/**
 * format of the dc1394video_frame_t data structure
 * video frame structure 
 */
//typedef struct __dc1394_video_frame
//{
//  unsigned char          * image;                 /* the image. May contain padding data too (vendor specific) */
//  uint32_t                 size[2];               /* the image size [width, height] */
//  uint32_t                 position[2];           /* the WOI/ROI position [horizontal, vertical] == [0,0] for full frame */
//  dc1394color_coding_t     color_coding;          /* the color coding used. This field is valid for all video modes. */
//  dc1394color_filter_t     color_filter;          /* the color filter used. This field is valid only for RAW modes and IIDC 1.31 */
//  uint32_t                 yuv_byte_order;        /* the order of the fields for 422 formats: YUYV or UYVY */
//  uint32_t                 bit_depth;             /* the number of bits per pixel. The number of grayscale levels is 2^(this_number) */
//  uint32_t                 stride;                /* the number of bytes per image line */
//  dc1394video_mode_t       video_mode;            /* the video mode used for capturing this frame */
//  uint64_t                 total_bytes;           /* the total size of the frame buffer in bytes. May include packet-
//                                                   * multiple padding and intentional padding (vendor specific) */
//  uint32_t                 image_bytes;           /* the number of bytes used for the image (image data only, no padding) */
//  uint32_t                 padding_bytes;         /* the number of extra bytes, i.e. total_bytes-image_bytes.  */
//  uint32_t                 bytes_per_packet;      /* the number of bytes per packet. (IIDC data) */
//  uint32_t                 packets_per_frame;     /* the number of packets per frame. (IIDC data) */
//  uint64_t                 timestamp;             /* the unix time [microseconds] at which the frame was captured in
//					 	   * the video1394 ringbuffer */
//  uint32_t                 frames_behind;         /* the number of frames in the ring buffer that are yet to be accessed by the user */
//  dc1394camera_t           *camera;               /* the parent camera of this frame */
//  uint32_t                 id;                    /* the frame position in the ring buffer */
//  uint64_t                 allocated_image_bytes; /* amount of memory allocated in for the *image field. -1 for output
//						                                       * of libdc1394? (this would avoid confusion between 'no allocated
//						                                       * memory' and 'don't touch this buffer' -> signed int?? */ 
//} dc1394video_frame_t;

/**
 * This function initializes the ladybug3 camera
 */
LB3camera_t * 
lb3_init (void);

/**
 * typedef enum{
 * DC1394_OPERATION_MODE_LEGACY = 480,
 * DC1394_OPERATION_MODE_1394B
 * }dc1394operation_mode_t;
 *
 *As the IEEE1394 speeds were increased with IEEE-1394b specifications, a new type
 *of control is necessary when the camera is operating in iso speeds over 800Mbps. 
 *If you wish to use a 1394b camera you may need to switch the operation mode to 1394b.
 *Legacy mode refers to speeds less than 400Mbps.
 */
dc1394error_t 
lb3_set_operation_mode (LB3camera_t *cam, dc1394operation_mode_t opMode);


/**
 * This function sets the video mode for the camera full res or half res
 */
dc1394error_t
lb3_set_video_mode (LB3camera_t *cam, LB3video_mode_t vidMode);

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
lb3_set_iso_speed (LB3camera_t *cam, dc1394speed_t speed);

/**
 *This function sets the framerate of the camera. 
 *If set frame rate is more than the transfer rate then the framerate is governed by the 
 *transfer rate.
 */
dc1394error_t 
lb3_set_frame_rate (LB3camera_t *cam, float framerate);

/**
 * This function sets the auto exposure mask for each camera
 * 1 to use the camera 0 to not. 
 * to ignore the top sensor pass in mask = {1, 1, 1, 1, 1, 0}
 */
dc1394error_t
lb3_set_ae_mask (LB3camera_t *cam, int  mask[6]);


/**
 * This function sets the max shutter speed (to prevent blur indoor)
 */
dc1394error_t 
lb3_set_max_shutter (LB3camera_t *cam, float max_shutter);

/**
 *This function sets the jpeg quality of the camera. 
 *
 */
dc1394error_t
lb3_set_jpeg_quality (LB3camera_t *cam, uint8_t quality);

/**
 * This function starts the video transmission, i.e. data starts transferring and image buffer
 * starts getting filled
 */
dc1394error_t
lb3_start_video_transmission (LB3camera_t *cam);

/**
 * This function stops the video transmission
 */
dc1394error_t
lb3_stop_video_transmission (LB3camera_t *cam);

/**
 * This function frees the parameter
 */
void
lb3_camera_free(LB3camera_t *lb3cam);

/**
 * These set of functions modify the camera settings
 */
dc1394error_t 
lb3_set_brightness_auto (LB3camera_t *cam);

dc1394error_t
lb3_set_exposure_manual (LB3camera_t *cam, int value);

dc1394error_t
lb3_set_exposure_auto (LB3camera_t *cam);

dc1394error_t
lb3_set_gama_manual (LB3camera_t *cam, int value);

dc1394error_t
lb3_set_gama_auto (LB3camera_t *cam);

dc1394error_t
lb3_set_gain_manual (LB3camera_t *cam, int value);

dc1394error_t
lb3_set_gain_auto (LB3camera_t *cam);

dc1394error_t
lb3_set_shutter_manual (LB3camera_t *cam, float value);

dc1394error_t
lb3_set_shutter_auto (LB3camera_t *cam);

dc1394error_t
lb3_set_whitebalance_auto (LB3camera_t *cam);

dc1394error_t
lb3_set_whitebalance_manual (LB3camera_t *cam, int valueBlue, int valueRed);

/**
 * This function enables embedding of timestamp into first few bytes 
 * of the image data
 */
dc1394error_t
lb3_embed_timestamp_into_frame (LB3camera_t *cam);

/**
 * This function returns the synchronized embedded timestamp in microseconds
 */
dc1394error_t
lb3_get_sync_embedded_timestamp (LB3camera_t *cam, dc1394video_frame_t* frame, uint64_t *timestamp);

/**
 * This function writes the ladybug stream header structure into the file
 */
void
lb3_write_stream_header_info (FILE *fp, LB3StreamHeaderInfo_t header);

void
lb3_write_calib_data (FILE *fp_pgr, const char *calib_file_name);

void
lb3_write_jpeg_header (FILE *fp, LB3JpegHeaderInfo_t *jpegHeader);

/**
 * This function returns "1" if keyboard has been hit
 */
int
lb3_kbhit (void);

/**
 * This function extracts the 24 jpeg compressed images (4 image for BGGR color
 * channel per camera) from the frame buffer.
 */
int
lb3_get_jpeg_block_sizes (unsigned char *frame, uint32_t **size);

int
lb3_get_jpeg_bayer_pattern (unsigned char* frame, unsigned char ***bayer, unsigned int **bayersize);

#ifdef __cplusplus
}
#endif

#endif //__LADYBUG3_H__
