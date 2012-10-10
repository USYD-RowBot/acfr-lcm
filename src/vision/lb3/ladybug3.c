#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/select.h>
#include <dc1394/dc1394.h>

#include "perls-vision/ladybug3_util.h"
#include "ladybug3.h"

/**
 * This function looks for the ladybug3 camera connected
 * to the computer and initializes it. It returns a pointer 
 * to the camera handle on success and returns "NULL" on 
 * failure.
 */
LB3camera_t * 
lb3_init (void)
{
    dc1394camera_t *cam = NULL;
    dc1394_t *dc1394 = dc1394_new ();
    if (!dc1394) {
        printf ("Returning dc1394 is null\n"); 
        return NULL;
    }

    dc1394camera_list_t *camlist;
    if (dc1394_camera_enumerate (dc1394, &camlist) < 0) {
        dc1394_free (dc1394);
        printf ("No camera found\n");
        return NULL;
    }
  
    printf ("camlist->num = %d\n", camlist->num);

    if (camlist->num == 0) {
        dc1394_camera_free_list (camlist);
        printf ("No Camera Found\n");
        return NULL; 
    }

    //loop over all firewire cameras available
    for (int i = 0; i < camlist->num; i++) {
        cam = dc1394_camera_new (dc1394, camlist->ids[i].guid);
        printf ("cam->model = %s cam->vendor = %s\n", cam->model,cam->vendor);
        if (0==strcmp (cam->model, LB3_CAM_MODEL)) { //finds Ladybug3
            break;
        }
        else {
            dc1394_camera_free (cam);
            cam = NULL;
        }  
    }
  
    if (cam == NULL) {
        printf ("Failed to find ladybug camera \n");
        dc1394_free (dc1394);
        return NULL;
    }
    
    dc1394_camera_free_list (camlist);

    // show some information about the camera we're using
    dc1394_camera_print_info (cam, stdout);
    
    //dc1394_free (dc1394);
  
    // initialize this camera
    dc1394_camera_reset (cam);

    dc1394_reset_bus (cam);
    
    //Allocate memory to the lb3 camera handle 
    LB3camera_t *lb3cam = malloc (sizeof (*lb3cam));
    lb3cam->cam = cam;
    lb3cam->dc1394 = dc1394; 
    return lb3cam;
}

/**
 * This function sets the operation mode to 1394A or 1394B
 */
dc1394error_t 
lb3_set_operation_mode (LB3camera_t *lb3cam, dc1394operation_mode_t mode)
{
    dc1394error_t status;
    status = dc1394_video_set_operation_mode (lb3cam->cam, mode);
    DC1394_ERR_RTN (status, "setting operation mode");
    return status;
}

/**
 * This function sets the video mode for the camera: full res or half res
 * The following four formats are supported
 * Should define the following video mode, width, height 
 * VideoMode  Width  Height
 * FORMAT7_0  1616   1232*6 //uncompressed, max fps = 6.5
 * FORMAT7_2  1616   616*6  //uncompressed, max fps = 13
 * FORMAT7_3  1616   616*6  //half height jpeg compressed, max fps = 30
 * FORMAT7_7  1616   1232*6 //jpeg compressed, max fps = 15 
 */
dc1394error_t
lb3_set_video_mode (LB3camera_t *lb3cam, LB3video_mode_t vidMode)
{
    dc1394error_t status;
    dc1394video_mode_t mode;
    uint32_t width, height;
    if (vidMode == LB3_FULL_RES_UNCOMPRESSED) {
        mode = DC1394_VIDEO_MODE_FORMAT7_0; // res = 1600 x 1200
        width = 1616; height = 1232*6;
    }
    else if (vidMode == LB3_HALF_RES_UNCOMPRESSED) {
        mode = DC1394_VIDEO_MODE_FORMAT7_2; // res = 1600 x 600
        width = 1616; height = 616*6;
    }
    else if (vidMode == LB3_FULL_RES_JPEG) {
        mode = DC1394_VIDEO_MODE_FORMAT7_7; // res = 800 x 600 x 4
        //In auto JPEG control mode the compression quality is controlled by changing this height  
        //The height also governs the framerate.
        //Current setting (height = 9084) allows a framerate of 10fps at any (1-100) quality.   
        width = 808; height = 9084; //616*4*6; //MAX IMAGE SIZE = width*height 
    }
    else if (vidMode == LB3_HALF_RES_JPEG) {
        mode = DC1394_VIDEO_MODE_FORMAT7_3; // res = 800 x 300 x 4
        //Current setting (height = 4542) allows a framerate of 20fps at any (1-100) quality.   
        width = 808; height = 4542; //308*4*6;
    }
  
    status = dc1394_video_set_mode (lb3cam->cam, mode);
    DC1394_ERR_RTN (status, "setting video mode\n");
  
    uint32_t tempwidth, tempheight;
    status = dc1394_format7_get_image_size (lb3cam->cam, mode, &tempwidth, &tempheight);
    DC1394_ERR_RTN (status, "setting image size");
    printf ("width = %d, height = %d\n", tempwidth, tempheight);
  
    //set image size  
    status = dc1394_format7_set_image_size (lb3cam->cam, mode, width, height);
    DC1394_ERR_RTN (status, "setting image size");
    status = dc1394_format7_get_image_size (lb3cam->cam, mode, &tempwidth, &tempheight);
    DC1394_ERR_RTN (status, "setting image size");
    printf ("Image size set to width = %d, height = %d\n", tempwidth, tempheight);
  
    uint32_t recbpp;
    status = dc1394_format7_get_recommended_packet_size (lb3cam->cam, mode, &recbpp);
    DC1394_ERR_RTN (status, "getting recommended bytes per packet\n");
    printf ("recommended bytes per packet: %d\n", recbpp);
 
    //Setting packet size to maximum recommended packet size 
    status = dc1394_format7_set_packet_size (lb3cam->cam, mode, recbpp);
    DC1394_ERR_RTN (status, "setting bytes per packet\n");
    return status;
}

/**
 * This function sets the framerate of the camera. 
 * If set frame rate is more than the transfer rate then the framerate is governed by the 
 * transfer rate.
 */
dc1394error_t
lb3_set_frame_rate (LB3camera_t *lb3cam, float framerate)
{
    //Check if framerate is out of range then set the max framerate available
    //for that mode
    dc1394error_t status;
    dc1394video_mode_t mode;
    float maxframerate;
    status = dc1394_video_get_mode (lb3cam->cam, &mode);

    if (mode == DC1394_VIDEO_MODE_FORMAT7_0) {
        maxframerate = 6.5;
        if (framerate > maxframerate) {
            framerate = maxframerate;
            printf ("Max frame rate supported for LB3_FULL_RES_UNCOMPRESSED mode is %f\n", framerate);
        }
    }
    else if (mode == DC1394_VIDEO_MODE_FORMAT7_2) {
        maxframerate = 13.0;
        if (framerate > maxframerate) {
            framerate = maxframerate;
            printf ("Max frame rate supported for LB3_HALF_RES_UNCOMPRESSED mode is %f\n", framerate);
        }
    }
    else if (mode == DC1394_VIDEO_MODE_FORMAT7_7) {
        maxframerate = 15.0;
        if (framerate > maxframerate) {
            framerate = maxframerate;
            printf ("Max frame rate supported for LB3_FULL_RES_JPEG mode is %f\n", framerate);
        }
    }
    else if (mode == DC1394_VIDEO_MODE_FORMAT7_3) {
        maxframerate = 30.0;
        if (framerate > maxframerate) {
            framerate = maxframerate;
            printf ("Max frame rate supported for LB3_HALF_RES_JPEG mode is %f\n", framerate);
        }
    }
  
    //Control frame rate with Absolute value control register
    uint32_t value = 0;
    value = 0xC2000000; 
    status = dc1394_set_control_register (lb3cam->cam, LB3_REGISTER_FRAME_RATE_CONTROL, value);
    DC1394_ERR_RTN (status, "Cannot set the frame rate register value");

    //copy the bits of IEEE*4 32-bit floating point number to unsigned integer.
    uint32_t ui;
    memcpy (&ui, &framerate, sizeof (ui));
    //printf ("framerate in hex: sizeof framerate = %d, ui = %x framerate = %f\n", sizeof(framerate), ui, framerate);
    status = dc1394_set_control_register (lb3cam->cam, LB3_REGISTER_ABS_FRAME_RATE_CONTROL, ui);
    
    //uint32_t value_set;
    //status = dc1394_get_control_register (lb3cam->cam, LB3_REGISTER_ABS_FRAME_RATE_CONTROL, &value_set);
    //printf("framerate value set to = %x\n", value_set);
    //DC1394_ERR_RTN (status, "Cannot set the frame rate register value");
    return status;
}

dc1394error_t 
lb3_set_max_shutter (LB3camera_t *lb3cam, float max_shutter)
{
    dc1394error_t status;

    // read and print the min, max, and value
    uint32_t min, max, value;
    float min_f, max_f, value_f;
    status = dc1394_get_control_register (lb3cam->cam, LB3_REGISTER_ABS_SHUTTER_MIN_VAL, &min);
    status = dc1394_get_control_register (lb3cam->cam, LB3_REGISTER_ABS_SHUTTER_MAX_VAL, &max);
    status = dc1394_get_control_register (lb3cam->cam, LB3_REGISTER_ABS_SHUTTER_VAL, &value);
    memcpy (&min_f, &min, sizeof (min));
    memcpy (&max_f, &max, sizeof (max));
    memcpy (&value_f, &value, sizeof (value));
    //printf("\n Shutter min=%f, max=%f, value=%f \n", min_f, max_f, value_f);
    
    // read the auto shutter range
    uint32_t shutter_reg;
    status = dc1394_get_control_register (lb3cam->cam, LB3_REGISTER_AUTO_SHUTTER_RANGE, &shutter_reg);
    uint32_t min_rel;
    /* uint32_t max_rel; */
    min_rel = ((shutter_reg >> 12)  & 0x00000FFF);
    /* max_rel = (shutter_reg & 0x00000FFF); */
    //printf ("Shutter range min = %d, max = %d \n", min_rel, max_rel);
        
    // bits 8-19 min value
    // bits 20-31 max value
    uint32_t target = round ((max_shutter-min_f)/(max_f-min_f)*(float)(0xFFF-min_rel)+min_rel);
    //printf ("target=%d \n", target);
    uint32_t ui = (shutter_reg & 0xFFFFF000) + (target & 0x00000FFF);
    status = dc1394_set_control_register (lb3cam->cam, LB3_REGISTER_AUTO_SHUTTER_RANGE, ui);
    
    return status;
}

dc1394error_t
lb3_set_ae_mask (LB3camera_t *lb3cam, int  mask[6])
{
    dc1394error_t status;

    uint32_t reg;
    status = dc1394_get_control_register (lb3cam->cam, LB3_REGISTER_AE_STATS_MASK, &reg);
    uint32_t ui = (reg & 0xFFFFFFC0);
    if (mask[0])
        ui += 0x00000020;
    else if (mask[1])
	ui += 0x00000010;
    else if (mask[2])
        ui += 0x00000008;
    else if (mask[3])
        ui += 0x00000004;
    else if (mask[4])
        ui += 0x00000002;
    else if (mask[5])
        ui += 0x00000001;

    status = dc1394_set_control_register (lb3cam->cam, LB3_REGISTER_AE_STATS_MASK, ui);
    return status;
}

/**
 * This function sets the jpeg quality of the camera. 
 * quality = 1 to 100
 */
dc1394error_t
lb3_set_jpeg_quality (LB3camera_t *lb3cam, uint8_t quality)
{
    dc1394error_t status;

#if 0
    //Set the framerate to AUTO mode. So that we can control the JPEG compression from the 
    //height of image
    uint32_t finalVal = 0x83000000;
    status = dc1394_set_control_register (lb3cam->cam, LB3_REGISTER_JPEG_QUALITY_CONTROL, finalVal);
    DC1394_ERR_RTN (status, "Cannot set the frame rate register value");
    dc1394video_mode_t mode;
    status = dc1394_video_get_mode (lb3cam->cam, &mode);
    uint32_t tempwidth, tempheight;
    status = dc1394_format7_get_image_size (lb3cam->cam, mode, &tempwidth, &tempheight);
    DC1394_ERR_RTN (status, "setting image size");
    printf ("Inside set jpeg quality : width = %d, height = %d\n", tempwidth, tempheight);
  
    //set image size
    int height = tempheight*quality/100;
    printf("new height = %d\n", height);
    status = dc1394_format7_set_image_size (lb3cam->cam, mode, tempwidth, height);
    
    uint32_t recbpp;
    status = dc1394_format7_get_recommended_packet_size (lb3cam->cam, mode, &recbpp);
    DC1394_ERR_RTN (status, "getting recommended bytes per packet\n");
    printf ("recommended bytes per packet: %d\n", recbpp);
  
    status = dc1394_format7_set_packet_size (lb3cam->cam, mode, recbpp);
    DC1394_ERR_RTN (status, "setting bytes per packet\n");
    return status;
#endif

#if 1 
    uint32_t value = 0x82000000;
    uint32_t finalVal = value;
    finalVal = value | quality;
    status = dc1394_set_control_register (lb3cam->cam, LB3_REGISTER_JPEG_QUALITY_CONTROL, finalVal);
    DC1394_ERR_RTN (status, "Cannot set the frame rate register value");
    
    return status; 
 #endif
}
/**
 * This function sets the isochronous speed
 */
dc1394error_t
lb3_set_iso_speed (LB3camera_t *lb3cam, dc1394speed_t speed)
{
    //For speed greater than 400 Mbps you need to set the operation mode to 1394B
    dc1394error_t status;
    status = dc1394_video_set_iso_speed (lb3cam->cam, speed);
    DC1394_ERR_RTN (status, "setting speed");
    return status;
}

/**
 * This function starts the video transmission, i.e. data starts transferring and image buffer
 * starts getting filled
 */
dc1394error_t
lb3_start_video_transmission (LB3camera_t *lb3cam)
{
    dc1394error_t status;
    status = dc1394_capture_setup (lb3cam->cam, 4, DC1394_CAPTURE_FLAGS_DEFAULT);
    DC1394_ERR_RTN (status, "capture setup");
   
    // start data transmission
    status = dc1394_video_set_transmission (lb3cam->cam, DC1394_ON);
    DC1394_ERR_RTN (status, "enabling video transmission");
    return status;
}
 
/**
 * This function stops the video transmission
 */
dc1394error_t
lb3_stop_video_transmission (LB3camera_t *lb3cam)
{
    dc1394error_t status;
    // stop data transmission
    status = dc1394_video_set_transmission (lb3cam->cam, DC1394_OFF);
    DC1394_ERR_RTN (status, "disabling video transmission");

    // release capture resources
    status=dc1394_capture_stop (lb3cam->cam);
    DC1394_ERR_RTN (status, "Could not stop capture");

    //dc1394_camera_free (cam);

    return status;
}

/**
 * This function frees the parameter
 */
void
lb3_camera_free (LB3camera_t *lb3cam)
{
  dc1394_camera_free (lb3cam->cam);
  dc1394_free (lb3cam->dc1394);
}
 
/**
 * These functions allows the user to do camera settings
 * Returns error code "DC1394_SUCCESS" on success and
 * "DC1394_FAILURE" on failure.
 */
dc1394error_t
lb3_set_brightness_manual (LB3camera_t* lb3cam, int value)
{
    dc1394error_t err = dc1394_feature_set_power (lb3cam->cam, DC1394_FEATURE_BRIGHTNESS, DC1394_ON);
    err = dc1394_feature_set_mode (lb3cam->cam, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL);
    err = dc1394_feature_set_absolute_value (lb3cam->cam, DC1394_FEATURE_BRIGHTNESS, value);
    return err;
}

dc1394error_t
lb3_set_brightness_auto (LB3camera_t *lb3cam)
{
    dc1394error_t err = dc1394_feature_set_power (lb3cam->cam, DC1394_FEATURE_BRIGHTNESS, DC1394_OFF);
    return err;
}

dc1394error_t
lb3_set_exposure_manual (LB3camera_t *lb3cam, int value)
{
    dc1394error_t err = dc1394_feature_set_power (lb3cam->cam, DC1394_FEATURE_EXPOSURE, DC1394_ON);
    err = dc1394_feature_set_mode (lb3cam->cam, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_MANUAL);
    err = dc1394_feature_set_absolute_value (lb3cam->cam, DC1394_FEATURE_EXPOSURE, value);
    return err;
}

dc1394error_t
lb3_set_exposure_auto (LB3camera_t *lb3cam)
{
    dc1394error_t err = dc1394_feature_set_power (lb3cam->cam, DC1394_FEATURE_EXPOSURE, DC1394_ON);
    err = dc1394_feature_set_mode (lb3cam->cam, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_AUTO);
    return err;
}

dc1394error_t
lb3_set_gama_manual (LB3camera_t *lb3cam, int value)
{
    dc1394error_t err = dc1394_feature_set_power (lb3cam->cam, DC1394_FEATURE_GAMMA, DC1394_ON);
    err = dc1394_feature_set_mode (lb3cam->cam, DC1394_FEATURE_GAMMA, DC1394_FEATURE_MODE_MANUAL);
    err = dc1394_feature_set_absolute_value (lb3cam->cam, DC1394_FEATURE_GAMMA, value);
    return err;
}

dc1394error_t
lb3_set_gama_auto (LB3camera_t *lb3cam)
{
    dc1394error_t err = dc1394_feature_set_power (lb3cam->cam, DC1394_FEATURE_GAMMA, DC1394_OFF);
    return err;
}

dc1394error_t
lb3_set_gain_manual (LB3camera_t *lb3cam, int value)
{
    dc1394error_t err = dc1394_feature_set_power (lb3cam->cam, DC1394_FEATURE_GAIN, DC1394_ON);
    err = dc1394_feature_set_mode (lb3cam->cam, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);
    err = dc1394_feature_set_absolute_value (lb3cam->cam, DC1394_FEATURE_GAIN, value);
    return err;
}

dc1394error_t
lb3_set_gain_auto (LB3camera_t *lb3cam)
{
    dc1394error_t err = dc1394_feature_set_power (lb3cam->cam, DC1394_FEATURE_GAIN, DC1394_ON);
    err = dc1394_feature_set_mode (lb3cam->cam, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO);
    return err;
}

dc1394error_t
lb3_set_shutter_manual (LB3camera_t *lb3cam, float value)
{
    dc1394error_t err = dc1394_feature_set_power (lb3cam->cam, DC1394_FEATURE_SHUTTER, DC1394_ON);
    err = dc1394_feature_set_mode (lb3cam->cam, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
    err = dc1394_feature_set_absolute_value (lb3cam->cam, DC1394_FEATURE_SHUTTER, value);
    return err;
}

dc1394error_t
lb3_set_shutter_auto (LB3camera_t *lb3cam)
{
    dc1394error_t err = dc1394_feature_set_power (lb3cam->cam, DC1394_FEATURE_SHUTTER, DC1394_ON);
    err = dc1394_feature_set_mode (lb3cam->cam, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO);
    return err;
}

dc1394error_t
lb3_set_whitebalance_manual (LB3camera_t *lb3cam, int valueBlue, int valueRed)
{
    dc1394error_t err = dc1394_feature_set_power (lb3cam->cam, DC1394_FEATURE_WHITE_BALANCE, DC1394_ON);
    err = dc1394_feature_set_mode (lb3cam->cam, DC1394_FEATURE_WHITE_BALANCE, DC1394_FEATURE_MODE_MANUAL);
    err = dc1394_feature_whitebalance_set_value (lb3cam->cam, valueBlue, valueRed);
    return err;
}

dc1394error_t
lb3_set_whitebalance_auto (LB3camera_t *lb3cam)
{
    dc1394error_t err = dc1394_feature_set_power (lb3cam->cam, DC1394_FEATURE_WHITE_BALANCE, DC1394_OFF);
    return err;
}

/**
 * This function embeds the timestamp coming from dc1394 library 
 * to the first few bytes of the image frame.
 */
dc1394error_t
lb3_embed_timestamp_into_frame (LB3camera_t *lb3cam)
{
    uint32_t value=0;
    dc1394error_t status = dc1394_get_control_register (lb3cam->cam, LB3_REGISTER_FRAME_TIMESTAMP, &value);
    DC1394_ERR_RTN (status, "The timestamp function is not available");
    if (!( value & 0x80000000)) {
        printf("Camera does not support the timestamp feature - upgrade firmware\n");
    }
    value = value | 0x1U;
    status = dc1394_set_control_register (lb3cam->cam, LB3_REGISTER_FRAME_TIMESTAMP, value);
    DC1394_ERR_RTN (status, "Cannot set the timestamp register value");
    return status;
}

/**
 * This function returns the synchronized embedded timestamp in microseconds
 */
dc1394error_t
lb3_get_sync_embedded_timestamp (LB3camera_t* lb3cam, dc1394video_frame_t* frame, uint64_t *timestamp) 
{   
    //This code is taken from camunits/plugins/dc1394/input_dc1394.c 
    
    dc1394error_t status;
    uint64_t bus_timestamp;
    // TODO David - FIXME
    uint32_t cycle_usec_now;
    uint64_t systime = 0;
    int sec_mask = 0;
    uint32_t cyctime;

    //get embedded timestamp from first 4 bytes of image data
    bus_timestamp = (frame->image[0] << 24) |
                    (frame->image[1] << 16) | (frame->image[2] << 8) |
                     frame->image[3];
    /* bottom 4 bits of cycle offset will be a frame count */
    bus_timestamp &= 0xfffffff0;

    status = dc1394_read_cycle_timer (lb3cam->cam, &cyctime, &systime); 
    if (status != DC1394_SUCCESS) {
      printf ("cannot read cycle time\n");
      return status;
    }
    sec_mask = 0x7f;
    cycle_usec_now = CYCLE_TIMER_TO_USEC (cyctime, sec_mask);

    int usec_diff = cycle_usec_now - CYCLE_TIMER_TO_USEC (bus_timestamp, sec_mask);
    if (usec_diff < 0)
      usec_diff += CYCLE_TIMER_MAX_USEC (sec_mask);

    *timestamp = systime - usec_diff;

    return status;
}

/**
 * This function writes the Ladybug stream header info into the file
 */
void
lb3_write_stream_header_info (FILE *fp, LB3StreamHeaderInfo_t header)
{
    // Signature	
    fprintf (fp,"PGRLADYBUGSTREAM");
    fwrite (&header.versionNum, 4, 1, fp);
    fwrite (&header.framerate, 4, 1, fp);
    fwrite (&header.baseSerialNum, 4, 1, fp);
    fwrite (&header.headSerialNum, 4, 1, fp);
    uint32_t reserve = 0x0000;
    for (int i = 0 ; i < 25; i++)
        fwrite (&reserve, 4, 1, fp);

    fwrite (&header.paddingBlock, 4, 1, fp);
    fwrite (&header.dataFormat, 4, 1, fp);
    fwrite (&header.resolution, 4, 1, fp);
    fwrite (&header.bayerPattern, 4, 1, fp);
    fwrite (&header.configDataSize, 4, 1, fp);
    fwrite (&header.numImages, 4, 1, fp);
    fwrite (&header.numIndex_M, 4, 1, fp);
    fwrite (&header.interval_K, 4, 1, fp);
    fwrite (&header.streamDataOffset, 4, 1, fp);
    fwrite (&header.gpsDataOffset, 4, 1, fp);
    fwrite (&header.gpsDataSize, 4, 1, fp);
    //Reserved space is 848 bytes 
    //Also need to fill space for Image index [0] to Image index [M-1]
    //Basically fill the space from position 0xA0 to 0xBF0
    for (int i = 0; i < 723; i++)
        fwrite (&reserve, 4, 1, fp);
 
    //|| Image index [0] || 4 unsigned int || Offset of image 0 ||
    //Since numIndex_M is fixed to 0
    fwrite (&header.streamDataOffset, 4, 1, fp);
}

/**
 * This function writes the calib data into the file stream
 */
void
lb3_write_calib_data (FILE *fp, const char *calib_file_name)
{
    uint32_t a;
    FILE *calib_fp = fopen (calib_file_name, "rb");
    while (fread (&a, 4, 1, calib_fp), !feof (calib_fp) && !ferror(calib_fp))
        fwrite (&a, 4, 1, fp); 

 
    //if (feof (calib_fp))
        //printf ("End of file was reached.\n");i

    if (ferror (calib_fp))
        printf ("An error occurred.\n");

    fclose (calib_fp);
}

/**
 * This function writes the Jpeg header in the file
 */
void
lb3_write_jpeg_header (FILE *fp, LB3JpegHeaderInfo_t *jpegHeader)
{
    uint32_t reserve = 0x0000;
    fwrite (&jpegHeader->timestamp, 8, 1, fp);
    //fwrite (&reserve, 4, 1, fp);
    fwrite (&jpegHeader->dataSize, 4, 1, fp);
    fwrite (&reserve, 4, 1, fp);
    fwrite (&jpegHeader->fingerPrint, 4, 1, fp);
    fwrite (&jpegHeader->versionNumber, 4, 1, fp);
    fwrite (&jpegHeader->timestamp_sec, 4, 1, fp);
    fwrite (&jpegHeader->timestamp_musec, 4, 1, fp);
    fwrite (&jpegHeader->seqId, 4, 1, fp);
    fwrite (&jpegHeader->refreshRate, 4, 1, fp);
    for (int i = 0; i < 6; i++)
        fwrite (&jpegHeader->gain[i], 4, 1, fp);
    fwrite (&jpegHeader->whiteBalance, 4, 1, fp);
    fwrite (&jpegHeader->bayerGain, 4, 1, fp);
    fwrite (&jpegHeader->bayerMap, 4, 1, fp);
    fwrite (&jpegHeader->brightness, 4, 1, fp);
    fwrite (&jpegHeader->gamma, 4, 1, fp);
    fwrite (&jpegHeader->headSerialNum, 4, 1, fp);
    for (int i = 0; i < 6; i++)
        fwrite (&jpegHeader->shutter[i], 4, 1, fp);
    //reserved 24 + 632 + 56 = 712 = 178*4 bytes
    for (int i = 0; i < 178; i++)
        fwrite (&reserve, 4, 1, fp);
    fwrite (&jpegHeader->gpsOffset, 4, 1, fp); 
    fwrite (&jpegHeader->gpsSize, 4, 1, fp); 
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 4; j++) {
            fwrite (&jpegHeader->jpegDataOffset[i][j], 4, 1, fp);
            fwrite (&jpegHeader->jpegDataSize[i][j], 4, 1, fp);
        }
    }
}

/**
 * This function returns 1 if keyboard is hit
 */
int
lb3_kbhit (void)
{
    struct timeval tv = {
        .tv_sec=0,
        .tv_usec=0,
    };
    fd_set read_fd;
    FD_ZERO (&read_fd);
    FD_SET (0,&read_fd);
  
    if (select (1, &read_fd, NULL, NULL, &tv) == -1)
        return 0;

    if (FD_ISSET (0,&read_fd))
        return 1;
 
    return 0;
}

/**
 * This function gets the size of the JPEG blocks
 */
int 
lb3_get_jpeg_block_sizes (unsigned char *frame, uint32_t **size)
{
    //Loop over all cameras 
    /* unsigned int jpgadr; */
    unsigned int jpgsize, adr;
    for (int cam = 0; cam < 6; cam++) {
        //4 bayer images (BGGR) per camera
        for (int k = 0; k < 4; k++) {
            adr = 0x340+(5-cam)*32+(3-k)*8;
            /* jpgadr = (((unsigned int)*(frame+adr))<<24)+ */
            /*          (((unsigned int)*(frame+adr+1))<<16)+ */
            /*          (((unsigned int)*(frame+adr+2))<<8)+ */
            /*          (((unsigned int)*(frame+adr+3))); */
            adr += 4;
            jpgsize = (((unsigned int)*(frame+adr))<<24)+
                      (((unsigned int)*(frame+adr+1))<<16)+
                      (((unsigned int)*(frame+adr+2))<<8)+
                      (((unsigned int)*(frame+adr+3)));

            if (jpgsize!=0) //copy the size
                size[cam][k] = jpgsize;
        }
    }
    return 0;
}

/**
 * This function extracts the 24 jpeg compressed images (4 image for BGGR color
 * channel per camera) from the frame buffer.
 */
int 
lb3_get_jpeg_bayer_pattern (unsigned char* frame, unsigned char*** bayer, unsigned int **size)
{
    //Loop over all cameras 
    unsigned int jpgadr, jpgsize, adr;
    for (int cam = 0; cam < 6; cam++) {
        //4 bayer images (BGGR) per camera
        for (int k = 0; k < 4; k++) {
            adr = 0x340+(5-cam)*32+(3-k)*8;
            jpgadr = (((unsigned int)*(frame+adr))<<24)+
                     (((unsigned int)*(frame+adr+1))<<16)+
                     (((unsigned int)*(frame+adr+2))<<8)+
                     (((unsigned int)*(frame+adr+3)));
            adr += 4;
            jpgsize = (((unsigned int)*(frame+adr))<<24)+
                      (((unsigned int)*(frame+adr+1))<<16)+
                      (((unsigned int)*(frame+adr+2))<<8)+
                      (((unsigned int)*(frame+adr+3)));
  
            if (jpgsize!=0) {
                //memcpy to bayer
                bayer[cam][k] = (unsigned char*) malloc (jpgsize); 
                memcpy (bayer[cam][k], (unsigned char*) (jpgadr+frame), jpgsize);
                size[cam][k] = jpgsize;
            }
        }
    }
    return 0;
}
