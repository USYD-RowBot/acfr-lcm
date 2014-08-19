/*
 *  AVT Vimba based camera driver
 *
 *  Christian Lees
 *  ACFR
 *  27/2/14
 */

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <vimba/VimbaC.h>
#include <pthread.h>
#include <semaphore.h>

#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>
//#include <opencv2/highgui/highgui_c.h>

#include <bot_param/param_client.h>
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-vision/botimage.h"
#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/senlcm_prosilica_t.h"
#include "perls-lcmtypes/acfrlcm_auv_camera_control_t.h"
#include "perls-lcmtypes/acfrlcm_auv_vis_rawlog_t.h"


#define BUFFERS 4

// structure used in the linked list of queued frames to be written to disk
struct _qframe_t
{
    VmbFrame_t *frame;
    int64_t utime;
    struct _qframe_t *next;
};
typedef struct _qframe_t qframe_t;

// time code borrowed from the Perls prosilica driver    
typedef struct {
    VmbInt64_t  dev_ticks_per_second; // how fast does device clock count? (nominal)
    double  max_rate_error;       // how fast do we need to count to ensure we're counting faster than device?

    int64_t sync_host_time;       // when we last synced, what time was it for the host?
    int64_t sync_dev_ticks;       // when we last synced, what device tick was it for the sensor?

    int64_t last_dev_ticks;       // what device time was it when we were last called?

    uint8_t is_valid;             // have we ever synced?
} timestamp_sync_private_state_t;
    
// state structure passed to all functions
typedef struct 
{
    lcm_t *lcm;  
    char *uid;  
    char *channel;
    VmbHandle_t camera;
    timestamp_sync_private_state_t tss;
    VmbInt64_t frame_size;
    char *path;
    int write_files;
    qframe_t *frames_head, *frames_tail;
    pthread_mutex_t frames_mutex;
    sem_t write_sem;
    int publish;
    int image_scale;    
    int64_t previous_frame_utime;
    int queue_length;
    int image_count;
} state_t;


// exit handler
int program_exit;
void signal_handler(int sig_num) 
{
    // do a safe exit
    if(sig_num == SIGINT)
        program_exit = 1;
}

// List the camera features, may be removed in the future and put in a seperate program
int list_camera_features(state_t *state) 
{
    VmbFeatureInfo_t *features;
    unsigned int feature_count = 0;
    VmbError_t err;
    
    err = VmbFeaturesList(state->camera, NULL, 0, &feature_count, sizeof(*features));

    if((err == VmbErrorSuccess) && (feature_count > 0))
    {
        features = (VmbFeatureInfo_t *)malloc(sizeof(VmbFeatureInfo_t) * feature_count);
        err = VmbFeaturesList(state->camera, features, feature_count, &feature_count, sizeof(*features));

        for(int i=0; i<feature_count; i++)
        {
            printf("%s, type: ", features[i].name);
            if(features[i].featureDataType == VmbFeatureDataEnum)
            {
                printf("enum\n\tValid values:\n");
                unsigned int enum_count;
                const char **enums;
                // get the number of enum values
                err = VmbFeatureEnumRangeQuery(state->camera, features[i].name, NULL, 0, &enum_count);
                if((err == VmbErrorSuccess) && (enum_count > 0))
                {
                    enums = malloc(sizeof(char *) * enum_count);
                    VmbFeatureEnumRangeQuery(state->camera, features[i].name, enums, enum_count, &enum_count);
                    for(int j=0; j<enum_count; j++)
                        printf("\t%s\n", enums[j]);
                    free(enums);
                }
            }
            if(features[i].featureDataType == VmbFeatureDataInt)
            {
                printf("int\n\tValid values:\n");
            }
                    if(features[i].featureDataType == VmbFeatureDataFloat)
            {
                printf("float\n\tValid values:\n");
            }
            if(features[i].featureDataType == VmbFeatureDataBool)
            {
                printf("bool\n\tValid values:\n");
            }
            if(features[i].featureDataType == VmbFeatureDataString)
            {
                printf("string\n\tValid values:\n");
            }
            if(features[i].featureDataType == VmbFeatureDataCommand)
            {
                printf("command\n");
            }

            if(features[i].featureDataType == VmbFeatureDataRaw)
            {
                printf("raw\n\tValid values:\n");
            }
            if(features[i].featureDataType == VmbFeatureDataNone)
            {
                printf("none\n\tValid values:\n");
            }


        }
        free(features);
    }
    return 1;
}


// Set the camera attributes based on the LCM config file
int set_camera_attributes(state_t *state, BotParam *params, char *root_key)
{
    // get the list of valid attributes from the camera
    VmbFeatureInfo_t *features;
    unsigned int feature_count = 0;
    VmbError_t err;
    char key[64];
    int ret = 0;
    
    // Things that need to be set
    VmbFeatureBoolSet(state->camera, "ChunkModeActive", 1); // so we get the exposure data
    
    // Things we need to get
    VmbFeatureIntGet(state->camera, "GevTimestampTickFrequency", &state->tss.dev_ticks_per_second);
    printf("Camera timer tick %lld\n", state->tss.dev_ticks_per_second);
    
    err = VmbFeaturesList(state->camera, NULL, 0, &feature_count, sizeof(*features));
    if((err == VmbErrorSuccess) && (feature_count > 0))
    {
        features = (VmbFeatureInfo_t *)malloc(sizeof(VmbFeatureInfo_t) * feature_count);
        err = VmbFeaturesList(state->camera, features, feature_count, &feature_count, sizeof(*features));
        
        for(int i=0; i<feature_count; i++)
        {
        
            // Look for each keyword in the config file and check to see if the value is valid
            sprintf(key, "%s.features.%s", root_key, features[i].name);
            if(bot_param_has_key(params, key))
            {
                // if the feature is an enum get the valid values for this key from the camera
                
        
                if(features[i].featureDataType == VmbFeatureDataEnum)
                {
                
                    unsigned int enum_count;
                    const char **enums;
                    // get the number of enum values
                    err = VmbFeatureEnumRangeQuery(state->camera, features[i].name, NULL, 0, &enum_count);
                    if((err == VmbErrorSuccess) && (enum_count > 0))
                    {
                        enums = malloc(sizeof(char *) * enum_count);
                        VmbFeatureEnumRangeQuery(state->camera, features[i].name, enums, enum_count, &enum_count);
                    
                        char *key_value;
                        int key_found;
                        if(bot_param_get_str(params, key, &key_value) == 0)
                        {
                            // check to see that is a valid key value
                            key_found = 0;
                            for(int j=0; j<enum_count; j++)
                            {
                                if(strncmp(enums[j], key_value, strlen(enums[j])) == 0)
                                    key_found = 1;
                            }
                            
                            if(key_found)
                            {
                                err = VmbFeatureEnumSet(state->camera, features[i].name, key_value);
                                if(err != VmbErrorSuccess)
                                {
                                    printf("Error setting %s to %s, error num %d\n", features[i].name, key_value, err);
                                    ret = -1;
                                }
                                else
                                    printf("Setting %s = %s\n", features[i].name, key_value);
                            }
                            else
                            {
                                printf("Invalid key for feature %s, %s\nValid keys are:\n", features[i].name, key_value);
                                for(int j=0; j<enum_count; j++)
                                    printf("\t%s\n", enums[j]);
                                ret = -1;
                            }
                        }
                        free(key_value);    
                    }    
                    
                    free(enums);
                }
                
                if(features[i].featureDataType == VmbFeatureDataInt)
                {
                    int key_value;
                    if(bot_param_get_int(params, key, &key_value) == 0)
                    {
                        VmbInt64_t value = key_value;
                        err = VmbFeatureIntSet(state->camera, features[i].name, value);
                        if(err != VmbErrorSuccess)
                        {
                            printf("Error setting %s to %d\n", features[i].name, key_value);
                            ret = -1;
                        }
                        else
                            printf("Setting %s = %d\n", features[i].name, key_value);
                    }
                }
                
                if(features[i].featureDataType == VmbFeatureDataFloat)
                {
                    double key_value;
                    if(bot_param_get_double(params, key, &key_value) == 0)
                    {
                        err = VmbFeatureFloatSet(state->camera, features[i].name, key_value);
                        if(err != VmbErrorSuccess)
                        {
                            printf("Error setting %s to %f\n", features[i].name, key_value);
                            ret = -1;
                        }
                        else
                            printf("Setting %s = %f\n", features[i].name, key_value);
                    }
                }
                
                if(features[i].featureDataType == VmbFeatureDataString)
                {
                    char *key_value;
                    if(bot_param_get_str(params, key, &key_value) == 0)
                    {
                        err = VmbFeatureStringSet(state->camera, features[i].name, key_value);
                        if(err != VmbErrorSuccess)
                        {
                            printf("Error setting %s to %s\n", features[i].name, key_value);
                            ret = -1;
                        }
                        else
                            printf("Setting %s = %s\n", features[i].name, key_value);
                    
                        free(key_value);
                    }
                }
                
                if(features[i].featureDataType == VmbFeatureDataBool)
                {
                    int key_value;
                    if(bot_param_get_boolean(params, key, &key_value) == 0)
                    {
                        err = VmbFeatureBoolSet(state->camera, features[i].name, (VmbBool_t)key_value);
                        if(err != VmbErrorSuccess)
                        {
                            printf("Error setting %s to %d\n", features[i].name, key_value);
                            ret = -1;
                        }
                        else
                            printf("Setting %s = %d\n", features[i].name, key_value);
                    }
                }
            }
                
        }       
        free(features);        
                
    }
    return ret;
}

int vimba_to_bot_pixel_format(VmbPixelFormat_t pf)
{
    int bot_pf;
    switch (pf) 
    {
        // mono
        case VmbPixelFormatMono8:   
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
            break;
        case VmbPixelFormatMono16:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16;
            break;

        // bayer 8 bit
        case VmbPixelFormatBayerRG8: 
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB;
            break;
        case VmbPixelFormatBayerGB8: 
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG;
            break;
        case VmbPixelFormatBayerGR8: 
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG;
            break;
        case VmbPixelFormatBayerBG8: 
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR;
            break;

        // bayer 12 bit (unpacked) or 16 bit
        case VmbPixelFormatBayerRG12:
        case VmbPixelFormatBayerRG16:  
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_RGGB;
            break;
        case VmbPixelFormatBayerGB12:
        case VmbPixelFormatBayerGB16:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GBRG;
            break;
        case VmbPixelFormatBayerGR12:
        case VmbPixelFormatBayerGR16:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GRBG;
            break;
        case VmbPixelFormatBayerBG12:
        case VmbPixelFormatBayerBG16:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_BGGR;
            break;

            // color
        case VmbPixelFormatRgb8:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
            break;
        case VmbPixelFormatRgb16:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16;
            break;
        case VmbPixelFormatYuv411:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU1;
            break;
        case VmbPixelFormatYuv422:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_UYVY;
            break;
        case VmbPixelFormatYuv444:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU2;
            break;
        case VmbPixelFormatBgr8:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR;
            break;
        case VmbPixelFormatRgba8:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGBA;
            break;
        case VmbPixelFormatBgra8:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGRA;
            break;
        default:
            fprintf(stderr, "unrecognized pixelformat\n");
            return -1;        
    }
    return bot_pf;
}
            

// Write the image to disk
int write_tiff_image(state_t *state, VmbFrame_t *frame, int64_t utime)
{
    // Generate a filename in the some what broken ACFR naming format
    char acfr_format[24] = "PR_%Y%m%d_%H%M%S_";
    
    // Convert the utime to a timeval then to a tm structure
    struct timeval tv;
    timestamp_to_timeval (utime, &tv);
    struct tm tm;
    localtime_r (&tv.tv_sec, &tm);

    char filename[64];
    memset(filename, 0, sizeof(filename));
    strftime(filename, sizeof(filename), acfr_format, &tm);
    
    // append the milliseconds
    char suffix[16];
    memset(suffix, 0, sizeof(suffix));
    sprintf(suffix, "%03ld_%s.tif", tv.tv_usec/1000, (state->channel + strlen(state->channel) - 4));

    // stick them together    
    strcat(filename, suffix);
    
    // add the path to the filename
    char fullname[128];
    strcpy(fullname, state->path);
    strcat(fullname, filename);
    
    // open TIFF file
    TIFF *image = TIFFOpen(fullname, "w");
    if(!image) 
    {
        fprintf(stderr, "TIFFOpen() failed\n");
        return 0;
    }
    
    
    // Check to make sure the image is in a format that works for TIFF
    if((frame->pixelFormat == VmbPixelFormatYuv411) || (frame->pixelFormat == VmbPixelFormatYuv422) || (frame->pixelFormat == VmbPixelFormatYuv444)  ||
        (frame->pixelFormat & VmbPixelOccupy12Bit) || (frame->pixelFormat & VmbPixelOccupy32Bit))
    {
        fprintf(stderr, "TIFF, unsupported pixel format\n");
        return 0;
    }
    
    // Bytes per pixel
    int bpp;
    if(frame->pixelFormat & VmbPixelOccupy16Bit)
    {
        bpp = 2;
        TIFFSetField (image, TIFFTAG_BITSPERSAMPLE, 16);
    }
    else
    {
        bpp = 1;
        TIFFSetField (image, TIFFTAG_BITSPERSAMPLE, 8);
    }
    
    // Colour or B&W, Bytes per row    
    int stride;
    if(frame->pixelFormat & VmbPixelColor)
    {
        stride = 3 * bpp * frame->width; // rgb, yuv
        TIFFSetField (image, TIFFTAG_SAMPLESPERPIXEL, 3);
        TIFFSetField (image, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
    }
    else
    {
        stride = bpp * frame->width;
        TIFFSetField (image, TIFFTAG_SAMPLESPERPIXEL, 1);
        TIFFSetField (image, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    }
    
    // TIFF attributes
    TIFFSetField (image, TIFFTAG_IMAGEWIDTH, frame->width);
    TIFFSetField (image, TIFFTAG_IMAGELENGTH, frame->height);
    TIFFSetField (image, TIFFTAG_ROWSPERSTRIP, 1);
    TIFFSetField (image, TIFFTAG_FILLORDER, FILLORDER_MSB2LSB);
    TIFFSetField (image, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField (image, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
    
    // Put some useful information in the TIFF description field    
    char description[64];
    char *frame_buffer = (char *)frame->buffer;
    char *ancillary = &frame_buffer[frame->imageSize + 8];
    unsigned int exposure = ((ancillary[8] & 0xFF) << 24) * ((ancillary[9] & 0xFF) << 16) + ((ancillary[10] & 0xFF) << 8) + (ancillary[11] & 0xFF); 
    unsigned int gain = ((ancillary[12] & 0xFF) << 24) * ((ancillary[13] & 0xFF) << 16) + ((ancillary[14] & 0xFF) << 8) + (ancillary[15] & 0xFF);

    snprintf(description, sizeof(description), "T:%"PRId64",W:%u,H:%u,E:%u,G:%u", utime, frame->width, frame->height, exposure, gain);
    TIFFSetField (image, TIFFTAG_IMAGEDESCRIPTION, description);
    
    int failed = 0;
    for(int i=0; i<frame->height; i++) 
        if(TIFFWriteScanline (image, &frame_buffer[i* stride], i, 0) == -1)
            failed = 1;
    
    if(!failed)
    {
        // Publish the ACFR raw log message    
        acfrlcm_auv_vis_rawlog_t vis_raw;
        vis_raw.utime = utime;
        vis_raw.exp_time = exposure;
        vis_raw.image_name = filename;
        acfrlcm_auv_vis_rawlog_t_publish(state->lcm, "ACFR_AUV_VIS_RAWLOG", &vis_raw);
    }
        
    TIFFClose(image);
    
    return 1;
}

// time code borrowed from the Perls prosilica driver
static int64_t timestamp_sync_private (timestamp_sync_private_state_t *s, int64_t dev_ticks, int64_t host_utime)
{
    if (!s->is_valid) {
        /* The first sync has no history */
        s->is_valid = 1;

        s->sync_host_time = host_utime;
        s->last_dev_ticks = dev_ticks;

        return host_utime;
    }

    // how many device ticks since the last sync?
    int64_t dev_ticks_since_sync = dev_ticks - s->sync_dev_ticks;

    // overestimate device time by a factor of s->rate
    double rate = 1000000.0 / s->dev_ticks_per_second * s->max_rate_error;

    // estimate of the host's time corresponding to the device's time
    int64_t dev_utime = s->sync_host_time + (dev_ticks_since_sync * rate);

    int64_t time_err = host_utime - dev_utime;

    /* If time_err is very large, resynchronize, emitting a warning. if
     * it is negative, we're just adjusting our timebase (it means
     * we got a nice new low-latency measurement.) */
    if (time_err > 1000000000LL) { /* 1000 seconds */
        fprintf (stderr, "Warning: Time sync has drifted by more than 1000 seconds\n");
        s->sync_host_time = host_utime;
        s->sync_dev_ticks = dev_ticks;
        dev_utime = host_utime;
    }
    if (time_err < 0) {
        s->sync_host_time = host_utime;
        s->sync_dev_ticks = dev_ticks;
        dev_utime = host_utime;
    }

    return dev_utime;
}



// A thread that handles the writing of the images to disk
static void *write_thread(void *context)
{
    state_t *state = (state_t *)context;
    

    while(!program_exit)
    {
        // wait on a semaphone to see if we have a frame to write to disk
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;


        if(sem_timedwait(&state->write_sem, &ts) != -1)
        {
            int count = 0;
            qframe_t *cframe = state->frames_head;
            while(cframe != state->frames_tail)
            {
                cframe = cframe->next;
                count++;
            }
            state->queue_length = count;
            state->image_count++;
             
            // we have managed to decrement the semaphore which means there is data to write
            pthread_mutex_lock(&state->frames_mutex);            
            qframe_t *frame = state->frames_head;
            
            if(frame != NULL && frame->frame != NULL)
            {
                // write the head frame to disk
                qframe_t *next = state->frames_head->next;
                write_tiff_image(state, frame->frame, frame->utime);
                state->frames_head = next;
                
                // free some memory
                if(frame->frame->buffer != NULL)
                    free(frame->frame->buffer);
                if(frame->frame != NULL)
                    free(frame->frame);
                if(frame != NULL)
                    free(frame);    

                if(frame == state->frames_tail)
                {
//                    printf("List is empty\n");
                    state->frames_tail = NULL;
                }
            }    

            pthread_mutex_unlock(&state->frames_mutex);        
        }
                
    }
    return NULL;
}


// If we are going to publish the image over LCM it happens here, this routine can also debayer and scale the image
// only problem is it is CPU intensive and may not be worth it            
int publish_image(state_t *state, VmbFrame_t *frame, unsigned int exposure, unsigned int gain, int64_t frame_utime)
{   
    bot_core_image_t image;
    bot_core_image_metadata_t metadata;
    
    // allocate meta data for exposure time and gain
    image.nmetadata = 2; 
    image.metadata=malloc(sizeof(bot_core_image_metadata_t)* image.nmetadata);
    image.utime = frame_utime;
    image.pixelformat = vimba_to_bot_pixel_format(frame->pixelFormat);

    char *key_exp = "ExposureValue";
    metadata.key =malloc(strlen(key_exp));
    strcpy(metadata.key,key_exp);
    metadata.n = 4;
    metadata.value =  (uint8_t *)&exposure;
    memcpy(&image.metadata[0], &metadata, sizeof(bot_core_image_metadata_t));
    
    char *key_gain = "GainValue";
    metadata.key =realloc(metadata.key, strlen(key_gain));
    strcpy(metadata.key,key_gain);
    metadata.n = 4;
    metadata.value =  (uint8_t *)&gain;
    memcpy(&image.metadata[1], &metadata, sizeof(bot_core_image_metadata_t));
    
    int bpp;    // bytes per pixel
    if(frame->pixelFormat & VmbPixelOccupy16Bit)
        bpp = 2;
    else
        bpp = 1;
    
    // fast image scale method, bayer images need to be scaled to 1/16, mono can be 1/4 or 1/16
    if(state->image_scale != 1)
    {
        // allocate some memory to put the image in
        int scale;
        int step;
        if((frame->pixelFormat != VmbPixelFormatMono8) || (frame->pixelFormat != VmbPixelFormatMono12) || (frame->pixelFormat != VmbPixelFormatMono14) || (frame->pixelFormat != VmbPixelFormatMono16))
        {
            scale = state->image_scale;
            if(scale == 4)
                step = 2;
            else
                step = 4;
        }
        else
        {
            scale = 16;
            step = 4;
        }
        
        unsigned char *img_resized = malloc(frame->width * frame->height * bpp / scale);
        unsigned short *img_resized_16 = (unsigned short *)img_resized;
        
        // do the fast loop rescale
        int count = 0;
        unsigned char *frame_buffer = (unsigned char *)frame->buffer;
        for(int y=0; y<frame->height; y+=step)
            for(int x=0; x<frame->width; x+=step)
                if(bpp == 2)
                    img_resized_16[count++] = *(short *)&frame_buffer[x + y * frame->width];
                else
                    img_resized[count++] = frame_buffer[x + y * frame->width];
        
        // done, no publish the image            
        image.data = img_resized;
        image.size = frame->width * frame->height * bpp / scale;
        image.width = frame->width / step;
        image.height = frame->height / step;
        image.row_stride = image.width * bpp;
        
        
        bot_core_image_t_publish(state->lcm, state->channel, &image);   
        free(img_resized);
    }
    else
    {
        image.width = frame->width;
        image.height = frame->height;
        image.row_stride = image.width * bpp;
        image.data = (unsigned char *)frame->buffer;
        image.size = frame->width * frame->height * bpp;
        bot_core_image_t_publish(state->lcm, state->channel, &image);   
    }    
    
    free(metadata.key);
    
    return 1;
}       


// Vimba frame callback, called for every frame that is sent from the camera            
void VMB_CALL frame_done_callback(const VmbHandle_t camera , VmbFrame_t *in_frame)   
{
    state_t *state = in_frame->context[0];
    int64_t utime = timestamp_now();
    VmbError_t err;    
    if(in_frame->receiveStatus != VmbFrameStatusComplete)
    {
        printf("Frame error %d\n", in_frame->receiveStatus);
	    err = VmbCaptureFrameQueue(state->camera, in_frame, frame_done_callback);
	    if(err != VmbErrorSuccess)
            printf("Frame requeue error: %d\n", err);
        return;
    }
    
    // make a copy of the frame
    VmbFrame_t *frame = malloc(sizeof(VmbFrame_t));
    memcpy(frame, in_frame, sizeof(VmbFrame_t));
    frame->buffer = malloc(in_frame->bufferSize);
    memcpy(frame->buffer, in_frame->buffer, in_frame->bufferSize);
    
    err = VmbCaptureFrameQueue(state->camera, in_frame, frame_done_callback);
	if(err != VmbErrorSuccess)
        printf("Frame requeue error: %d\n", err);

    // get the actual frame timestamp based on its clock and our clock
    int64_t frame_utime = timestamp_sync_private(&state->tss, frame->timestamp, utime);
    
    // Get a pointer to the start of the ancillary data, the Vimba API hasn't implemented this yet
    char *frame_buffer = (char *)frame->buffer;
    char *ancillary = &frame_buffer[frame->imageSize + 8];
    
    unsigned int exposure = ((ancillary[8] & 0xFF) << 24) * ((ancillary[9] & 0xFF) << 16) + ((ancillary[10] & 0xFF) << 8) + (ancillary[11] & 0xFF); 
    unsigned int gain = ((ancillary[12] & 0xFF) << 24) * ((ancillary[13] & 0xFF) << 16) + ((ancillary[14] & 0xFF) << 8) + (ancillary[15] & 0xFF);
    
    double fps = 1.0 / ((double)(frame_utime / 1e6) - (double)(state->previous_frame_utime / 1e6));
    state->previous_frame_utime = frame_utime;
    double buffer_length = ((double)(state->queue_length * (state->frame_size + sizeof(qframe_t))) / 1024.0) / 1024.0;
    printf("Exp: %u, Gain: %u, fps: %2.2f, utime: %"PRId64", size %d kB, buffer: %3.2f MB, count: %d\n", 
        exposure, gain, fps, frame_utime, frame->imageSize/1024, buffer_length, state->image_count); 
    
    // work out the image stride
    int stride;
    if(frame->pixelFormat & VmbPixelOccupy16Bit)
    {
        if(frame->pixelFormat & VmbPixelColor)
            stride = 6 * frame->width;
        else
            stride = 2 * frame->width;
    }
    
    // shift the 12 bit data to the top of the 16 bit word, I wish this wasnt required as it takes up CPU cycles
    if((frame->pixelFormat & VmbPixelFormatBayerGR12) || (frame->pixelFormat & VmbPixelFormatBayerRG12) || (frame->pixelFormat & VmbPixelFormatBayerBG12) ||
        (frame->pixelFormat & VmbPixelFormatBayerGB12) || (frame->pixelFormat & VmbPixelFormatMono12) ||
        (frame->pixelFormat & VmbPixelFormatBayerGR10) || (frame->pixelFormat & VmbPixelFormatBayerRG10) || (frame->pixelFormat & VmbPixelFormatBayerBG10) ||
        (frame->pixelFormat & VmbPixelFormatBayerGB10) || (frame->pixelFormat & VmbPixelFormatMono10))
        for(int x=0; x<stride*frame->height; x+=2)
        {
            unsigned short *d = (unsigned short *)&frame_buffer[x];
            *d = *d << 4;
        }
    
    if(state->publish)
        {
	printf("pub\n");
        publish_image(state, frame, exposure, gain, frame_utime);
    }
    // If we are writing the data to disk put the frame in the queue
    if(state->write_files)
    {
        pthread_mutex_lock(&state->frames_mutex);
        qframe_t *qframe = malloc(sizeof(qframe_t));
        qframe->frame = frame;
        qframe->utime = frame_utime;
        qframe->next = NULL;
       
        // stick it in the queue
        if(state->frames_tail == NULL)
            state->frames_tail = qframe;
        else
        {
            state->frames_tail->next = qframe;
            state->frames_tail = state->frames_tail->next;
        }
        
        // if there is only one entry in the list then the head is the tail
        if(state->frames_head == NULL)
            state->frames_head = state->frames_tail;
          
        pthread_mutex_unlock(&state->frames_mutex);    
        // increment the semaphone
        sem_post(&state->write_sem);
    }
    else
    {
        free(frame->buffer);
        free(frame);
    }

    
}
    
void camera_control_callback(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_camera_control_t *cc, void *u)
{
    state_t *state = (state_t *)u;
    switch(cc->command)
    {
        case ACFRLCM_AUV_CAMERA_CONTROL_T_LOG_START:
            state->write_files = 1;
            printf("Starting log to disk\n");
            state->image_count = 0;
            break;
            
        case ACFRLCM_AUV_CAMERA_CONTROL_T_LOG_STOP:
            printf("Stopping log to disk\n");
            state->write_files = 0;
            break;
            
        case ACFRLCM_AUV_CAMERA_CONTROL_T_SET_PATH:
            state->path = realloc(state->path, strlen(cc->path) + 1);
            memset(state->path, 0, strlen(cc->path) + 1);
            memcpy(state->path, cc->path, strlen(cc->path));
            // make sure the last charater is a '/', if not add one
            if(state->path[strlen(state->path) - 1] != '/')
                strcat(state->path, "/");
            printf("Setting path to %s\n", state->path);
            break;
    }
}

        


int main(int argc, char **argv)
{
    program_exit = 0;
    signal(SIGINT, signal_handler);
    signal(SIGPIPE, signal_handler);
    
    BotParam *params;
	char root_key[64];
	char key[64];
	
	state_t state;
	
	// Seems more complicated then it should be but we want to be able to dynamically change its size.
    char path[] = "./";
    state.path = malloc(strlen(path) + 1);
    memset(state.path, 0, strlen(path) + 1);
    memcpy(state.path, path, strlen(path));
    
    state.frames_head = NULL;
    state.frames_tail = NULL;
    state.queue_length = 0;
    state.image_count = 0;
    sem_init(&state.write_sem, 0, 0);
    state.write_files = 0;
    pthread_mutex_init(&state.frames_mutex, 0);   
    state.publish = 1;
    state.tss.is_valid = 0;
    state.tss.max_rate_error = 1.0;
    state.previous_frame_utime = 0;
    
    // read the config file, we base the entry on a command line switch
    char opt;
    int got_key = 0;
    int list_features = 0;
    while((opt = getopt(argc, argv, "hlk:")) != -1) {
        if(opt == 'k') 
        {
            strcpy(root_key, optarg);
            got_key = 1;
        }
        if(opt == 'h') 
        {
            fprintf(stderr, "Usage: vimba -k <config key>\n");
            return 0;
        }
        if(opt == 'l') 
            list_features = 1;
    }

    if(!got_key)
    {
        fprintf(stderr, "a config file key is required\n");
        return 0;
    }
    
    
    state.lcm = lcm_create(NULL);
    params = bot_param_new_from_server (state.lcm, 1); 
    
    sprintf(key, "%s.uid", root_key);
    state.uid = bot_param_get_str_or_fail(params, key);
    
    sprintf(key, "%s.channel", root_key);
    state.channel = bot_param_get_str_or_fail(params, key);

    sprintf(key, "%s.publish", root_key);
    state.publish = bot_param_get_boolean_or_fail(params, key);

    sprintf(key, "%s.scale", root_key);
    state.image_scale = bot_param_get_int_or_fail(params, key);
    if((state.image_scale != 1) && (state.image_scale != 4) && (state.image_scale != 16)) 
    {
        fprintf(stderr, "scale must be 1, 4 or 16\n");
        return 0;
    }


    printf("Publish = %d, Scale = %d\n", state.publish, state.image_scale);

    // start Vimba
    if(VmbStartup() == VmbErrorInternalFault)
    {
        fprintf(stderr, "An error occured starting Vimba\n");
        return 0;
    }
    
    VmbError_t err;
    VmbBool_t gige;
    // We ask Vimba for the presence of a GigE transport layer
    err = VmbFeatureBoolGet(gVimbaHandle, "GeVTLIsPresent", &gige);
    if(err == VmbErrorSuccess)
    {
        if (gige)
        {
            // We query all network interfaces using the global Vimba handle
            err = VmbFeatureCommandRun(gVimbaHandle , "GeVDiscoveryAllOnce");
            // Wait for the discovery packets to return
            usleep(200000);
        }
    }
    
    // open the camera
    err = VmbCameraOpen((const char *)&state.uid[0], VmbAccessModeFull, &state.camera);
    
    if(err == VmbErrorNotFound)
    {
        // get a list of the cameras that we can see
        VmbCameraInfo_t *cameras;
        unsigned int camera_count;
        if(VmbCamerasList(NULL , 0, &camera_count , sizeof(*cameras)) == VmbErrorSuccess)
        {
            cameras = malloc(sizeof(VmbCameraInfo_t) * camera_count);
            VmbCamerasList(cameras , camera_count , &camera_count , sizeof(*cameras));
            fprintf(stderr, "Valid cameras are:\n");
            for(int i=0; i<camera_count; i++)
                fprintf(stderr, "Name: %s, Model: %s, Serial: %s, Interface %s. ID: %s\n", cameras[i].cameraName, cameras[i].modelName, cameras[i].serialString, cameras[i].interfaceIdString, cameras[i].cameraIdString);
        }
        else
            fprintf(stderr, "could not get list of cameras\n"); 
        
        VmbShutdown();
        return 0;
    }
    else
        printf("Opened camera UID: %s\n", state.uid);
        

    // Adjust the packet size to match the network        
    if (VmbFeatureCommandRun(state.camera, "GVSPAdjustPacketSize" ) == VmbErrorSuccess)
    {
        VmbBool_t bIsCommandDone = 0;
        do
        {
            if (VmbFeatureCommandIsDone(state.camera, "GVSPAdjustPacketSize", &bIsCommandDone ) != VmbErrorSuccess)
                break;
        } while (bIsCommandDone == 0);
    }    

    if(list_features)
    {
        list_camera_features(&state);
        VmbShutdown();
        return 1;
    }
        
    // get the camera attributes
    if(set_camera_attributes(&state, params, root_key) == -1)
    {
        VmbShutdown();
        return -1;
    }
    
    // create the camera buffers
    // first find out the frame size
    err = VmbFeatureIntGet(state.camera, "PayloadSize", &state.frame_size);
    
        
    VmbFrame_t frames[BUFFERS];
    for(int i=0; i<BUFFERS; i++)
    {    
        frames[i].buffer = malloc(state.frame_size);
        frames[i].bufferSize = state.frame_size;
        frames[i].context[0] = &state;
        
        VmbFrameAnnounce(state.camera, &frames[i], sizeof(VmbFrame_t));
    }
    
    err = VmbCaptureStart(state.camera);
    
    // Queue frames and register callback
    for(int i=0; i<BUFFERS; i++)
    {    
        err = VmbCaptureFrameQueue(state.camera, &frames[i], frame_done_callback);
	    if(err != VmbErrorSuccess)
            printf("Frame queue error: %d\n", err);
    }  
    // start the write thread and detach it
    pthread_t tid;
    pthread_create(&tid, NULL, write_thread, &state);
    pthread_detach(tid);	

    // subscribe to LCM messages
    acfrlcm_auv_camera_control_t_subscribe(state.lcm, "CAMERA_CONTROL", &camera_control_callback, &state);
    
    // start the capture
    err = VmbFeatureCommandRun(state.camera, "AcquisitionStart");
 
    // wait here
    while (!program_exit) {
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        lcmu_handle_timeout(state.lcm, &tv);
    }    
    
    VmbShutdown();
    for(int i=0; i<BUFFERS; i++)
        free(frames[i].buffer);    
    
    pthread_join(tid, NULL);
    
    
}
