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
#include <pthread.h>
#include <semaphore.h>
#include <tiffio.h>
#include <time.h>

#include <flycapture/C/FlyCapture2_C.h>

#include <bot_param/param_client.h>
//#include "perls-common/lcm_util.h"
#include "acfr-common/timestamp.h"
#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/senlcm_prosilica_t.h"
#include "perls-lcmtypes/acfrlcm_auv_camera_control_t.h"
#include "perls-lcmtypes/acfrlcm_auv_vis_rawlog_t.h"

#define BUFFERS 4

// structure used in the linked list of queued frames to be written to disk
struct _qframe_t
{
    fc2Image *frame;
    int64_t utime;
    struct _qframe_t *next;
};
typedef struct _qframe_t qframe_t;

// time code borrowed from the Perls prosilica driver
typedef struct
{
    //VmbInt64_t  dev_ticks_per_second; // how fast does device clock count? (nominal)
    double  max_rate_error;       // how fast do we need to count to ensure we're counting faster than device?

    int64_t sync_host_time;       // when we last synced, what time was it for the host?
    int64_t sync_dev_ticks;       // when we last synced, what device tick was it for the sensor?

    int64_t last_dev_ticks;       // what device time was it when we were last called?

    uint8_t is_valid;             // have we ever synced?
} timestamp_sync_private_state_t;

typedef enum
{
    CAMERA_IDLE,
    CAMERA_OPEN,
    CAMERA_CLOSE,
    CAMERA_NONE
} camera_state_t;

// state structure passed to all functions
typedef struct
{
    lcm_t *lcm;
    BotParam *params;
    char root_key[64];

    char *uid;
    char *mac;
    char *channel;
    char *camera_name;

    fc2Context fc_context;
    //VmbHandle_t camera;
    timestamp_sync_private_state_t tss;
    //VmbInt64_t frame_size;
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
    //VmbFrame_t frames[BUFFERS];

    camera_state_t camera_state;
    int camera_state_change;
    pthread_mutex_t camera_lock;
} state_t;


// exit handler
int program_exit;
void signal_handler(int sig_num)
{
    // do a safe exit
    if(sig_num == SIGINT)
        program_exit = 1;
}

// Set the camera attributes based on the LCM config file
int set_camera_attributes(state_t *state)
{
    /*
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
            sprintf(key, "%s.features.%s", state->root_key, features[i].name);

            if(bot_param_has_key(state->params, key))
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
                        if(bot_param_get_str(state->params, key, &key_value) == 0)
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
                    if(bot_param_get_int(state->params, key, &key_value) == 0)
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
                    if(bot_param_get_double(state->params, key, &key_value) == 0)
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
                    if(bot_param_get_str(state->params, key, &key_value) == 0)
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
                    if(bot_param_get_boolean(state->params, key, &key_value) == 0)
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
    else
        fprintf(stderr, "There was a problem getting the camera feature list, feature count: %d\n", feature_count);

    */

    return 0; // ret;
}

int fc2_to_bot_pixel_format(fc2PixelFormat pf, fc2BayerTileFormat bayer)
{
    int bot_pf;
    switch (pf)
    {
    // mono
    case FC2_PIXEL_FORMAT_MONO8:
        bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
        break;
    case FC2_PIXEL_FORMAT_MONO16:
        bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16;
        break;

    // bayer 8 bit
    case FC2_PIXEL_FORMAT_RAW8:
        switch (bayer)
        {
        case FC2_BT_RGGB:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB;
        case FC2_BT_GBRG:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG;
        case FC2_BT_GRBG:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG;
        case FC2_BT_BGGR:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR;
        }
        break;

    // bayer 12 bit (unpacked) or 16 bit
    case FC2_PIXEL_FORMAT_RAW12:
    case FC2_PIXEL_FORMAT_RAW16:
        case FC2_BT_RGGB:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER16_RGGB;
        case FC2_BT_GBRG:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER16_GBRG;
        case FC2_BT_GRBG:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER16_GRBG;
        case FC2_BT_BGGR:
            bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER16_BGGR;
        break;

    // color
    case FC2_PIXEL_FORMAT_RGB:
        bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
        break;
    case FC2_PIXEL_FORMAT_RGB16:
        bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16;
        break;
    case FC2_PIXEL_FORMAT_411YUV8:
        bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU1;
        break;
    case FC2_PIXEL_FORMAT_422YUV8:
        bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_UYVY;
        break;
    case FC2_PIXEL_FORMAT_444YUV8:
        bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU2;
        break;
    case FC2_PIXEL_FORMAT_BGR:
        bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR;
        break;
    case FC2_PIXEL_FORMAT_RGBU:
        bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGBA;
        break;
    case FC2_PIXEL_FORMAT_BGRU:
        bot_pf = BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGRA;
        break;
    default:
        fprintf(stderr, "unrecognized pixelformat\n");
        return -1;
    }
    return bot_pf;
}


// Write the image to disk
int write_tiff_image(state_t *state, fc2Image *frame, int64_t utime)
{
    // Generate a filename in the some what broken ACFR naming format
    char acfr_format[24] = "PR_%Y%m%d_%H%M%S_";

    // Convert the utime to a timeval then to a tm structure
    struct timeval tv;
    timestamp_to_timeval (utime, &tv);
    struct tm *tm = localtime (&tv.tv_sec);

    char filename[64];
    memset(filename, 0, sizeof(filename));
    strftime(filename, sizeof(filename), acfr_format, tm);

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

    /*// open TIFF file
    TIFF *image = TIFFOpen(fullname, "w");
    if(!image)
    {
        fprintf(stderr, "TIFFOpen() failed\n");
        return 0;
    }


    // Check to make sure the image is in a format that works for TIFF
    if((frame->pixelFormat == FC2_PIXEL_FORMAT_Yuv411) || (frame->pixelFormat == FC2_PIXEL_FORMAT_Yuv422) || (frame->pixelFormat == FC2_PIXEL_FORMAT_Yuv444)  ||
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

    TIFFClose(image);*/

    return 1;
}

// time code borrowed from the Perls prosilica driver
static int64_t timestamp_sync_private (timestamp_sync_private_state_t *s, int64_t dev_ticks, int64_t host_utime)
{
    if (!s->is_valid)
    {
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
    if (time_err > 1000000000LL)   /* 1000 seconds */
    {
        fprintf (stderr, "Warning: Time sync has drifted by more than 1000 seconds\n");
        s->sync_host_time = host_utime;
        s->sync_dev_ticks = dev_ticks;
        dev_utime = host_utime;
    }
    if (time_err < 0)
    {
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
        timespec_get(&ts, TIME_UTC);
        //clock_gettime(CLOCK_REALTIME, &ts);
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
                /*if(frame->frame->buffer != NULL)
                    free(frame->frame->buffer);
                if(frame->frame != NULL)
                    free(frame->frame);
                if(frame != NULL)
                    free(frame);
                    */

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
int publish_image(state_t *state, fc2Image *frame, unsigned int exposure, unsigned int gain, int64_t frame_utime)
{
    bot_core_image_t image;
    bot_core_image_metadata_t metadata;

    // allocate meta data for exposure time and gain
    image.nmetadata = 2;
    image.metadata=malloc(sizeof(bot_core_image_metadata_t)* image.nmetadata);
    image.utime = frame_utime;
    image.pixelformat = fc2_to_bot_pixel_format(frame->format, frame->bayerFormat);

    char *key_exp = "ExposureValue";
    metadata.key =malloc(strlen(key_exp));
    strncpy(metadata.key,key_exp, strlen(key_exp));
    metadata.n = 4;
    metadata.value =  (uint8_t *)&exposure;
    memcpy(&image.metadata[0], &metadata, sizeof(bot_core_image_metadata_t));

    char *key_gain = "GainValue";
    metadata.key =realloc(metadata.key, strlen(key_gain));
    strncpy(metadata.key,key_gain, strlen(key_gain));
    metadata.n = 4;
    metadata.value =  (uint8_t *)&gain;
    memcpy(&image.metadata[1], &metadata, sizeof(bot_core_image_metadata_t));

    int bpp;    // bytes per pixel
    // TODO
    /*if(frame->pixelFormat & VmbPixelOccupy16Bit)
        bpp = 2;
    else
        bpp = 1;*/

    // fast image scale method, bayer images need to be scaled to 1/16, mono can be 1/4 or 1/16
    if(state->image_scale != 1)
    {
        // allocate some memory to put the image in
        int scale;
        int step;
        if((frame->format != FC2_PIXEL_FORMAT_MONO8) || (frame->format != FC2_PIXEL_FORMAT_MONO12) || (frame->format != FC2_PIXEL_FORMAT_MONO16))
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

        unsigned char *img_resized = malloc(frame->cols * frame->rows * bpp / scale);

        // do the fast loop rescale
        int count = 0;
        unsigned char *frame_buffer = (unsigned char *)frame->pData;
        for(int y=0; y<frame->rows; y+=step)
            for(int x=0; x<frame->cols; x+=step)
            {
                memcpy(&img_resized[count], &frame_buffer[(x + y * frame->cols) * bpp], bpp);
                count += bpp;
            }


        // done, now publish the image
        image.data = img_resized;
        image.size = frame->cols * frame->rows * bpp / scale;
        image.width = frame->cols / step;
        image.height = frame->rows / step;
        image.row_stride = image.width * bpp;

        bot_core_image_t_publish(state->lcm, state->channel, &image);
        free(img_resized);
    }
    else
    {
        image.width = frame->cols;
        image.height = frame->rows;
        image.row_stride = image.width * bpp;
        image.data = (unsigned char *)frame->pData;
        image.size = frame->cols * frame->rows * bpp;
        bot_core_image_t_publish(state->lcm, state->channel, &image);
    }

    free(metadata.key);

    return 1;
}


// frame callback, called for every frame that is sent from the camera
void image_callback(fc2Image *in_frame, void *callback_data)
{

    state_t *state = (state_t *)callback_data;
    int64_t utime = timestamp_now();


    fc2Error err;
    // TODO
    //
    // the vimba driver copies the frame.
    // we can't do that here due to an 'internal' structure
    // we don't know the size of.
    // or we create a second image... and 'convert' into it

    // get the actual frame timestamp based on its clock and our clock
    fc2ImageMetadata meta;
    err = fc2GetImageMetadata(in_frame, &meta);

    int64_t frame_utime = timestamp_sync_private(&state->tss, meta->embeddedTimeStamp, utime);

    // Get a pointer to the start of the ancillary data, the Vimba API hasn't implemented this yet
    char *frame_buffer = (char *)in_frame->pData;

    unsigned int exposure = embeddedExposure;
    unsigned int gain = embeddedGain;

    double fps = 1.0 / ((double)(frame_utime / 1e6) - (double)(state->previous_frame_utime / 1e6));
    state->previous_frame_utime = frame_utime;
    double buffer_length = ((double)(state->queue_length * (state->frame_size + sizeof(qframe_t))) / 1024.0) / 1024.0;
    printf("Exp: %u, Gain: %u, fps: %2.2f, utime: %"PRId64", size %d kB, buffer: %3.2f MB, count: %d\n",
           exposure, gain, fps, frame_utime, in_frame->dataSize/1024, buffer_length, state->image_count);

    /*
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
    if(frame->pixelFormat & FC2_PIXEL_FORMAT_RAW12)
        for(int x=0; x<stride*frame->height; x+=2)
        {
            unsigned short *d = (unsigned short *)&frame_buffer[x];
            *d = *d << 4;
        }

    if(state->publish)
    {
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
*/

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
        printf("Path: %s, Length: %d\n", cc->path, (int)strlen(cc->path));
        state->path = realloc(state->path, strlen(cc->path) + 2);
        memset(state->path, 0, strlen(cc->path) + 2);
        memcpy(state->path, cc->path, strlen(cc->path));
        // make sure the last charater is a '/', if not add one
        if(state->path[strlen(state->path) - 1] != '/')
            strcat(state->path, "/");
        printf("Setting path to %s\n", state->path);
        break;
    }
}


int open_camera(state_t *state)
{
    /*
    VmbError_t err;
    printf("Camera_name: %s\n", state->camera_name);

    // Open the camera
    err = VmbCameraOpen((const char *)state->camera_name, VmbAccessModeFull, &state->camera);
    if(err == VmbErrorNotFound)
    {
        fprintf(stderr, "There was a problem opening the Camera (open_camera)\n");
        return 0;
    }

    // Adjust the packet size to match the network
    if (VmbFeatureCommandRun(state->camera, "GVSPAdjustPacketSize" ) == VmbErrorSuccess)
    {
        VmbBool_t bIsCommandDone = 0;
        do
        {
            if (VmbFeatureCommandIsDone(state->camera, "GVSPAdjustPacketSize", &bIsCommandDone ) != VmbErrorSuccess)
                break;
        }
        while (bIsCommandDone == 0);
    }

    // set the camera attributes based on the config file
    if(set_camera_attributes(state) == -1)
    {
        fprintf(stderr, "There was a problem programming the Camera (open_camera)\n");
        return 0;
    }

    // Allocate the frames
    err = VmbFeatureIntGet(state->camera, "PayloadSize", &state->frame_size);
    for(int i=0; i<BUFFERS; i++)
    {
        state->frames[i].buffer = malloc(state->frame_size);
        state->frames[i].bufferSize = state->frame_size;
        state->frames[i].context[0] = state;

        VmbFrameAnnounce(state->camera, &state->frames[i], sizeof(VmbFrame_t));
    }

    // start the capture
    err = VmbCaptureStart(state->camera);

    // Queue frames and register callback
    for(int i=0; i<BUFFERS; i++)
    {
        err = VmbCaptureFrameQueue(state->camera, &state->frames[i], frame_done_callback);
        if(err != VmbErrorSuccess)
            fprintf(stderr, "Frame queue error: %d\n", err);
    }

    // start the acquisition
    err = VmbFeatureCommandRun(state->camera, "AcquisitionStart");

    state->camera_state = CAMERA_IDLE;
*/
    return 1;
}

int close_camera(state_t *state)
{
    /*
    // check to see if the camera is open
    if(state->camera == NULL)
        return 0;

    VmbError_t err;

    // stop acquiring images
    err = VmbFeatureCommandRun(state->camera, "AcquisitionStop");
    if(err == VmbErrorNotFound)
        fprintf(stderr, "There was a problem stopping the acquisition\n");

    err = VmbCaptureEnd(state->camera);
    if(err == VmbErrorNotFound)
        fprintf(stderr, "There was a problem ending the capture\n");


    // dequeue the frames
    VmbFrameRevokeAll(state->camera);

    // Free the memory
    for(int i=0; i<BUFFERS; i++)
    {
        if(state->frames[i].buffer != NULL)
            free(state->frames[i].buffer);
    }

    VmbCameraClose(state->camera);
    state->camera_state = CAMERA_NONE;

    return 1;
    */
}



// The Camera list just changed, we may need to start up a camera if it is the one we are
// looking after
/*void camera_plugged(VmbHandle_t handle , const char* name , void* context)
{
    state_t *state = (state_t *)context;

    char camera_name[255];
    memset(camera_name, 0, sizeof(camera_name));

    VmbFeatureStringGet(handle , "DiscoveryCameraIdent", camera_name, sizeof(camera_name), NULL);
    printf ("Event was fired by camera %s\n", camera_name);

    char *event_type = NULL;
    VmbFeatureEnumGet(handle , "DiscoveryCameraEvent", (const char **)&event_type);
    printf ("Event type %s\n", event_type);

    if(strstr(camera_name, state->mac) != NULL)
    {
        if(!strcmp(event_type, "Detected"))
        {
            // Camera was just attached, lets open it and configure it
            pthread_mutex_lock(&state->camera_lock);
            state->camera_state_change = 1;
            state->camera_state = CAMERA_OPEN;
            state->camera_name = malloc(strlen(camera_name) + 1);
            memset(state->camera_name, 0, strlen(state->camera_name));
            strcpy(state->camera_name, camera_name);
            pthread_mutex_unlock(&state->camera_lock);

        }
        else if(!strcmp(event_type, "Missing"))
        {
            pthread_mutex_lock(&state->camera_lock);
            state->camera_state_change = 1;
            state->camera_state = CAMERA_CLOSE;
            free(state->camera_name);
            pthread_mutex_unlock(&state->camera_lock);
        }

    }

    return;
}*/

void bus_arrive(void *user_data, unsigned int serialNumber)
{
    printf("Bus arrival: %lu\n", serialNumber);
};

void bus_remove(void *user_data, unsigned int serialNumber)
{
    printf("Bus removal: %lu\n", serialNumber);
};

int main(int argc, char **argv)
{
    program_exit = 0;
    signal(SIGINT, signal_handler);
    signal(SIGPIPE, signal_handler);

    //BotParam *params;
    //char root_key[64];
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
    while((opt = getopt(argc, argv, "hk:")) != -1)
    {
        if(opt == 'k')
        {
            strcpy(state.root_key, optarg);
            got_key = 1;
        }
        if(opt == 'h')
        {
            fprintf(stderr, "Usage: blackfly -k <config key>\n");
            return 0;
        }
    }

    if(!got_key)
    {
        fprintf(stderr, "a config file key is required\n");
        return 0;
    }


    state.lcm = lcm_create(NULL);
    state.params = bot_param_new_from_server (state.lcm, 1);

    sprintf(key, "%s.serial", state.root_key);
    state.mac = bot_param_get_int_or_fail(state.params, key);

    sprintf(key, "%s.mac", state.root_key);
    state.mac = bot_param_get_str_or_fail(state.params, key);

    sprintf(key, "%s.channel", state.root_key);
    state.channel = bot_param_get_str_or_fail(state.params, key);

    sprintf(key, "%s.publish", state.root_key);
    state.publish = bot_param_get_boolean_or_fail(state.params, key);

    sprintf(key, "%s.scale", state.root_key);
    state.image_scale = bot_param_get_int_or_fail(state.params, key);
    if((state.image_scale != 1) && (state.image_scale != 4) && (state.image_scale != 16))
    {
        fprintf(stderr, "scale must be 1, 4 or 16\n");
        return 0;
    }


    printf("Publish = %d, Scale = %d\n", state.publish, state.image_scale);

    // start Vimba
    if(fc2CreateGigEContext(state.fc_context) != FC2_ERROR_OK)
    {
        fprintf(stderr, "An error occured creating FlyCapture2 context.\n");
        return 0;
    }

    fc2Error err;

    printf("Installing detection callback\n");
    state.camera_state = CAMERA_NONE;
    state.camera_state_change = 0;
    pthread_mutex_init(&state.camera_lock, 0);
    // Register the callback to detect cameras

    fc2RegisterCallback(state.fc_context, &bus_arrive, FC2_ARRIVAL , &state, NULL);
    fc2RegisterCallback(state.fc_context, &bus_remove, FC2_REMOVAL, &state, NULL);

    // start the write thread and detach it
    //pthread_t tid;
    //pthread_create(&tid, NULL, write_thread, &state);
    //pthread_detach(tid);

    // subscribe to LCM messages
    acfrlcm_auv_camera_control_t_subscribe(state.lcm, "CAMERA_CONTROL", &camera_control_callback, &state);

    printf("Main loop\n");

    // wait here
    while (!program_exit)
    {
        lcm_handle_timeout(state.lcm, 1000);

        // check the camera state
        pthread_mutex_lock(&state.camera_lock);
        if(state.camera_state_change)
        {
            if(state.camera_state == CAMERA_OPEN)
            {
                printf("Opening camera\n");
                if(!open_camera(&state))
                    fprintf(stderr, "There was a problem opening the camera (main)\n");
            }
            else if(state.camera_state == CAMERA_CLOSE)
            {
                printf("closing camera\n");
                close_camera(&state);
            }
            state.camera_state_change = 0;
        }
        pthread_mutex_unlock(&state.camera_lock);
    }

    if (fc2DestoryContext(state.fc_context) != FC2_ERROR_OK)
    {
        fprintf(stderr, "An error occured closing FlyCapture2 context.\n");
        return 0;
    }

    pthread_join(tid, NULL);


}
