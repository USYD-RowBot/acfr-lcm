#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <inttypes.h>
#include <unistd.h>
#include <libgen.h>
#include <limits.h>
#include <glib.h>
#include <ctype.h> // isdigit()
#include <prosilica/PvApi.h>

#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/unix.h"
#include "perls-vision/botimage.h"

#include "perls-lcmtypes/bot_core_image_metadata_t.h"
#include "perls-lcmtypes/bot_core_image_sync_t.h"
#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/senlcm_prosilica_t.h"

#include "prosilica.h"

#define TIMER_PVATTRIBUTES_HZ   1.0
#define TIMER_TIMESTAMP_HZ      0.1

bool gblDone = false;

typedef struct camera camera_t;
struct camera
{
    unsigned long  uid;          // UniqueId
    tPvHandle      handle;       // Camera handle
    int            nFrames;
    tPvFrame      *Frames;
    int64_t        framecount;

    bool           plugged;
    bool           alive;
};

typedef struct driver driver_t;
struct driver
{
    bool        daemon;     // daemon?

    /* driver */
    int         monitor;      // monitor mode
    char       *attfile;      // prosilica camera attributes file
    int         queue;        // driver image frame buffer depth
    int         multicam;     // auto compute StreamBytesPerSecond for ncameras

    /* prosilica */
    int         ConfigFileIndex;         // {'0', '1', '2', '3', '4', '5'}
    char       *ExposureMode;           // {'Manual', 'Auto', 'AutoOnce'}
    uint32_t    ExposureValue;          // microseconds
    double      FrameRate;              // Hz
    char       *FrameStartTriggerMode;  // {'Freerun', 'SyncIn1', 'SyncIn2', 'FixedRate'}
    char       *GainMode;               // {'Manual', 'Auto', 'AutoOnce'}
    uint32_t    GainValue;              // dB
    char       *MulticastEnable;        // {'On', 'Off'}
    char       *PixelFormat;            // {Mono8, Mono16, Bayer8, Bayer16, Rgb24, Rgb48, Yuv411, Yuv422, Yuv444}
    uint32_t    PacketSize;             // Bytes
    uint32_t    StreamBytesPerSecond;   // Bytes per second
    senlcm_prosilica_t pvatt;

    /* logging */
    int         logtodisk;   // log to disk?
    char       *logdir;      // where?
    int         compression; // TIFF compression scheme
    uint8_t     quality;     // 0-100 compression quality, for use with jpeg compression
    char       *strftime;    // image filename format
    int         bayer;       // if color camera, log just the raw bayer pattern w/o interpolation

    /* lcm */
    char       *channel;
    int         publish;     // bool: publish images to lcm ?
    char       *url;         // image lcm url
};

typedef struct lcminfo lcminfo_t;
struct lcminfo
{
    lcm_t *lcm;
    char  *channel_data;
    char  *channel_sync;
    char  *channel_attributes;
};

typedef struct timestamp_sync_private_state timestamp_sync_private_state_t;
struct timestamp_sync_private_state {
    double  dev_ticks_per_second; // how fast does device clock count? (nominal)
    double  max_rate_error;       // how fast do we need to count to ensure we're counting faster than device?

    int64_t sync_host_time;       // when we last synced, what time was it for the host?
    int64_t sync_dev_ticks;       // when we last synced, what device tick was it for the sensor?

    int64_t last_dev_ticks;       // what device time was it when we were last called?

    uint8_t is_valid;             // have we ever synced?
};


typedef struct state state_t;
struct state 
{
    BotParam    *param;
    getopt_t    *gopt;
    camera_t    *camera;
    driver_t    *driver;
    lcminfo_t   *lcminfo;

    timestamp_sync_private_state_t *tss;
    GMutex                         *tss_mutex;

    char   *rootkey;
    char   *rootdir;
};

typedef struct write_data write_data_t;
struct write_data
{
    tPvFrame *Frame;
    state_t  *state;
    int32_t   framecount;
    int64_t   utime;
};


static timestamp_sync_private_state_t *
timestamp_sync_private_init (double dev_ticks_per_second, double rate)
{
    timestamp_sync_private_state_t *s = calloc (1, sizeof (*s));
    if (!s)
        return NULL;

    s->dev_ticks_per_second = dev_ticks_per_second;
    s->max_rate_error = rate;
    s->is_valid = 0;

    return s;
}

static void
timestamp_sync_private_free (timestamp_sync_private_state_t *s)
{
    free (s);
}


static int64_t
timestamp_sync_private (timestamp_sync_private_state_t *s, int64_t dev_ticks,
                        int64_t host_utime)
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
#if 0
    printf ("%"PRId64"  %"PRId64"  %+6"PRId64"  %"PRId64"\n", 
            host_utime, dev_utime, time_err, dev_ticks);
#endif

    return dev_utime;
}


static void
low_latency_timestamp_sync (state_t *state)
{
    // update timestamp_sync object w/ a low-latency correspondence between pc utime and internal camera time
    unsigned long TimestampLo, TimestampHi;
    int64_t utime_before = timestamp_now ();
    int ret = prosilica_get_timestamp (state->camera->handle, &TimestampLo, &TimestampHi);
    int64_t utime_after = timestamp_now ();
    if (ret == 0) {
        int64_t Timestamp = ((int64_t)TimestampLo) + (((int64_t)TimestampHi)<<32);
        int64_t utime_raw = (utime_before + utime_after)/2; // use the midpoint as the actual event time
        timestamp_sync_private (state->tss, Timestamp, utime_raw);
    }
    else
        PROSILICA_ERROR (NULL, "prosilica_get_timestamp()");
}

static int
timer_callback_pvattributes (void *context)
{
    state_t *state = context;

    static bool firsttime = true;
    if (firsttime) {
        timeutil_sleep (2);
        firsttime = false;
    }

    if (state->camera->alive) {
        // query and publish pvattributes
        senlcm_prosilica_t *pvatt = prosilica_get_pvattributes (state->camera->handle);
        if (pvatt != NULL) {
            pvatt->self = 1;
            pvatt->utime = timestamp_now ();
            senlcm_prosilica_t_publish (state->lcminfo->lcm, state->lcminfo->channel_attributes, pvatt);
            senlcm_prosilica_t_destroy (pvatt);
        }
    }

    if (!state->camera->plugged)
        printf (".");

    return 1;
}

static int
timer_callback_timestamp (void *context)
{
    state_t *state = context;

    if (state->camera->alive) {
        g_mutex_lock (state->tss_mutex);
        low_latency_timestamp_sync (state);
        g_mutex_unlock (state->tss_mutex);
    }

    return 1;
}



static gpointer
write_thread (void *context)
{
    write_data_t *wdata = context;
    
    char filename[NAME_MAX];
    vis_botimage_filename (filename, sizeof filename, wdata->state->driver->strftime, wdata->utime, wdata->framecount);
printf ("Saving image %s\n", filename);

    bool color_interpolate = wdata->state->driver->bayer ? false : true;
    prosilica_save_tiff (wdata->Frame, wdata->state->camera->uid, wdata->state->driver->logdir, 
                         filename, color_interpolate, wdata->state->driver->compression, wdata->state->driver->quality);

    prosilica_free_frames (wdata->Frame, 1);
    free (wdata);

    g_thread_exit (0);
    return NULL;
}


static void
FrameDoneCB (tPvFrame *Frame)
{
    int64_t utime_raw = timestamp_now ();

    state_t *state = Frame->Context[0];

    static int64_t utime_prev = 0;
    static double fps = 0.0;
    const  double alpha = 0.25;

    switch (Frame->Status) {
    case ePvErrSuccess: {
        // timestamp_sync the actual camera frame event
        int32_t framecount = state->camera->framecount++;
        int64_t Timestamp = ((int64_t)Frame->TimestampLo) + (((int64_t)Frame->TimestampHi)<<32);
        g_mutex_lock (state->tss_mutex);
        int64_t utime = timestamp_sync_private (state->tss, Timestamp, utime_raw);
        if (utime == utime_raw) // our tss model is out of sync
            low_latency_timestamp_sync (state);
        g_mutex_unlock (state->tss_mutex);

        // compute avg framerate
        int64_t dt = utime - utime_prev;
        fps = alpha*fps + (1-alpha)*1./(dt*1E-6);
        utime_prev = utime;

        // publish image sync over lcm
        bot_core_image_sync_t sync = { .utime = utime };
        bot_core_image_sync_t_publish (state->lcminfo->lcm, state->lcminfo->channel_sync, &sync);

        // clone and requeue the original Frame to the camera
        tPvFrame *dupFrame = prosilica_dup_frames (Frame, 1);
        if (!gblDone)
            PvCaptureQueueFrame (state->camera->handle, Frame, FrameDoneCB);

        // bit shift for easy viewing
        if (dupFrame->BitDepth > 8 && (state->driver->publish || state->driver->logtodisk))
            prosilica_bitshift_frame (dupFrame);

        // publish image data over lcm?
        if (state->driver->publish) {
            bot_core_image_t image = vis_pvframe_to_botimage_view (dupFrame, utime);
            
            // add exposure info to metadata of image
            unsigned long ExpValue;
            PvAttrUint32Get (state->camera->handle, "ExposureValue", &ExpValue);
            //PvAttrUint32Get (state->camera->handle, "ExposureValue", (tPvUint32 *)&(state->driver->ExposureValue));
            state->driver->ExposureValue = (uint32_t) ExpValue;
	        bot_core_image_metadata_t metadata;
	        char *key = "ExposureValue";
	        metadata.key =malloc(strlen(key));
	        strcpy(metadata.key,key);
	        metadata.n = 4;
	        metadata.value =  (uint8_t *) &(state->driver->ExposureValue);
	        image.nmetadata = 1; 
	        image.metadata=malloc(sizeof(  bot_core_image_metadata_t)* image.nmetadata);
            memcpy(image.metadata,&metadata,sizeof(bot_core_image_metadata_t));
            
            bot_core_image_t_publish (state->lcminfo->lcm, state->lcminfo->channel_data, &image);
        }

        // write to disk?
        if (state->driver->logtodisk) {
            write_data_t *wdata = malloc (sizeof (*wdata));
            wdata->Frame = dupFrame;
            wdata->state = state;
            wdata->framecount = framecount;
            wdata->utime = utime;

            GThread *tid = g_thread_create (&write_thread, wdata, 0, NULL);
            if (tid == NULL) {
                free (wdata);
                PROSILICA_ERROR (NULL, "unable to create write_thread");
            }
        }
        else
            prosilica_free_frames (dupFrame, 1);

        // frame count
        printf ("fps: %-10.2lf dt: %-10"PRId64" frame: %-10d\n", fps, dt, framecount);

        break;
    }
    case ePvErrCancelled:
        break;
    case ePvErrUnplugged:
        break;
    default:
        if (!gblDone) {
            PROSILICA_ERROR (Frame->Status, "");
            PvCaptureQueueFrame (state->camera->handle, Frame, FrameDoneCB);
        }
    }
}


static void
PvLinkAdd (state_t *state)
{
    tPvErr err;

    // fail-safe open the camera
    int attempts = 0;
    while (!state->camera->alive) {
        timeutil_sleep (1);

        if (prosilica_camera_online (state->camera->uid)) {
            if (state->driver->monitor)
                err = PvCameraOpen (state->camera->uid, ePvAccessMonitor, &state->camera->handle);
            else
                err = PvCameraOpen (state->camera->uid, ePvAccessMaster, &state->camera->handle);
            
            if (err == ePvErrSuccess)
                state->camera->alive = true;
            else {
                if (++attempts > 1 )
                    PROSILICA_ERROR (err, "PvCameraOpen()");
                timeutil_sleep (5);
            }
        }
    }

    // create a timestamp_sync object
    if (state->tss != NULL)
        timestamp_sync_private_free (state->tss);
    unsigned long TimeStampFrequency;
    err = PvAttrUint32Get (state->camera->handle, "TimeStampFrequency",  &TimeStampFrequency);
    if (err != ePvErrSuccess)
        PROSILICA_ERROR (err, "PvAttrUint32Get(\"TimeStampFrequency\")");
    state->tss = timestamp_sync_private_init (TimeStampFrequency, 1);
    low_latency_timestamp_sync (state);

    // configure camera settings
    if (!state->driver->monitor) {
        err = PvCaptureAdjustPacketSize (state->camera->handle, PROSILICA_MAX_PACKET_SIZE);
        if (err != ePvErrSuccess)
            PROSILICA_ERROR (err, "PvCaputerAdjustPacketSize()");

        // restore factory config and set StreamBytesPerSecond to max, otherwise FrameRate and other settings may not take
        prosilica_load_config (state->camera->handle, 0);
        err = PvAttrUint32Set (state->camera->handle, "StreamBytesPerSecond", PROSILICA_STREAMBYTESPERSECOND_MAX);
        if (err != ePvErrSuccess)
            PROSILICA_ERROR (err, "PvAttrUint32Set()");

        // load any custom config settings
        if (state->driver->attfile) {
            if (prosilica_load_attfile (state->camera->handle, state->driver->attfile))
                PROSILICA_ERROR (NULL, "prosilica_load_attfile() failed to load %s", state->driver->attfile);
            else
                printf ("Loaded attfile=%s\n", state->driver->attfile);
        }

        // send over any master.cfg PvAttributes
        prosilica_set_pvattributes (state->camera->handle, &state->driver->pvatt);

        // ensure that command line options, if present, override
        if (getopt_has_flag (state->gopt, "ConfigFileIndex"))
            prosilica_load_config (state->camera->handle, state->driver->ConfigFileIndex);
        if (getopt_has_flag (state->gopt, "ExposureValue")) {
            char v[24];
            snprintf (v, sizeof v, "%d", state->driver->ExposureValue);
            prosilica_set_attribute (state->camera->handle, "ExposureValue", v);
        }
        if (getopt_has_flag (state->gopt, "ExposureMode"))
            prosilica_set_attribute (state->camera->handle, "ExposureMode", state->driver->ExposureMode);
        if (getopt_has_flag (state->gopt, "FrameRate")) {
            char v[24];
            snprintf (v, sizeof v, "%g", state->driver->FrameRate);
            prosilica_set_attribute (state->camera->handle, "FrameRate", v);
        }
        if (getopt_has_flag (state->gopt, "FrameStartTriggerMode"))
            prosilica_set_attribute (state->camera->handle, "FrameStartTriggerMode", state->driver->FrameStartTriggerMode);
        if (getopt_has_flag (state->gopt, "GainValue")) {
            char v[24];
            snprintf (v, sizeof v, "%d", state->driver->GainValue);
            prosilica_set_attribute (state->camera->handle, "GainValue", v);
        }
        if (getopt_has_flag (state->gopt, "GainMode"))
            prosilica_set_attribute (state->camera->handle, "GainMode", state->driver->GainMode);
        if (getopt_has_flag (state->gopt, "MulticastEnable"))
            prosilica_set_attribute (state->camera->handle, "MulticastEnable", state->driver->MulticastEnable);
        if (getopt_has_flag (state->gopt, "PixelFormat"))
            prosilica_set_attribute (state->camera->handle, "PIxelFormat", state->driver->PixelFormat);
        if (getopt_has_flag (state->gopt, "PacketSize")) {
            char v[24];
            snprintf (v, sizeof v, "%d", state->driver->PacketSize);
            prosilica_set_attribute (state->camera->handle, "PacketSize", v);
        }
        if (getopt_has_flag (state->gopt, "StreamBytesPerSecond")) {
            char v[24];
            snprintf (v, sizeof v, "%d", state->driver->StreamBytesPerSecond);
            prosilica_set_attribute (state->camera->handle, "StreamBytesPerSecond", v);
        }
            

        // auto StreamBytesPerSecond
        if (getopt_has_flag (state->gopt, "multicam")) {
            long ret = prosilica_multicam (state->camera->handle, state->driver->multicam);
            if (ret < 0)
                PROSILICA_ERROR (NULL, "prosilica_multicam: unable to auto set StreamBytesPerSecond");
        }

        err = PvCaptureStart (state->camera->handle);
        if (err != ePvErrSuccess)
            PROSILICA_ERROR (err, "PvCaptureStart()");
    }

    // alloc frame buffer and queue frames
    state->camera->nFrames = state->driver->queue;
    state->camera->Frames = prosilica_alloc_frames (state->camera->handle, state->camera->nFrames);
    if (state->camera->Frames == NULL) {
        PROSILICA_ERROR (-1, "unable to alloc frame buffer memory - abort");
        abort ();
    }
    for (int i=0; i<state->camera->nFrames; i++) {
        state->camera->Frames[i].Context[0] = state;
        PvCaptureQueueFrame (state->camera->handle, state->camera->Frames+i, FrameDoneCB);
    }

    // user info
    unsigned long TotalBytesPerFrame;
    PvAttrUint32Get (state->camera->handle, "TotalBytesPerFrame", &TotalBytesPerFrame);
    char PixelFormat[256], type[32];
    PvAttrEnumGet (state->camera->handle, "PixelFormat", PixelFormat, sizeof PixelFormat, NULL);
    if (state->driver->monitor)
        sprintf (type, "Monitor");
    else
        sprintf (type, "Master");
    printf ("camera %lu opened and ready: %s, %s, TotalBytesPerFrame=%lu\n", 
            state->camera->uid, type, PixelFormat, TotalBytesPerFrame);

    // bandwidth
    prosilica_bandwidth_t bw;
    int ret = prosilica_bandwidth_used (state->camera->handle, &bw);
    if (ret < 0)
        PROSILICA_ERROR (ret, "prosilica_bandwidth_used()");
    else
        printf ("StreamBytesPerSecond=%ld   Used=%ld   (%.2f%%)\n",
                bw.StreamBytesPerSecond, bw.Used, bw.Percentage);

    // go forth!
    if (!getopt_get_bool (state->gopt, "stop"))
        PvCommandRun (state->camera->handle, "AcquisitionStart");
}


static void
PvLinkRemove (state_t *state)
{
    tPvErr err;
    state->camera->alive = false;

    if (!state->camera->handle) {
        PROSILICA_ERROR (NULL, "invalid state->camera->handle");
        return;
    }

    err = PvCaptureQueueClear (state->camera->handle);
    if (err != ePvErrSuccess)
        PROSILICA_ERROR (err, "PvCaptureQueueClear()");

    err = PvCaptureEnd (state->camera->handle);
    if (err != ePvErrSuccess)
        PROSILICA_ERROR (err, "PvCameraEnd()");

    err = PvCameraClose (state->camera->handle);
    if (err != ePvErrSuccess)
        PROSILICA_ERROR (err, "PvCameraClose()");
    else
        printf ("camera %lu closed\n", state->camera->uid);

    prosilica_free_frames (state->camera->Frames, state->camera->nFrames);
}


static void
LinkEventCB (void* context, tPvInterface interface, tPvLinkEvent event, unsigned long uid)
{
    state_t *state = context;

    if (uid != state->camera->uid)
        return;

    switch(event) {
    case ePvLinkAdd: {
        state->camera->plugged = true;
        printf ("camera %lu plugged\n", uid);
        PvLinkAdd (state);
        break;
    }
    case ePvLinkRemove: {
        printf ("camera %lu unplugged\n", uid);
        PvLinkRemove (state);
        state->camera->plugged = false;
        printf ("camera %ld waiting\n", state->camera->uid);
        break;
    }
    default:
        break;
    }
}


static void
pvattributes_callback (const lcm_recv_buf_t *rbuf, const char *channel, 
                       const senlcm_prosilica_t *pvatt, void *user)
{
    state_t *state = user;
    if (!pvatt->self && !state->driver->monitor && prosilica_camera_online (state->camera->uid))
        prosilica_set_pvattributes (state->camera->handle, pvatt);
}


static state_t *
state_create (int argc, char *argv[])
{
    // init state
    state_t *state = (state_t *) calloc (1, sizeof (state_t));
    
    state->gopt = getopt_create ();
    state->camera = (camera_t *) calloc (1, sizeof (camera_t));
    state->driver = (driver_t *) calloc (1, sizeof (driver_t));
    state->lcminfo = (lcminfo_t *) calloc (1, sizeof (lcminfo_t));

    state->tss_mutex = g_mutex_new ();

    state->rootdir = get_current_dir_name ();
    char rootkey[256];
    sprintf (rootkey, "sensors.%s", basename (argv[0]));
    state->rootkey = strdup (rootkey);
    return state;
}


static void
state_destroy (state_t *state)
{
    if (state->param)
        bot_param_destroy (state->param);
    if (state->gopt)
        getopt_destroy (state->gopt);
    if (state->camera)
        free (state->camera);
    if (state->driver)
        free (state->driver);

    if (state->tss != NULL)
        timestamp_sync_private_free (state->tss);
    if (state->tss_mutex)
        g_mutex_free (state->tss_mutex);        

    if (state->rootkey)
        free (state->rootkey);
    if (state->rootdir)
        free (state->rootdir);
    if (state)
        free (state);
}


static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    if (gblDone)
        abort ();
    else
        gblDone = true;
}


static void
parse_args (state_t *state, int argc, char *argv[])
{
    char key[256];

   /* Add std command-line options */
    getopt_add_description (state->gopt, "Prosilica Gig-E camera driver.");
    getopt_add_help   (state->gopt, NULL);
    getopt_add_bool   (state->gopt, 'D',  "daemon",     0,                   "Run as daemon");
    getopt_add_string (state->gopt, 'k',  "key",        state->rootkey,      "Config file key");
    getopt_add_spacer (state->gopt, "");

    getopt_add_spacer (state->gopt, "Driver Options:");
    getopt_add_long   (state->gopt, 'u',  "uid",         "",                 "UniqueId");
    getopt_add_bool   (state->gopt, 'M',  "monitor",     0,                  "Open camera driver in Monitor instead of Master mode");
    getopt_add_string (state->gopt, 'a',  "attfile",     "",                 "Camera attributes file");
    getopt_add_int    (state->gopt, 'q',  "queue",       "20",               "Image frame buffer queue depth");
    getopt_add_int    (state->gopt, 'm',  "multicam",    "1",                "Number of cameras to use in auto StreamBytesPerSecond calc");
    getopt_add_bool   (state->gopt, 's',  "stop",        0,                  "If true, don't immediately start acquiring images");
    getopt_add_spacer (state->gopt, "");

    getopt_add_spacer (state->gopt, "Prosilica Options:");
    getopt_add_string (state->gopt, '\0', "ConfigFileIndex",       "",       "{'Factory', '1', '2', '3', '4', '5'}");
    getopt_add_string (state->gopt, '\0', "ExposureMode",          "",       "{'Manual', 'Auto', 'AutoOnce'}");
    getopt_add_double (state->gopt, '\0', "ExposureValue",         "",       "usec (valid in manual mode)");
    getopt_add_double (state->gopt, '\0', "FrameRate",             "",       "valid in FixedRate mode");
    getopt_add_string (state->gopt, '\0', "FrameStartTriggerMode", "",       "{'Freerun', 'SyncIn1', 'SyncIn2', 'FixedRate'}");
    getopt_add_string (state->gopt, '\0', "GainMode",              "",       "{'Manual', 'Auto', 'AutoOnce'}");
    getopt_add_double (state->gopt, '\0', "GainValue",             "",       "dB (valid in manual gain mode)");
    getopt_add_bool   (state->gopt, '\0', "MulticastEnable",       0,        "Multicast stream instead of unicasting it");
    getopt_add_string (state->gopt, '\0', "PixelFormat",           "",       "{Mono8, Mono16, Bayer8, Bayer16, Rgb24, Rgb48, Yuv411, Yuv422, Yuv444}");
    getopt_add_int    (state->gopt, '\0', "PacketSize",            "",       "Bytes");
    getopt_add_long   (state->gopt, '\0', "StreamBytesPerSecond",  "",       "Bytes per second");
    getopt_add_spacer (state->gopt, "");

    getopt_add_spacer (state->gopt, "Logging Options:");
    getopt_add_string (state->gopt, 'd',  "logdir",      "./",               "Image directory");
    getopt_add_bool   (state->gopt, 'l',  "logtodisk",   0,                  "Log .tif images to disk");
    getopt_add_string (state->gopt, '\0', "compression", "none",             ".tif compression scheme {none,packbits,lzw,jpeg,jpeg:xx,deflate}");
    getopt_add_string (state->gopt, '\0', "strftime",    "IMG-%s.%i-%f.tif", "Image file name strftime format, %f for frame number, %i for usecs");
    getopt_add_bool   (state->gopt, '\0', "bayer",       0,                  "Log raw Bayer pattern");
    getopt_add_spacer (state->gopt, "");

    getopt_add_spacer (state->gopt, "LCM:");
    getopt_add_string (state->gopt, 'c',  "channel",     "PROSILICA",        "LCM channel, e.g., SENSOR_FOO");
    getopt_add_bool   (state->gopt, '\0', "nopublish",   0,                  "disable LCM image_t data frame publishing");
    getopt_add_string (state->gopt, '\0', "url",         "",                 "LCM publish url (default is NULL)");

    if (!getopt_parse (state->gopt, argc, argv, 1) || state->gopt->extraargs->len!=0) {
        getopt_do_usage (state->gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (state->gopt, "help")) {
        getopt_do_usage (state->gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    // daemon
    state->driver->daemon = getopt_get_bool (state->gopt, "daemon");

    // key
    state->rootkey = strdup (getopt_get_string (state->gopt, "key"));

    /* DRIVER OPTS */

    // uid
    sprintf (key, "%s.uid", state->rootkey);
    int uid_int;
    if (getopt_has_flag (state->gopt, "uid"))
        state->camera->uid = getopt_get_int (state->gopt, "uid");
    else if (bot_param_get_int (state->param, key, &uid_int) == 0)
        state->camera->uid = uid_int;
    else {
        ERROR ("unspecified camera uid");
        exit (EXIT_FAILURE);
    }

    // monitor
    sprintf (key, "%s.monitor", state->rootkey);
    if (getopt_has_flag (state->gopt, "monitor"))
        state->driver->monitor = getopt_get_bool (state->gopt, "monitor");
    else if (bot_param_get_boolean (state->param, key, &state->driver->monitor) != 0)
        state->driver->monitor = getopt_get_bool (state->gopt, "monitor"); /* default */

    // attfile
    sprintf (key, "%s.attfile", state->rootkey);
    if (getopt_has_flag (state->gopt, "attfile"))
        state->driver->attfile = strdup (getopt_get_string (state->gopt, "attfile"));
    else if (bot_param_get_str (state->param, key, &state->driver->attfile) != 0)
        state->driver->attfile = strdup (getopt_get_string (state->gopt, "attfile"));
    if (strlen (state->driver->attfile) == 0) {
        free (state->driver->attfile);
        state->driver->attfile = NULL;
    }
    
    // queue
    sprintf (key, "%s.queue", state->rootkey);
    if (getopt_has_flag (state->gopt, "queue"))
        state->driver->queue = getopt_get_int (state->gopt, "queue");
    else if (bot_param_get_int (state->param, key, &state->driver->queue) != 0)
        state->driver->queue = getopt_get_int (state->gopt, "queue"); /* default */

    // multicam
    sprintf (key, "%s.multicam", state->rootkey);
    if (getopt_has_flag (state->gopt, "multicam"))
        state->driver->multicam = getopt_get_int (state->gopt, "multicam");
    else if (bot_param_get_int (state->param, key, &state->driver->multicam) != 0)
        state->driver->multicam = getopt_get_int (state->gopt, "multicam"); /* default */

    /* PROSILICA OPTS */

    // ConfigFileIndex
    if (getopt_has_flag (state->gopt, "ConfigFileIndex")) {
        int index = 0;
        const char *index_str = getopt_get_string (state->gopt, "ConfigFileIndex");
        if (0==strcasecmp (index_str, "Factory"))
            state->driver->ConfigFileIndex = 0;
        else if (isdigit (index_str[0]) && (index=atoi (index_str)) && 0<index && index <= 5)
            state->driver->ConfigFileIndex = index;
        else {
            PROSILICA_ERROR (NULL, "unrecognized argument to --ConfigFileIndex");
            exit (EXIT_FAILURE);
        }
    }

    // ExposureValue
    if (getopt_has_flag (state->gopt, "ExposureValue"))
        state->driver->ExposureValue = getopt_get_int (state->gopt, "ExposureValue");

    // ExposureMode
    if (getopt_has_flag (state->gopt, "ExposureMode")) {
        const char *expmode = getopt_get_string (state->gopt, "ExposureMode");
        if (0==strcasecmp (expmode, "Manual"))
            state->driver->ExposureMode = strdup ("Manual");
        else if (0==strcasecmp (expmode, "Auto"))
            state->driver->ExposureMode = strdup ("Auto");
        else if (0==strcasecmp (expmode, "AutoOnce"))
            state->driver->ExposureMode = strdup ("AutoOnce");
        else {
            PROSILICA_ERROR (NULL, "unrecognized argument to --ExposureMode");
            exit (EXIT_FAILURE);
        }
    }

    // FrameRate
    if (getopt_has_flag (state->gopt, "FrameRate"))
        state->driver->FrameRate = getopt_get_double (state->gopt, "FrameRate");

    // FrameStartTriggerMode
    if (getopt_has_flag (state->gopt, "FrameStartTriggerMode")) {
        const char *trigger = getopt_get_string (state->gopt, "FrameStartTriggerMode");
        if (0==strcasecmp (trigger, "Freerun"))
            state->driver->FrameStartTriggerMode = strdup ("Freerun");
        else if (0==strcasecmp (trigger, "SyncIn1"))
            state->driver->FrameStartTriggerMode = strdup ("SyncIn1");
        else if (0==strcasecmp (trigger, "SyncIn2"))
            state->driver->FrameStartTriggerMode = strdup ("SyncIn2");
        else if (0==strcasecmp (trigger, "FixedRate"))
            state->driver->FrameStartTriggerMode = strdup ("FixedRate");
        else {
            PROSILICA_ERROR (NULL, "unrecognized argument to --FrameStartTriggerMode");
            exit (EXIT_FAILURE);
        }
    }

    // MulticastEnable
    if (getopt_get_bool (state->gopt, "MulticastEnable"))
        state->driver->MulticastEnable = strdup ("On");
    else
        state->driver->MulticastEnable = strdup ("Off");


    // GainValue
    if (getopt_has_flag (state->gopt, "GainValue"))
        state->driver->GainValue = getopt_get_int (state->gopt, "GainValue");

    // GainMode
    if (getopt_has_flag (state->gopt, "GainMode")) {
        const char *gainmode = getopt_get_string (state->gopt, "GainMode");
        if (0==strcasecmp (gainmode, "Manual"))
            state->driver->GainMode = strdup ("Manual");
        else if (0==strcasecmp (gainmode, "Auto"))
            state->driver->GainMode = strdup ("Auto");
        else if (0==strcasecmp (gainmode, "AutoOnce"))
            state->driver->GainMode = strdup ("AutoOnce");
        else {
            PROSILICA_ERROR (NULL, "unrecognized argument to --GainMode");
            exit (EXIT_FAILURE);
        }
    }

    // PixelFormat
    if (getopt_has_flag (state->gopt, "PixelFormat")) {
        const char *pixelformat = getopt_get_string (state->gopt, "PixelFormat");
        if (0==strcasecmp (pixelformat, "Mono8"))
            state->driver->PixelFormat = strdup ("Mono8");
        else if (0==strcasecmp (pixelformat, "Mono16"))
            state->driver->PixelFormat = strdup ("Mono16");
        else if (0==strcasecmp (pixelformat, "Bayer8"))
            state->driver->PixelFormat = strdup ("Bayer8");
        else if (0==strcasecmp (pixelformat, "Bayer16"))
            state->driver->PixelFormat = strdup ("Bayer16");
        else if (0==strcasecmp (pixelformat, "Rgb24"))
            state->driver->PixelFormat = strdup ("Rgb24");
        else if (0==strcasecmp (pixelformat, "Rgb48"))
            state->driver->PixelFormat = strdup ("Rgb48");
        else if (0==strcasecmp (pixelformat, "Yuv411"))
            state->driver->PixelFormat = strdup ("Yuv411");
        else if (0==strcasecmp (pixelformat, "Yuv422"))
            state->driver->PixelFormat = strdup ("Yuv422");
        else if (0==strcasecmp (pixelformat, "Yuv444"))
            state->driver->PixelFormat = strdup ("Yuv444");
        else if (0==strcasecmp (pixelformat, "Bgr24"))
            state->driver->PixelFormat = strdup ("Bgr24");
        else if (0==strcasecmp (pixelformat, "Rgba32"))
            state->driver->PixelFormat = strdup ("Rgba32");
        else if (0==strcasecmp (pixelformat, "Bgra32"))
            state->driver->PixelFormat = strdup ("Bgra32");
        else {
            PROSILICA_ERROR (NULL, "unrecognized argument to --PixelFormat");
            exit (EXIT_FAILURE);
        }
    }

    // PacketSize
    if (getopt_has_flag (state->gopt, "PacketSize"))
        state->driver->PacketSize = getopt_get_int (state->gopt, "PacketSize");

    // StreamBytesPerSecond
    if (getopt_has_flag (state->gopt, "StreamBytesPerSecond"))
        state->driver->StreamBytesPerSecond = getopt_get_long (state->gopt, "StreamBytesPerSecond");

    // PvAttributes
    sprintf (key, "%s.PvAttributes", state->rootkey);
    int nkeys = bot_param_get_num_subkeys (state->param, key);
    char **subkeys = bot_param_get_subkeys (state->param, key);
    state->driver->pvatt.n_attributes = nkeys;
    state->driver->pvatt.PvAttributes = calloc (nkeys, sizeof (*state->driver->pvatt.PvAttributes));
    for (int i=0; i<nkeys; i++) {
        char *subkey = subkeys[i];
        state->driver->pvatt.PvAttributes[i].label = subkey;
        char subkeyfull[256];
        sprintf (subkeyfull, "%s.%s", key, subkey);
        bot_param_get_str (state->param, subkeyfull, &state->driver->pvatt.PvAttributes[i].value);
    }


    /* LOGGING OPTS */

    // logtodisk
    sprintf (key, "%s.logtodisk", state->rootkey);
    if (getopt_has_flag (state->gopt, "logtodisk"))
        state->driver->logtodisk = getopt_get_bool (state->gopt, "logtodisk");
    else if (bot_param_get_boolean (state->param, key, &state->driver->logtodisk) != 0)
        state->driver->logtodisk = getopt_get_bool (state->gopt, "logtodisk"); /* default */

    // logdir
    sprintf (key, "%s.logdir", state->rootkey);
    if (getopt_has_flag (state->gopt, "logdir"))
        state->driver->logdir = strdup (getopt_get_string (state->gopt, "logdir"));
    else if (bot_param_get_str (state->param, key, &state->driver->logdir) != 0)
        state->driver->logdir = strdup (getopt_get_string (state->gopt, "logdir")); /* default */

    
    // compression
    sprintf (key, "%s.compression", state->rootkey);
    char *compression;
    if (getopt_has_flag (state->gopt, "compression"))
        compression = (char *) getopt_get_string (state->gopt, "compression");
    else if (bot_param_get_str (state->param, key, &compression) != 0)
        compression = (char *) getopt_get_string (state->gopt, "compression"); /* default */

    state->driver->quality = 95;
    if (0==strcasecmp (compression, "none"))
        state->driver->compression = COMPRESSION_NONE;
    else if (0==strcasecmp (compression, "jpeg")) 
        state->driver->compression = COMPRESSION_JPEG;
    else if (0==strncasecmp (compression, "jpeg:",5)) {
        state->driver->compression = COMPRESSION_JPEG;
        unsigned int q;
        if (1==sscanf (compression, "jpeg:%u", &q))
            state->driver->quality = q;
        else
            PROSILICA_ERROR (NULL, "jpeg quality not parseable");
    }
    else if (0==strcasecmp (compression, "packbits"))
        state->driver->compression = COMPRESSION_PACKBITS;
    else if (0==strcasecmp (compression, "lzw"))
        state->driver->compression = COMPRESSION_LZW;
    else if (0==strcasecmp (compression, "deflate"))
        state->driver->compression = COMPRESSION_DEFLATE;
    else {
        PROSILICA_ERROR (NULL, "unrecognized argument to --compression");
        exit (EXIT_FAILURE);
    }
    

    // strftime
    sprintf (key, "%s.strftime", state->rootkey);
    if (getopt_has_flag (state->gopt, "strftime"))
        state->driver->strftime = strdup (getopt_get_string (state->gopt, "strftime"));
    else if (bot_param_get_str (state->param, key, &state->driver->strftime) != 0)
        state->driver->strftime = strdup (getopt_get_string (state->gopt, "strftime"));

    // bayer
    sprintf (key, "%s.bayer", state->rootkey);
    if (getopt_has_flag (state->gopt, "bayer"))
        state->driver->bayer = getopt_get_bool (state->gopt, "bayer");
    else if (bot_param_get_boolean (state->param, key, &state->driver->bayer) != 0)
        state->driver->bayer = getopt_get_bool (state->gopt, "bayer"); /* default */


    /* LCM OPTS */

    // channel
    sprintf (key, "%s.channel", state->rootkey);
    if (getopt_has_flag (state->gopt, "channel"))
        state->driver->channel = strdup (getopt_get_string (state->gopt, "channel"));
    else if (bot_param_get_str (state->param, key, &state->driver->channel) != 0)
        state->driver->channel = strdup (getopt_get_string (state->gopt, "channel")); /* default */

    char channel[LCM_MAX_CHANNEL_NAME_LENGTH];
    snprintf (channel, LCM_MAX_CHANNEL_NAME_LENGTH, "%s", state->driver->channel);
    state->lcminfo->channel_data = strdup (channel);

    snprintf (channel, LCM_MAX_CHANNEL_NAME_LENGTH, "%s.%s", state->driver->channel, "SYNC");
    state->lcminfo->channel_sync = strdup (channel);

    snprintf (channel, LCM_MAX_CHANNEL_NAME_LENGTH, "%s.%s", state->driver->channel, "ATTRIBUTES");
    state->lcminfo->channel_attributes = strdup (channel);


    // nopublish
    sprintf (key, "%s.publish", state->rootkey);
    if (getopt_has_flag (state->gopt, "nopublish"))
        state->driver->publish = !getopt_get_bool (state->gopt, "nopublish");
    else if (bot_param_get_boolean (state->param, key, &state->driver->publish) != 0)
        state->driver->publish = !getopt_get_bool (state->gopt, "nopublish"); /* default */

printf("LCM publish state: %d\n",  state->driver->publish);
    // url
    sprintf (key, "%s.url", state->rootkey);
    if (getopt_has_flag (state->gopt, "url"))
        state->driver->url = strdup (getopt_get_string (state->gopt, "url"));
    else if (bot_param_get_str (state->param, key, &state->driver->url) != 0)
        state->driver->url = strdup (getopt_get_string (state->gopt, "url")); /* default */

    if (strlen (state->driver->url) == 0) {
        free (state->driver->url);
        state->driver->url = NULL;
    }
}


int main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    if (!g_thread_supported ()) 
        g_thread_init (NULL);

    state_t *state = state_create (argc, argv);
    // fire up lcm
    state->lcminfo->lcm = lcm_create (NULL);
    if (!state->lcminfo->lcm) {
        ERROR ("lcm_create() failed");
        exit (EXIT_FAILURE);
    }
    state->param = bot_param_new_from_server (state->lcminfo->lcm, 1);
    parse_args (state, argc, argv);
    
    printf("Got param\n");
    // make sure logging directory exists
    if (state->driver->logtodisk && unix_mkpath (state->driver->logdir, 0775) < 0) {
        PERROR ("unix_mkpath()");
        exit (EXIT_FAILURE);
    }

    if (getopt_get_bool (state->gopt, "daemon")) {
        daemon_fork ();
        close (STDERR_FILENO);
    }

    // bring up the API
    tPvErr err = PvInitialize ();
    if (err != ePvErrSuccess) {
        PROSILICA_ERROR (err, "PvInitialize()");
        exit (EXIT_FAILURE);
    }
    
    printf ("camera %ld waiting\n", state->camera->uid);

    // register a callback for when cameras enter/leave the bus
    PvLinkCallbackRegister (LinkEventCB, ePvLinkAdd, state);
    PvLinkCallbackRegister (LinkEventCB, ePvLinkRemove, state);


    
    senlcm_prosilica_t_subscription_t *sub = 
        senlcm_prosilica_t_subscribe (state->lcminfo->lcm, state->lcminfo->channel_attributes,
                                      &pvattributes_callback, state);

    // install custom signal handler
    struct sigaction act = {
        .sa_sigaction = my_signal_handler,
    };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);


    // launch pvattributes timer
    struct timespec spec = timeutil_hz_to_timespec (TIMER_PVATTRIBUTES_HZ);
    timeutil_timer_t timer_pvattributes = timeutil_timer_create (spec, &timer_callback_pvattributes, state);

    // launch timestamp timer
    spec = timeutil_hz_to_timespec (TIMER_TIMESTAMP_HZ);
    timeutil_timer_t timer_timestamp = timeutil_timer_create (spec, &timer_callback_timestamp, state);

    // main loop
    while (!gblDone) {
        struct timeval timeout = {
            .tv_sec = 1,
            .tv_usec = 0,
        };
        lcmu_handle_timeout (state->lcminfo->lcm, &timeout);
    }


    // send message to turn off triggers, otherwise flash keeps going
    PvAttrEnumSet (state->camera->handle, "SyncOut1Mode", "Off");
    PvAttrEnumSet (state->camera->handle, "SyncOut2Mode", "Off");
    err = PvCommandRun (state->camera->handle, "AcquisitionStop");
    if (err != ePvErrSuccess) {
        PROSILICA_ERROR (err, "PvCommandRun(%s)", "AcquisitionStop");
    }

    // clear queue and unload camera API
    err = PvCaptureQueueClear (state->camera->handle);
    if (err != ePvErrSuccess)
        PROSILICA_ERROR (err, "PvCaptureQueueClear()");

    PvLinkRemove (state);
    PvUnInitialize ();


    // free resources
    printf ("\n%s: Goodbye\n", state->rootkey);
    timeutil_timer_destroy (&timer_pvattributes);
    timeutil_timer_destroy (&timer_timestamp);
    senlcm_prosilica_t_unsubscribe (state->lcminfo->lcm, sub);
    lcm_destroy (state->lcminfo->lcm);
    state_destroy (state);
}
