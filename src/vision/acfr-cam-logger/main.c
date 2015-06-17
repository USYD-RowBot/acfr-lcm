#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h> // needed for PRId64 macros

#include <glib.h>

#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/senlcm_uvc_osi_t.h"
#include "perls-lcmtypes/senlcm_os_compass_t.h"
#include "perls-lcmtypes/acfrlcm_auv_vis_rawlog_t.h"
#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/units.h"
#include "perls-common/unix.h"

#include "perls-vision/botimage.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define IMAGE_DESCRIPTION_NONE      0
#define IMAGE_DESCRIPTION_IVER      1
#define IMAGE_DESCRIPTION_SEGWAY    2
#define IMAGE_DESCRIPTION_QUADROTER 4


GMainLoop *_mainloop;

typedef struct iver iver_t;
struct iver
{
    int nextwp;
    double depth; // m
    double alt;   // m
    double r;     // deg
    double p;     // deg
    double h;     // deg
    double lat;   // deg
    double lon;   // deg
    double speed; // m/s
};

typedef struct cam_event cam_event_t;
struct cam_event
{
    char *channel;
    bot_core_image_t *image;
    int   mem_size;
    iver_t iver;
};

static inline void
cam_event_free (cam_event_t *ce)
{
    free (ce->channel);
    bot_core_image_t_destroy (ce->image);
    free (ce);
}

typedef struct channel_stats channel_stats_t;
struct channel_stats
{
    int64_t images_count;
    int64_t images_size;
    int64_t curr_event_utime;
    int64_t last_event_utime;
};

typedef struct cam_logger cam_logger_t;
struct cam_logger
{
    int64_t     utime0;
    char        *logpath;
    int64_t     max_write_queue_size;
    GAsyncQueue *write_queue;
    GThread     *write_thread;
    lcm_t       *lcm;
    bool        bayerfilt;

    // these members controlled by mutex
    GMutex *mutex;
    int32_t write_queue_length;
    int64_t write_queue_size;
    bool    write_thread_exit_flag;

    // these members control tif file compression format
    unsigned int compression;  // COMPRESSION_NONE; COMPRESSION_LZW; COMPRESSION_JPEG, COMPRESSION_DEFLATE
    unsigned int quality;      // 0-100

    // these members controlled by write thread
    GHashTable *channels_hash;
    char    channels_name[10*(LCM_MAX_CHANNEL_NAME_LENGTH+1+1)]; // e.g. FOO|BAR|GOO
    int64_t images_count;
    int64_t images_size;
    int64_t last_report_utime;
    int64_t last_report_count;

    // these members controlled by callback
    int64_t dropped_images_count;
    int64_t last_drop_report_utime;
    int64_t last_drop_report_count;

    // these members used for image description options
    int     image_description;
    iver_t  iver;
};

static void
print_stats (cam_logger_t *logger, int64_t now)
{
    if (now - logger->last_report_utime >= 900000)
    {
        g_mutex_lock (logger->mutex);
        printf ("Summary: t: %2"PRId64"s Buffer: %-2d ( %2"PRId64" MB ) Written: %-4"PRId64" ( %3"PRId64" MB ) Channels: %s\n",
                (now-logger->utime0)/1000000,
                logger->write_queue_length, logger->write_queue_size/1024/1024,
                logger->images_count, logger->images_size/1024/1024,
                logger->channels_name);
        logger->last_report_utime = now;
        logger->last_report_count = logger->images_count;
        g_mutex_unlock (logger->mutex);
    }
}


size_t
acfr_timestamp_strftime (char *s, size_t max, const char *format, struct timeval *tv)
{
    int tmalloc = 0;
    if (tv == NULL)
    {
        tmalloc = 1;
        tv = malloc (sizeof (struct timeval));
        timestamp_to_timeval (timestamp_now (), tv);
    }

    struct tm tm;
    localtime_r (&tv->tv_sec, &tm);


    const char *formatend = format + strlen (format);
    char format2[1024], tmp[1024];

    // handle %i arg if present
    char *istr = strstr (format, "%i");
    if (istr != NULL)
    {
        if (istr > format)
        {
            memset (tmp, '\0', sizeof (tmp));
            strncpy (tmp, format, istr - format);
            sprintf (format2, "%s%03ld%s", tmp, tv->tv_usec/1000, istr+2 < formatend ? istr+2 : "");
        }
        else
        {
            sprintf (format2, "%03ld%s", tv->tv_usec/1000, format+2);
        }
    }
    else
        strcpy (format2, format);

    if (tmalloc)
        free (tv);

    return strftime (s, max, format2, &tm);
}


size_t
acfr_strftime_filename (char *filename, size_t len, const char *format, int64_t utime)
{

    char format2[1024];


    strcpy (format2, format);

    struct timeval tv;
    timestamp_to_timeval (utime, &tv);

    return acfr_timestamp_strftime (filename, len, format2, &tv);
}


static void *
write_thread (void *user)
{
    cam_logger_t *logger = user;
    acfrlcm_auv_vis_rawlog_t vis_raw;

    while (1)
    {
        GTimeVal end_time;
        g_get_current_time (&end_time);
        g_time_val_add (&end_time, 250000);
        cam_event_t *ce = g_async_queue_timed_pop (logger->write_queue, &end_time);
        int64_t now = timestamp_now ();

        // Should the write thread exit?
        g_mutex_lock (logger->mutex);
        if (logger->write_thread_exit_flag)
        {
            g_mutex_unlock (logger->mutex);
            return NULL;
        }
        // nope. did we timeout?
        if (!ce)
        {
            g_mutex_unlock (logger->mutex);
            print_stats (logger, now);
            continue;
        }
        // ladies and gentlemen we have a cam event
        logger->write_queue_length--;
        logger->write_queue_size -= ce->mem_size;
        g_mutex_unlock (logger->mutex);

        // have we seen this channel before?
        channel_stats_t *channel_stats = g_hash_table_lookup (logger->channels_hash, ce->channel);
        if (!channel_stats)   // nope. create an entry for it
        {
            char channeldir[PATH_MAX];
            //snprintf (channeldir, sizeof channeldir, "%s/%s", logger->logpath, ce->channel);
            snprintf (channeldir, sizeof channeldir, "%s", logger->logpath);
            if (0 != unix_mkpath (channeldir, 0775))
            {
                fprintf (stderr, "error, unable to create directory: %s\n", channeldir);
                cam_event_free (ce);
                continue;
            }
            if (strlen (logger->channels_name))
                strncat (logger->channels_name, "|", sizeof logger->channels_name);
            strncat (logger->channels_name, ce->channel, sizeof logger->channels_name);
            channel_stats = g_malloc0 (sizeof (*channel_stats));
            g_hash_table_insert (logger->channels_hash, strdup (ce->channel), channel_stats);
        }

        // format image description
        char description[1024] = "";
        switch (logger->image_description)
        {
        case IMAGE_DESCRIPTION_IVER:
            snprintf (description, sizeof description,
                      "$IVER,nwp=%d,lat=%.6f,lon=%.6f,d=%.1f,a=%.1f,r=%.1f,p=%.1f,h=%.1f,s=%.2f",
                      ce->iver.nextwp, ce->iver.lat, ce->iver.lon, ce->iver.depth, ce->iver.alt,
                      ce->iver.r, ce->iver.p, ce->iver.h, ce->iver.speed);
            break;
        case IMAGE_DESCRIPTION_NONE:
            break;
        default:
            ERROR ("unknown image_description option = %d", logger->image_description);
        }

        if (logger->bayerfilt)
        {
            bot_core_image_t *bayer;
            vis_botimage_bayerfilt (&bayer, ce->image);
            bot_core_image_t_destroy (ce->image);
            ce->image = bayer;
        }

        // write image to disk
        char filename[PATH_MAX];
        char pathfilename[PATH_MAX];
        char acfr_format[24] = "PR_%Y%m%d_%H%M%S_%i_";

        //pick char from channel name
        // convention: last four characters of string indicate camera side and mode
        int channel_name_length = strlen(ce->channel);
        strcat(acfr_format,ce->channel+channel_name_length-4);
        acfr_strftime_filename (filename, sizeof filename, acfr_format, ce->image->utime);

        // get exposure time and publish LCM message
        vis_raw.utime = ce->image->utime;
        vis_raw.exp_time = *(int32_t *)(ce->image->metadata[0].value);
        vis_raw.image_name = malloc(strlen(filename)+5);
        strcpy(vis_raw.image_name,filename);
        strcat(vis_raw.image_name, ".tif\0");
        printf("***filename vis_raw %s, exp_time: %.2fms\n",vis_raw.image_name, vis_raw.exp_time/1000.);
        acfrlcm_auv_vis_rawlog_t_publish(logger->lcm,"ACFR_AUV_VIS_RAWLOG",&vis_raw);
        //add path to filename for logging
        snprintf(pathfilename,sizeof pathfilename, "%s/%s.tif",logger->logpath,filename);

//        if (0 != vis_botimage_write_tiff (ce->image, filename, ce->channel, description, logger->compression | logger->quality)) {
        if (0 != vis_botimage_write_tiff (ce->image, pathfilename, ce->channel, description, logger->compression | logger->quality))
        {
            static int64_t last_spew_utime = 0;
            if (now - last_spew_utime > 500000)
            {
                fprintf (stderr, "error writing: %s\n", filename);
                last_spew_utime = now;
            }
            cam_event_free (ce);
            continue;
        }

        // bookkeeping, cleanup
        channel_stats->images_count++;
        channel_stats->images_size += ce->mem_size;
        channel_stats->last_event_utime = channel_stats->curr_event_utime;
        channel_stats->curr_event_utime = now;
        logger->images_count++;
        logger->images_size += ce->mem_size;
        print_stats (logger, now);

        cam_event_free (ce);
    }
}

static void
bot_core_image_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                           const bot_core_image_t *image, void *user)
{
    cam_logger_t *logger = user;

    // check if backlog of unwritten images is too big.  If so, then
    // ignore this event
    int chanlen = strlen (channel);
    int64_t mem_size = sizeof (cam_event_t) + chanlen + 1 + rbuf->data_size;
    g_mutex_lock (logger->mutex);
    int64_t mem_required = mem_size + logger->write_queue_size;

    if (mem_required > logger->max_write_queue_size)
    {
        // can't write images fast enough.  drop image.
        g_mutex_unlock (logger->mutex);

        // maybe print an informational message to stdout
        int64_t now = timestamp_now ();
        logger->dropped_images_count++;
        int rc = logger->dropped_images_count - logger->last_drop_report_count;

        if (now - logger->last_drop_report_utime > 1000000 && rc > 0)
        {
            fprintf (stderr, "Can't write images fast enough.  Dropped %d image%s\n",
                     rc, rc==1 ? "":"s");
            logger->last_drop_report_utime = now;
            logger->last_drop_report_count = logger->dropped_images_count;
        }
        return;
    }
    else
    {
        logger->write_queue_length++;
        logger->write_queue_size = mem_required;
        g_mutex_unlock (logger->mutex);
    }

    // queue up the cam event for writing to disk by the write thread
    cam_event_t *ce = g_malloc (sizeof (*ce));
    ce->channel = strdup (channel);
    ce->image = bot_core_image_t_copy (image);
    ce->mem_size = mem_size;
    ce->iver = logger->iver;

    g_async_queue_push (logger->write_queue, ce);
}

static void
senlcm_uvc_osi_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                           const senlcm_uvc_osi_t *msg, void *user)
{
    cam_logger_t *logger = user;

    logger->iver.nextwp = msg->nextwp;
    logger->iver.lat = msg->latitude;
    logger->iver.lon = msg->longitude;
    logger->iver.alt = msg->altimeter;
    logger->iver.speed = msg->speed;
}

static void
senlcm_os_compass_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                              const senlcm_os_compass_t *msg, void *user)
{
    cam_logger_t *logger = user;

    logger->iver.r = msg->rph[0] * RTOD;
    logger->iver.p = msg->rph[1] * RTOD;
    logger->iver.h = msg->rph[2] * RTOD;
    logger->iver.depth = msg->depth;
}




int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt,
                            "Camera LCM logger logs bot_core_image_t events to disk as TIFF image files.");
    getopt_add_bool   (gopt, 'D',  "daemon",           0,               "Run as system daemon");
    getopt_add_string (gopt, 'c',  "channel",          "^PROSILICA_[A-Z]+$", "POSIX regular expression of channels to log");
    getopt_add_string (gopt, 'l',  "lcm-url",          "\0",            "Log messages on the specified LCM URL");
    getopt_add_int    (gopt, 'm',  "max-unwritten-mb", "100",           "Maximum size of in-memory image buffer (MB)");
    getopt_add_string (gopt, 'o',  "outdir",          "./",             "Output directory path");
    getopt_add_bool   (gopt, 'b',  "bayerfilt",        0,               "Enable bayer color interpolation");
    getopt_add_string (gopt, 'C',  "compression",      "none",          "TIFF file compression scheme {none,lzw,deflate,jpeg,jpeg:quality}");
    getopt_add_string (gopt, 'd',  "description",      "none",          "Write state description to TIFF header {none,iver,segway,quad}");
    getopt_add_help   (gopt, NULL);
    getopt_add_example (gopt,
                        "Match PROSLICA_C or PROSILICA_M, log to ~/foo/bar, color interpolate, and use 95%% JPEG compression\n"
                        "%s --channel ^PROSILICA_.$ --outdir ~/foo/bar/.  --bayerfilt --compression jpeg:95", argv[0]);


    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len !=0)
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help"))
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    // initialize GLib threading
    if (!g_thread_supported ())
        g_thread_init (NULL);

    cam_logger_t *logger = g_malloc0 (sizeof (*logger));


    // fire up LCM
    const char *lcmurl = NULL;
    if (getopt_has_flag (gopt, "lcm-url"))
        lcmurl = getopt_get_string (gopt, "lcm-url");
    logger->lcm = lcm_create (lcmurl);
    if (!logger->lcm)
    {
        ERROR ("lcm_create() failed");
        exit (EXIT_FAILURE);
    }


    BotParam *param = bot_param_new_from_server (logger->lcm, 1);

    logger->utime0 = timestamp_now ();
    char filename[PATH_MAX];
    const char *outdir = getopt_get_string (gopt, "outdir");
    if (outdir[strlen (outdir)-1] != '/' &&
            outdir[strlen (outdir)-1] != '.')
    {
        ERROR ("[%s] does not specify a fully qualified directory path", outdir);
        exit (EXIT_FAILURE);
    }
    else
    {
        timeutil_strftime (filename, sizeof filename, outdir,
                           timestamp_now ());
        logger->logpath = g_path_get_dirname (filename);
    }
    logger->max_write_queue_size = ((int64_t)getopt_get_int (gopt, "max-unwritten-mb")) * 1024 * 1024;
    logger->channels_hash = g_hash_table_new_full (&g_str_hash, &g_str_equal, &free, &free);
    const char *compression_str = getopt_get_string (gopt, "compression");
    unsigned int quality = 95;
    if (0 == strcasecmp (compression_str, "none"))
        logger->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_NONE;
    else if (0 == strcasecmp (compression_str, "lzw"))
        logger->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_LZW;
    else if (0 == strcasecmp (compression_str, "deflate"))
        logger->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_DEFLATE;
    else if (0 == strcasecmp (compression_str, "jpeg"))
        logger->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_JPEG | quality;
    else if (0 == strncasecmp (compression_str, "jpeg:", 5) &&
             1 == sscanf (compression_str, "jpeg:%u", &quality) &&
             0 < quality && quality <= 100)
        logger->compression = VIS_BOTIMAGE_TIFF_COMPRESSION_JPEG | quality;
    else
    {
        fprintf (stderr, "unrecognized compression format\n");
        exit (EXIT_FAILURE);
    }
    logger->bayerfilt = getopt_get_bool (gopt, "bayerfilt");

    // create write thread
    logger->write_thread_exit_flag = 0;
    logger->mutex = g_mutex_new ();
    logger->write_queue_length = 0;
    logger->write_queue_size = 0;
    logger->write_queue = g_async_queue_new ();
    logger->write_thread = g_thread_create (&write_thread, logger, TRUE, NULL);

    const char *description = getopt_get_string (gopt, "description");
    if (0==strcasecmp (description, "none"))
    {
        logger->image_description = IMAGE_DESCRIPTION_NONE;
    }
    else if (0==strcasecmp (description, "iver"))
    {
        logger->image_description = IMAGE_DESCRIPTION_IVER;

        char *osc_osi_channel = lcmu_channel_get_os_conduit (param, LCMU_CHANNEL_OS_CONDUIT_OSI);
        senlcm_uvc_osi_t_subscribe (logger->lcm, osc_osi_channel, &senlcm_uvc_osi_t_callback, logger);
        free (osc_osi_channel);

        char *os_compass_channel = botu_param_get_str_or_default (param, "sensors.os_compass.gsd.channel", ".*OS_COMPASS");
        senlcm_os_compass_t_subscribe (logger->lcm, os_compass_channel, &senlcm_os_compass_t_callback, logger);
        free (os_compass_channel);
    }
    else
    {
        fprintf (stderr, "unrecognized description \"%s\"\n", description);
        exit (EXIT_FAILURE);
    }

    // begin logging
    bot_core_image_t_subscription_t *sub =
        bot_core_image_t_subscribe (logger->lcm, getopt_get_string (gopt, "channel"), &bot_core_image_t_callback, logger);

    if (getopt_get_bool (gopt, "daemon"))
        daemon_fork ();

    _mainloop = g_main_loop_new (NULL, FALSE);
    bot_signal_pipe_glib_quit_on_kill (_mainloop);
    bot_glib_mainloop_attach_lcm (logger->lcm);

    // main loop
    g_main_loop_run (_mainloop);

    fprintf (stderr, "Logger exiting\n");
    bot_core_image_t_unsubscribe (logger->lcm, sub);
    int n = g_async_queue_length (logger->write_queue), n_last_report = 0;
    while (n > 0)
    {
        if (n!=n_last_report)
        {
            fprintf (stderr, "%d images remaining in the queue\n", n);
            n_last_report = n;
        }
        n = g_async_queue_length (logger->write_queue);
        timeutil_usleep (50000);
    }
    print_stats (logger, timestamp_now ()+2000000);
    fprintf (stderr, "Goodbye\n");

    // stop the write thread
    g_mutex_lock (logger->mutex);
    logger->write_thread_exit_flag = 1;
    g_mutex_unlock (logger->mutex);
    g_thread_join (logger->write_thread);
    g_mutex_free (logger->mutex);

    // cleanup.  This isn't strictly necessary, do it to be pendantic and so that
    // leak checkers don't complain
    bot_glib_mainloop_detach_lcm (logger->lcm);
    lcm_destroy (logger->lcm);
    g_async_queue_unref (logger->write_queue);
    free (logger->logpath);
    free (logger);
    getopt_destroy (gopt);

    exit (EXIT_SUCCESS);
}
