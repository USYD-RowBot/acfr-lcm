#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>
#include <inttypes.h> // needed for PRId64 macros

#include <glib.h>

#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/bot_core_image_sync_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/cache.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/glib_util.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"

#include "perls-vision/botimage.h"

#define DEBUG                  0
#define MAX_CAMERA_CHANNELS    20
#define MAX_CAMERA_CACHE_DEPTH 10

typedef struct camera camera_t;
struct camera {
    char    *channel;
    char    *channel_out;
    char    *campath;
    GAsyncQueue *prefetch_queue;
    GThread     *prefetch_thread;
    int32_t nfiles;

    // these members are controlled by mutex
    GMutex      *mutex;
    GList       *files;
    int32_t      cpos; // current position in glist
    int32_t      qpos; // queue position in glist
    int32_t      qlen;
    GHashTable  *cache;
    bool         done;
    bool ignore_tiff_tag;
};

typedef struct player player_t;
struct player {
    int64_t     utime0;
    char        *logpath;
    lcm_t       *lcm_sync;
    lcm_t       *lcm_pub;
    const char  *channel_out;

    GHashTable  *sync_channel_hash;
    int         ncameras; 
    camera_t    *cameras[MAX_CAMERA_CHANNELS];

    bool ignore_tiff_tag;
    
};


GMainLoop *_mainloop;


static void
bot_core_image_t_destroy_wrapper (gpointer data)
{
    bot_core_image_t_destroy (data);
}

static void
cache_value_destroy_func (void *value)
{
    bot_core_image_t_destroy (value);
}

static gint 
glist_sorter (gconstpointer a, gconstpointer b)
{
    const int64_t *ai = a, *bi = b;
    if (*ai > *bi)
        return 1;
    else if (*ai == *bi)
        return 0;
    else
        return -1;
} 

static void
glist_destroyer (gpointer data, gpointer user)
{
    free (data);
}


static void *
prefetch_thread (void *user)
{
    camera_t *camera = user;

    while (1) {
        GTimeVal end_time;
        g_get_current_time (&end_time);
        g_time_val_add (&end_time, 250000);
        int *ce = g_async_queue_timed_pop (camera->prefetch_queue, &end_time);

        // Should the prefetch thread exit?
        g_mutex_lock (camera->mutex);
        if (camera->done) {
            g_mutex_unlock (camera->mutex);
#if DEBUG
            printf ("%s: exiting\n", camera->channel);
#endif
            return NULL;
        }
        // nope. did we timeout?
        if (!ce) {
            g_mutex_unlock (camera->mutex);
            continue;
        }
        // we have a cam event
        free (ce);
        if (--camera->qlen > 0) {
            fprintf (stderr, "%s: unabled to keep up, try slowing down lcm-logplayer [%d]\n",
                     camera->channel, camera->qlen);
        }
        g_mutex_unlock (camera->mutex);
#if DEBUG
        int64_t utime = *((int64_t *)camera->files->data);
        printf ("%s: %"PRId64" prefetch event\n", camera->channel, utime);
#endif

        int n = 0;
        g_mutex_lock (camera->mutex);
        n = MAX_CAMERA_CACHE_DEPTH - (camera->qpos - camera->cpos);
        g_mutex_unlock (camera->mutex);
        if (n < 0) {
            fprintf (stderr, "TIME WARP n=%d!\n", n);
            n = 0;
        }
        for (int i=0; i<n; i++) {
            g_mutex_lock (camera->mutex);
            GList *next = g_list_next (camera->files);
            if (!next)
                next = g_list_first (camera->files); // rewind
            camera->files = next;
            g_mutex_unlock (camera->mutex);

            // queue up our next camera event and cache it
            bot_core_image_t *img = NULL;
            int64_t utime_next = *((int64_t *)next->data);
            char filename[PATH_MAX] = "";
            sprintf (filename, "%s/%"PRId64".tif", camera->campath, utime_next);
            if (0 == vis_botimage_read_tiff (&img, NULL, NULL, filename, camera->ignore_tiff_tag)) {
                g_mutex_lock (camera->mutex);
                g_hash_table_insert (camera->cache, gu_dup (&utime_next, sizeof utime_next), img);
                camera->qpos++;
                g_mutex_unlock (camera->mutex);
#if DEBUG               
                printf ("%s: %"PRId64" queued\n", camera->channel, utime_next);
#endif
            }
            else {
                ERROR ("%s: %"PRId64".tif error reading, unable to queue up image", camera->channel, utime_next);
                continue;
            }
        }
    }
}

camera_t *
camera_new (player_t *player, const char *data_channel)
{
    char campath[PATH_MAX];
    sprintf (campath, "%s/%s/", player->logpath, data_channel);

    // populate our camera struct
    camera_t *camera = g_malloc0 (sizeof (*camera));
    camera->channel = strdup (data_channel);
    camera->channel_out = player->channel_out ? strdup (player->channel_out) : strdup (data_channel);
    camera->campath = strdup (campath);
    camera->cache = g_hash_table_new_full (&gu_int64_hash, &gu_int64_equal, &free, &bot_core_image_t_destroy_wrapper);
    camera->prefetch_queue = g_async_queue_new_full (&g_free);
    camera->mutex = g_mutex_new ();
    camera->files = NULL;
    camera->done = 0;
    camera->ignore_tiff_tag = player->ignore_tiff_tag;

    if (player->channel_out)
        printf ("%s: output mapped to channel %s\n", camera->channel, camera->channel_out);

   // populate camera's list of available .tif files
   GDir *dir = g_dir_open (campath, 0, NULL);
    if (!dir) {
        ERROR ("unable to open dir [%s]", campath);
        goto on_error;
    }
    else {
        for (const char *file = g_dir_read_name (dir); file != NULL; file = g_dir_read_name (dir)) {
            int64_t utime;
            if (1 != sscanf (file, "%16"PRId64".tif", &utime)) {
                ERROR ("unexpected file [%s] in dir [%s]", file, camera->campath);
                continue;
            }
            else {
                camera->nfiles++;
                camera->files = g_list_prepend (camera->files, gu_dup (&utime, sizeof utime));
            }
        }
    }
    camera->files = g_list_sort (camera->files, &glist_sorter);
    g_dir_close (dir);

    camera->prefetch_thread = g_thread_create (&prefetch_thread, camera, TRUE, NULL);
    if (!camera->prefetch_thread) {
        ERROR ("unable to create camera_thread for [%s]", data_channel);
        goto on_error;
    }

    return camera;

  on_error:
    free (camera->channel);
    free (camera->channel_out);
    free (camera->campath);
    g_hash_table_unref (camera->cache);
    g_async_queue_unref (camera->prefetch_queue);
    g_list_foreach (camera->files, &glist_destroyer, NULL);
    g_list_free (camera->files);
    free (camera);
    return NULL;
}

static void
camera_free (camera_t *camera)
{
    free (camera->channel);
    free (camera->channel_out);
    free (camera->campath);
    g_hash_table_unref (camera->cache);
    g_async_queue_unref (camera->prefetch_queue);
    g_list_foreach (camera->files, &glist_destroyer, NULL);
    g_list_free (camera->files);
    free (camera);
}


static void
bot_core_image_sync_t_callback (const lcm_recv_buf_t *rbuf, const char *sync_channel,
                              const bot_core_image_sync_t *msg, void *user)
{
    player_t *player = user;

    camera_t *camera = g_hash_table_lookup (player->sync_channel_hash, sync_channel);
    if (!camera) {
        char data_channel[LCM_MAX_CHANNEL_NAME_LENGTH] = "";
        char *sync_ptr = strstr (sync_channel, ".SYNC");
        if (!sync_ptr) {
            ERROR ("unable to parse channel name from [%s]", sync_channel);
            return;
        }
        else
            memcpy (data_channel, sync_channel, sync_ptr - sync_channel);

        camera = camera_new (player, data_channel);
        if (!camera) {
            ERROR ("unable to create camera for [%s]", data_channel);
            return;
        }
        else {
            printf ("%s: adding channel\n", data_channel);
            g_hash_table_insert (player->sync_channel_hash, strdup (sync_channel), camera);
            player->cameras[player->ncameras++] = camera;
        }
    }

    // look for camera event in cache
    g_mutex_lock (camera->mutex);
    bot_core_image_t *img = g_hash_table_lookup (camera->cache, &msg->utime);
    g_mutex_unlock (camera->mutex);
    if (img) {
#if DEBUG
        printf ("%s: %"PRId64" in cache\n", camera->channel, msg->utime);
#endif
        // publish camera event
        bot_core_image_t_publish (player->lcm_pub, camera->channel_out, img);

        // pull it out of the cache
        g_mutex_lock (camera->mutex);
        g_hash_table_remove (camera->cache, &msg->utime);
        g_mutex_unlock (camera->mutex);

        // queue up the next event
        g_mutex_lock (camera->mutex);
        camera->cpos++;
        camera->qlen++;
        g_mutex_unlock (camera->mutex);
        int i = 1;
        g_async_queue_push (camera->prefetch_queue, gu_dup (&i, sizeof i));
    }
    else {
        // event not in cache, look for it on disk
        fprintf (stderr, "%s: %"PRId64" event not in cache, loading...\n", camera->channel, msg->utime);
        g_mutex_lock (camera->mutex);
        GList *event = g_list_find_custom (g_list_first (camera->files), &msg->utime, &glist_sorter);
        if (!event) {
            g_mutex_unlock (camera->mutex);
            fprintf (stderr, "%s: %"PRId64".tif not on disk!\n", camera->channel, msg->utime);
            return;
        }
        else {
            camera->files = event;
            g_mutex_unlock (camera->mutex);
            
            // requeue the cache
            g_mutex_lock (camera->mutex);
            g_hash_table_remove_all (camera->cache);
            camera->cpos = 0;
            camera->qpos = 0;
            camera->qlen++;
            g_mutex_unlock (camera->mutex);
            int i = 1;
            g_async_queue_push (camera->prefetch_queue, gu_dup (&i, sizeof i));


            char filename[PATH_MAX];
            sprintf (filename, "%s/%"PRId64".tif", camera->campath, msg->utime);
            if (0 == vis_botimage_read_tiff (&img, NULL, NULL, filename, player->ignore_tiff_tag)) {
                // publish camera event
                bot_core_image_t_publish (player->lcm_pub, camera->channel_out, img);
                bot_core_image_t_destroy (img);            }
            else {
                ERROR ("%s: %"PRId64".tif error reading", camera->channel, msg->utime);
                return;
            }
        }
    }
}

#define DESCRIPTION \
    "Camera LCM player synchronously publishes bot_core_image_t's in step with bot_core_image_sync_t events\n" \
    "from recorded perls-vis-logger TIFFs."

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, DESCRIPTION);
    getopt_add_bool   (gopt, 'D',  "daemon",           0,                      "Run as system daemon");
    getopt_add_string (gopt, 'c',  "channel",          "^PROSILICA_[A-Z]+\\.SYNC$", "POSIX regular expression of image sync channels to play");
    getopt_add_string (gopt, 'o',  "channel-out",      "",                     "Play image events on the specified LCM channel");
    getopt_add_string (gopt, 'l',  "lcm-url",          "",                     "Play image events on the specified LCM URL");
    getopt_add_bool   (gopt, 'i',  "ignore",           0,                      "Ignore bot-specific tiff tags checking");
    getopt_add_help   (gopt, NULL);

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len !=1) {
        getopt_do_usage (gopt, "ROOTDIR");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt, "ROOTDIR");
        exit (EXIT_SUCCESS);
    }

    // initialize GLib threading
    if (!g_thread_supported ()) 
        g_thread_init (NULL);

    // parse options
    player_t *player = g_malloc0 (sizeof (*player));
    player->utime0 = timestamp_now ();
    player->sync_channel_hash = g_hash_table_new_full (&g_str_hash, &g_str_equal, &free, NULL);
    
    const char *dirpath = g_ptr_array_index (gopt->extraargs, 0);
    if (dirpath[strlen (dirpath)-1] != '/' &&
        dirpath[strlen (dirpath)-1] != '.') {
        ERROR ("[%s] does not specify a fully qualified directory path", dirpath);
        exit (EXIT_FAILURE);
    }
    else
        player->logpath = g_path_get_dirname (dirpath);
    
    const char *channel_out = getopt_get_string (gopt, "channel-out");
    if (getopt_has_flag (gopt, "channel-out"))
        player->channel_out = channel_out;
    else
        player->channel_out = NULL;

    // ignore tiff tags
    player->ignore_tiff_tag = getopt_get_bool (gopt, "ignore");

    // begin playback
    player->lcm_sync = lcm_create (NULL);
    if (!player->lcm_sync) {
        ERROR ("lcm_create() failed");
        exit (EXIT_FAILURE);
    }
    const char *lcmurl = NULL;
    if (getopt_has_flag (gopt, "lcm-url"))
        lcmurl = getopt_get_string (gopt, "lcm-url");
    player->lcm_pub = lcm_create (lcmurl);
    if (!player->lcm_pub) {
        ERROR ("lcm_create() failed");
        exit (EXIT_FAILURE);
    }
    bot_core_image_sync_t_subscription_t *sub = 
        bot_core_image_sync_t_subscribe (player->lcm_sync, getopt_get_string (gopt, "channel"), &bot_core_image_sync_t_callback, player);

    if (getopt_get_bool (gopt, "daemon"))
        daemon_fork ();

    _mainloop = g_main_loop_new (NULL, FALSE);
    bot_signal_pipe_glib_quit_on_kill (_mainloop);
    bot_glib_mainloop_attach_lcm (player->lcm_sync);

    // main loop
    g_main_loop_run (_mainloop);

    fprintf (stderr, "Player exiting\n");
    for (int i=0; i<player->ncameras; i++) {
        camera_t *camera = player->cameras[i];
        g_mutex_lock (camera->mutex);
        camera->done = 1;
        g_mutex_unlock (camera->mutex);
        g_thread_join (camera->prefetch_thread);
        camera_free (camera);
    }
    fprintf (stderr, "Goodbye\n");

    // cleanup.  This isn't strictly necessary, do it to be pendantic and so that
    // leak checkers don't complain
    bot_core_image_sync_t_unsubscribe (player->lcm_sync, sub);
    bot_glib_mainloop_detach_lcm (player->lcm_sync);
    lcm_destroy (player->lcm_sync);
    lcm_destroy (player->lcm_pub);
    g_hash_table_destroy (player->sync_channel_hash);
    free (player->logpath);
    free (player);
    getopt_destroy (gopt);

    exit (EXIT_SUCCESS);
}
