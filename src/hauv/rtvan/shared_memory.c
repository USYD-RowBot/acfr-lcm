#include <stdio.h>
#include <stdlib.h>

// external linking req'd
#include <glib.h>
#include <opencv/cv.h>

#include "perls-common/error.h"
#include "perls-common/unix.h"
#include "perls-common/bot_util.h"

#include "perls-vision/distortion.h"

#include "shared_memory.h"

#define SHM_IMGCACHE_MAX 100

#define DTOR UNITS_DEGREE_TO_RADIAN
#define RTOD UNITS_RADIAN_TO_DEGREE

static void
imgcache_value_destroy (void *value)
{
    IplImage *img = value;
    cvReleaseImage (&img);
}

static void *
imgcache_value_copy (const void *value)
{
    const IplImage *img = value;
    return cvCloneImage (img);
}

static void
camera_params (camera_params_t *camera, BotParam *param, const char *cfgkey)
{
    camera->calib = vis_calib_load_config (param, cfgkey);
    camera->map = vis_undistort_map (&camera->calib);
    camera->mask = vis_undistort_mask (&camera->calib);

    // enlarged mask for siftgpu
    int pad = 25;
    IplConvKernel *strel = cvCreateStructuringElementEx (pad, pad, (pad-1)/2, (pad-1)/2, CV_SHAPE_ELLIPSE, NULL);
    camera->mask_siftgpu = cvCloneImage (camera->mask);
    cvErode (camera->mask, camera->mask_siftgpu, strel, 1);
    cvReleaseStructuringElement (&strel);
}

static void
load_van_options (perllcm_van_options_t *van_opts, BotParam *param)
{
    van_opts->vis_plot_features = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_plot_features", (bool *)&van_opts->vis_plot_features);

    van_opts->vis_plot_scene_prior = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_plot_scene_prior", (bool *)&van_opts->vis_plot_scene_prior);

    van_opts->vis_plot_put_corr = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_plot_put_corr", (bool *)&van_opts->vis_plot_put_corr);

    van_opts->vis_plot_search_ellipses = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_plot_search_ellipses", (bool *)&van_opts->vis_plot_search_ellipses);

    van_opts->vis_plot_in_and_out = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_plot_in_and_out", (bool *)&van_opts->vis_plot_in_and_out);

    van_opts->vis_plot_inliers = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_plot_inliers", (bool *)&van_opts->vis_plot_inliers);

    van_opts->vis_plot_relpose = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_plot_relpose", (bool *)&van_opts->vis_plot_relpose);

    van_opts->vis_plot_relpose_3pts = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_plot_relpose_3pts", (bool *)&van_opts->vis_plot_relpose_3pts);

    van_opts->vis_plot_3pts = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_plot_3pts", (bool *)&van_opts->vis_plot_3pts);

    van_opts->vis_plot_summary = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_plot_summary", (bool *)&van_opts->vis_plot_summary);

    van_opts->vis_plot_waitkey = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_plot_waitkey", (bool *)&van_opts->vis_plot_waitkey);

    van_opts->manual_corr = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_manual_corr", (bool *)&van_opts->manual_corr);

    van_opts->mdist_plink = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_mdist_plink", (bool *)&van_opts->mdist_plink);

    van_opts->n_plinks = 3;
    bot_param_get_int (param, "rtvan.link_thread.n_plinks", &van_opts->n_plinks);

    van_opts->vis_use_saliency = 0;
    botu_param_get_boolean_to_bool (param, "rtvan.plot_thread.vis_use_saliency", (bool *)&van_opts->vis_use_saliency);
}

shm_t *
shared_memory_init (int argc, char *argv[])
{
    // allocate shared memory object
    shm = calloc (1, sizeof (*shm));

    g_static_mutex_init (&shm->mutex);

    // img_warp list
    // ---------------------------------------------- //
    shm->imgcache = cache_new (SHM_IMGCACHE_MAX, &imgcache_value_copy, &imgcache_value_destroy);

    // lcm
    // ---------------------------------------------- //
    shm->lcm = lcm_create (NULL);
    if (!shm->lcm) {
        ERROR ("lcm_create() failed!");
        exit (EXIT_FAILURE);
    }

    // parse our config file
    // ---------------------------------------------- //
    shm->param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (! shm->param) {
      ERROR ("Could not get config parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
      exit(EXIT_FAILURE);
    }

    shm->cameraUw_rootkey   = bot_param_get_str_or_fail (shm->param, "rtvan.cameraUw");
    shm->cameraPeri_rootkey = bot_param_get_str_or_fail (shm->param, "rtvan.cameraPeri");

    char cfgkey[256];
    snprintf (cfgkey, sizeof cfgkey, "%s.channel", shm->cameraUw_rootkey);
    shm->bot_core_image_t_channelUw = bot_param_get_str_or_fail (shm->param, cfgkey);
    snprintf (cfgkey, sizeof cfgkey, "%s.channel", shm->cameraPeri_rootkey);
    shm->bot_core_image_t_channelPeri = bot_param_get_str_or_fail (shm->param, cfgkey);

    char channel[LCM_MAX_CHANNEL_NAME_LENGTH];
    snprintf (channel, sizeof channel, "%s.SYNC", shm->bot_core_image_t_channelUw);
    shm->bot_core_image_sync_t_channelUw = strdup (channel);
    snprintf (channel, sizeof channel, "%s.SYNC", shm->bot_core_image_t_channelPeri);
    shm->bot_core_image_sync_t_channelPeri = strdup (channel);

    shm->logdir = botu_param_get_str_or_default (shm->param, "rtvan.logdir", "./");
    unix_mkpath (shm->logdir, 0775);

    // camera calib water
    snprintf (cfgkey, sizeof cfgkey, "%s.%s", shm->cameraUw_rootkey, "calib_water");
    camera_params (&shm->waterUw, shm->param, cfgkey);
    snprintf (cfgkey, sizeof cfgkey, "%s.%s", shm->cameraPeri_rootkey, "calib_water");
    camera_params (&shm->waterPeri, shm->param, cfgkey);

    // camera calib air
    snprintf (cfgkey, sizeof cfgkey, "%s.%s", shm->cameraUw_rootkey, "calib_air");
    camera_params (&shm->airUw, shm->param, cfgkey);
    snprintf (cfgkey, sizeof cfgkey, "%s.%s", shm->cameraPeri_rootkey, "calib_air");
    camera_params (&shm->airPeri, shm->param, cfgkey);

    // options related to plot and link proposal
    load_van_options (&shm->van_opts, shm->param);

    shm->active = false;
    shm->imgstep = 1;
    bot_param_get_int (shm->param, "rtvan.imgstep", &shm->imgstep);

    // parse our options
    // ---------------------------------------------- //
    shm->gopt = getopt_create ();
    getopt_parse (shm->gopt, argc, argv, 1);

    return shm;
}

static void
glist_destroyer (gpointer data, gpointer user)
{
    free (data);
}

void
shared_memory_free (shm_t *shm)
{
    // free memory in reverse order of allocation
    getopt_destroy (shm->gopt);
    cache_destroy (shm->imgcache);
    
    /* clean up underwater camera parameters */
    vis_cvu_map_free (shm->waterUw.map);
    cvReleaseImage (&shm->waterUw.mask);
    cvReleaseImage (&shm->waterUw.mask_siftgpu);
    vis_cvu_map_free (shm->airUw.map);
    cvReleaseImage (&shm->airUw.mask);
    cvReleaseImage (&shm->airUw.mask_siftgpu);

    /* clean up periscope camera parameters */
    vis_cvu_map_free (shm->waterPeri.map);
    cvReleaseImage (&shm->waterPeri.mask);
    cvReleaseImage (&shm->waterPeri.mask_siftgpu);
    vis_cvu_map_free (shm->airPeri.map);
    cvReleaseImage (&shm->airPeri.mask);
    cvReleaseImage (&shm->airPeri.mask_siftgpu);

    g_slist_foreach (shm->utimelist, &glist_destroyer, NULL);

    bot_param_destroy (shm->param);
    lcm_destroy (shm->lcm);
    free (shm);
}
