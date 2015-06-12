#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <inttypes.h> // needed for PRId64 macros
#include <sys/types.h>

// external linking req'd
#include <glib.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "perls-common/error.h"
#include "perls-common/timestamp.h"
#include "perls-common/lcm_util.h"

#include "perls-math/gsl_util.h"

#include "perls-vision/botimage.h"
#include "perls-vision/clahs.h"
#include "perls-vision/feature.h"

#include <lcm/lcm.h>
#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/perllcm_pose3d_t.h"
#include "perls-lcmtypes/perllcm_van_feature_collection_t.h"
#include "perls-lcmtypes/perllcm_rdi_bathy_t.h"

#include "shared_memory.h"
#include "van_util.h"
#include "feature_thread.h"

#define printf(format, ...)                                     \
    printf ("%-12s " format, "[feature]", ## __VA_ARGS__)

#define POOL_THREADS_MAX          10
#define FEATURE_POINTS_MAX        1000
#define FEATURE_POINTS_MAX_WORDS  1000

#define BATHY_TIME_STEPS    200
#define BATHY_BEAMS         4
#define BATHY_POINTS_MAX    (BATHY_BEAMS*BATHY_TIME_STEPS)

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

typedef struct thread_data thread_data_t;
struct thread_data
{
    GThreadPool     *pool;

    perllcm_pose3d_t x_lcUw;    // current underwater camera pose (can be used for invariant feature extraction)
    perllcm_pose3d_t x_lcPeri;  // current periscope camera pose (can be used for invariant feature extraction)
    int              ftypes;    // bitmask of feature types to extract

    gsl_matrix      *bathy_l;   // [4 x BATHY_POINTS_MAX] array in local-level
    size_t           bathy_i;   // current column index of bathy_l array (modulo number of cols)


    // harris feature patch
    gsl_matrix      *featpatch_sampler;

    // process every Nth frame
    size_t           imgcounter;

    // siftgpu server information
    char *siftgpu_server;

    // surfgpu server information
    char *surfgpu_server;

    // use surfgpu server?
    int use_surfgpu;
};

typedef struct pool_data pool_data_t;
struct pool_data
{
    int64_t utime;
    IplImage *img_gray;
    perllcm_pose3d_t x_lc;
    gsl_matrix *bathy_l;
    camera_params_t *params_water, *params_air;
    vis_cvu_map_t *map;
    IplImage *mask, *mask_siftgpu;
    const char *bot_core_image_t_channel;
};

void
pooldata_free (pool_data_t *pdata)
{
    if (pdata->img_gray)
        cvReleaseImage (&pdata->img_gray);
    if (pdata->bathy_l)
        gsl_matrix_free (pdata->bathy_l);
    free (pdata);
}

static void
perllcm_pose3d_t_uw_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                              const perllcm_pose3d_t *msg, void *user)
{
    thread_data_t *tdata = user;
    tdata->x_lcUw = *msg;
}

static void
perllcm_pose3d_t_peri_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                const perllcm_pose3d_t *msg, void *user)
{
    thread_data_t *tdata = user;
    tdata->x_lcPeri = *msg;
}

static void
perllcm_rdi_bathy_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                              const perllcm_rdi_bathy_t *msg, void *user)
{
    thread_data_t *tdata = user;

    for (size_t b=0; b<4; b++, tdata->bathy_i++)
    {
        const double r = msg->range[b];
        const double x = msg->xyz[b][0];
        const double y = msg->xyz[b][1];
        const double z = msg->xyz[b][2];
        if (r > PERLLCM_RDI_BATHY_T_RANGE_SENTINAL)
        {
            GSLU_VECTOR_VIEW (X, 4, {x, y, z, b+1});

            tdata->bathy_i %= tdata->bathy_l->size2;
            gsl_matrix_set_col (tdata->bathy_l, tdata->bathy_i, &X.vector);
        }
    }
}


perllcm_van_feature_t *
_publish_surf_words (const IplImage *img, int64_t utime, void *user)
{

    if (img == NULL)
        return NULL;

    perllcm_van_feature_t *f = NULL;

    //IplImage *img_warp = cvCreateImage (cvGetSize (img_8bit), IPL_DEPTH_8U, 1);
    //cvSmooth(img_8bit, img_warp, CV_GAUSSIAN, 55, 0, 0, 0);
    //cache_push (shm->imgcache, pdata->utime, cvCloneImage (img_warp));

    thread_data_t *tdata = user;

    if (! tdata->use_surfgpu)
    {
        CvSURFParams params = cvSURFParams (500, 1);
        params.extended = 0; // or 1 (64 vec or 128 vec)
        params.hessianThreshold = 1000; // 300~500
        params.nOctaves = 3;
        params.nOctaveLayers = 4;
        f = vis_feature_cvsurf (img, NULL, params, FEATURE_POINTS_MAX_WORDS);
    }
    else
        f = vis_feature_surfgpu (img, NULL, tdata->surfgpu_server, -1, FEATURE_POINTS_MAX_WORDS);

    if (f)
        f->utime = utime;
    else
        ERROR ("vis_feature_cvsurf(): returned NULL");

    return f;
}

static void
bot_core_image_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                           const bot_core_image_t *msg, void *user)
{
    thread_data_t *tdata = user;

    if (!shm->active)
        return;

    // process only every nth frame
    if ((tdata->imgcounter++ % shm->imgstep))
    {
        printf ("bot_core_image_t event %"PRId64" - skipping feat ext\n", msg->utime);
        //printf ("imgstep=%d\n", shm->imgstep);
        return;
    }

    // drop if number of concurrent pool threads is unable to keep up
    guint ntasks = g_thread_pool_unprocessed (tdata->pool);
    if (ntasks >= g_thread_pool_get_max_threads (tdata->pool))
    {
        printf ("Warning: number of unprocessed tasks (%u) exceeds pool (%u)\n",
                ntasks, g_thread_pool_get_max_threads (tdata->pool));
        printf ("bot_core_image_t event %"PRId64" - dropping\n", msg->utime);
        return;
    }

    // stuff pool thread data and launch
    printf ("bot_core_image_t event %"PRId64" - processing\n", msg->utime);
    pool_data_t *pdata = malloc (sizeof (*pdata));
    pdata->utime = msg->utime;
    pdata->bathy_l = gslu_matrix_clone (tdata->bathy_l);
    pdata->img_gray = vis_botimage_to_iplimage_convert (msg, VIS_BOT2CVGRAY);

    if (strcmp (channel, shm->bot_core_image_t_channelUw) == 0)
    {
        pdata->params_water = &(shm->waterUw);
        pdata->params_air = &(shm->airUw);
        pdata->bot_core_image_t_channel = shm->bot_core_image_t_channelUw;
        pdata->x_lc = tdata->x_lcUw;
    }
    else if (strcmp (channel, shm->bot_core_image_t_channelPeri) == 0)
    {
        pdata->params_water = &(shm->waterPeri);
        pdata->params_air = &(shm->airPeri);
        pdata->bot_core_image_t_channel = shm->bot_core_image_t_channelPeri;
        pdata->x_lc = tdata->x_lcPeri;
    }
    else
    {
        ERROR ("Unknown bot_core_image_t channel");
        exit (1);
    }

    g_thread_pool_push (tdata->pool, pdata, NULL);
}

static void
feature_pool_thread (gpointer pooldata, gpointer user)
{
    pool_data_t *pdata = pooldata;
    thread_data_t *tdata = user;

    // select appropriate camera calibration...
    // -------------------------------------------------------------- //
    perllcm_van_calib_t *calib = NULL;
    vis_cvu_map_t *map = NULL;
    IplImage *mask = NULL, *mask_siftgpu = NULL;
    double depth = pdata->x_lc.mu[2];

    if (depth > 0.25)
    {
        printf ("calibration=WATER, depth=%f m\n", depth);
        calib = &(pdata->params_water->calib);
        map   = pdata->params_water->map;
        mask  = pdata->params_water->mask;
        mask_siftgpu = pdata->params_water->mask_siftgpu;
    }
    else
    {
        printf ("calibration=AIR,   depth=%f m\n", depth);
        calib = &(pdata->params_air->calib);
        map   = pdata->params_air->map;
        mask  = pdata->params_air->mask;
        mask_siftgpu = pdata->params_air->mask_siftgpu;
    }

    vis_calib_view_gsl_t calib_gsl = vis_calib_view_gsl (calib);

    // copy raw 8 bit before clahs for vocab generation
    // -------------------------------------------------------------- //
    IplImage *img_8bit_no_clahs = NULL;
    if (shm->van_opts.vis_use_saliency &&
            strcmp(pdata->bot_core_image_t_channel, shm->bot_core_image_t_channelUw) == 0)
    {
        if (pdata->img_gray->depth > 8)
        {
            img_8bit_no_clahs = cvCreateImage (cvGetSize (pdata->img_gray), IPL_DEPTH_8U, 1);
            cvConvertImage (pdata->img_gray, img_8bit_no_clahs, 0);
        }
        else
            img_8bit_no_clahs = cvCloneImage (pdata->img_gray);
    }

    // adaptive histogram equalization...
    // -------------------------------------------------------------- //
    vis_clahs_opts_t opts =
    {
        .tiles = {8, 10},
        .cliplimit = 0.0075,
        .bins = pdata->img_gray->depth > 8 ? 1024 : 256,
        .range = {0, 0},
        .dist = VIS_CLAHS_DIST_RAYLEIGH,
        .alpha = 0.4,
    };
    if (vis_clahs (pdata->img_gray->imageData, pdata->img_gray->width,
                   pdata->img_gray->height, pdata->img_gray->depth, &opts) < 0)
    {
        ERROR ("vis_clahs() failed");
        pooldata_free (pdata);
        return;
    }

    // converting to 8 bit...
    // -------------------------------------------------------------- //
    IplImage *img_8bit;
    if (pdata->img_gray->depth > 8)
    {
        img_8bit = cvCreateImage (cvGetSize (pdata->img_gray), IPL_DEPTH_8U, 1);
        cvConvertImage (pdata->img_gray, img_8bit, 0);
    }
    else
        img_8bit = pdata->img_gray;

    IplImage *img_saliency = NULL;
    if (shm->van_opts.vis_use_saliency &&
            strcmp(pdata->bot_core_image_t_channel, shm->bot_core_image_t_channelUw) == 0)
    {
        img_saliency = cvCreateImage (cvGetSize (pdata->img_gray), IPL_DEPTH_8U, 1);
        cvSmooth(img_8bit, img_saliency, CV_GAUSSIAN, 33, 0, 0, 0);
    }

    // undistorting image...
    // -------------------------------------------------------------- //
    IplImage *img_warp = cvCreateImage (cvGetSize (img_8bit), IPL_DEPTH_8U, 1);
    vis_cvu_warp_image (img_8bit, img_warp, map);
    cache_push (shm->imgcache, pdata->utime, cvCloneImage (img_warp));

    // extracting features...
    // -------------------------------------------------------------- //
    perllcm_van_feature_collection_t *fc = vis_feature_collection_alloc (pdata->utime, pdata->bot_core_image_t_channel);

    bool feature_found = false;
    if (tdata->ftypes & PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF)
    {
        int64_t t0 = timestamp_now ();
        CvSURFParams params = cvSURFParams (500, 1);

        perllcm_van_feature_t *f = vis_feature_cvsurf (img_warp, mask, params, FEATURE_POINTS_MAX);
        int64_t dt = timestamp_now () - t0;
        if (f)
        {
            vis_feature_collection_add (fc, f);
            printf ("cvsurf\t\tnpts=%4d\tdt=%"PRId64"\n", f->npts, dt);
            if (f->npts > 0) feature_found = true;
        }
        else
            ERROR ("vis_feature_cvsurf(): returned NULL");
    }

    if (tdata->ftypes & PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVHARRIS)
    {
        vis_feature_harris_params_t harris_params =
        {
            .qualityLevel = 0.01,   // 0.01 of best feature
            .minDistance = 10.0,     // euclidean distance in pixel
            .blockSize = 3,         // average block
            .k = 0.04,
        };
        int64_t t0 = timestamp_now ();
        perllcm_van_feature_t *f = vis_feature_cvharris (img_warp, mask, &harris_params, tdata->featpatch_sampler,
                                   pdata->x_lc, &calib_gsl.K.matrix, FEATURE_POINTS_MAX);
        int64_t dt = timestamp_now () - t0;
        if (f)
        {
            vis_feature_collection_add (fc, f);
            printf ("cvharris\t\tnpts=%4d\tdt=%"PRId64"\n", f->npts, dt);
            if (f->npts > 0) feature_found = true;
        }
        else
            ERROR ("vis_feature_cvharris(): returned NULL");
    }

    if (tdata->ftypes & PERLLCM_VAN_FEATURE_T_ATTRTYPE_SIFTGPU)
    {
        int64_t t0 = timestamp_now ();
        perllcm_van_feature_t *f = vis_feature_siftgpu (img_warp, mask_siftgpu, tdata->siftgpu_server, -1, FEATURE_POINTS_MAX);
        int64_t dt = timestamp_now () - t0;
        if (f)
        {
            vis_feature_collection_add (fc, f);
            printf ("siftgpu\t\tnpts=%4d\tdt=%"PRId64"\n", f->npts, dt);
            if (f->npts > 0) feature_found = true;
        }
        else
            ERROR ("vis_feature_siftgpu(): returned NULL");
    }

    if (fc->ntypes > 0 && feature_found)
    {
        // add our calibration
        // -------------------------------------------------------------- //
        fc->calib = *calib;

        // compute scene depth prior from bathymetry
        // -------------------------------------------------------------- //
        int64_t t0 = timestamp_now ();
        gsl_vector_view x_lc = gsl_vector_view_array (pdata->x_lc.mu, 6);
        gsl_matrix_view XYZ_l = gsl_matrix_submatrix (pdata->bathy_l, 0, 0, 3, pdata->bathy_l->size2);
        gsl_vector_view bid = gsl_matrix_row (pdata->bathy_l, 3);
        vis_feature_collection_scene_prior (fc, &XYZ_l.matrix, &bid.vector, &x_lc.vector,  calib);
        int64_t dt = timestamp_now () - t0;
        printf ("scene_prior\tnpts=%4d\tdt=%"PRId64"\n", fc->scene_prior.npts, dt);

        if (fc->scene_prior.npts == 0)
        {
            printf ("************************************************************\n");
            printf ("Warning: No DVL points project into bbox of image %"PRId64"!\n", pdata->utime);
            printf ("************************************************************\n");
        }


        // publish feature collection
        // -------------------------------------------------------------- //
        perllcm_van_feature_collection_t_publish (shm->lcm, VAN_FEATURE_COLLECTION_CHANNEL, fc);

        // save features
        // -------------------------------------------------------------- //
        char filename[PATH_MAX];
        snprintf (filename, sizeof filename, "%s/%"PRId64".feat", shm->logdir, pdata->utime);
        if (LCMU_FWRITE (filename, fc, perllcm_van_feature_collection_t) < 0)
            ERROR ("couldn't write %s to disk!", filename);
    }
    else
    {
        printf ("Warning: no features detected in image %"PRId64"\n", pdata->utime);
    }

    // publish SURF keys for saliency (only for underwater camera)
    // -------------------------------------------------------------- //
    if (shm->van_opts.vis_use_saliency &&
            strcmp(pdata->bot_core_image_t_channel, shm->bot_core_image_t_channelUw) == 0)
    {
        //perllcm_van_feature_t *words = _publish_surf_words (img_8bit_wo_clahs, pdata->utime);
        perllcm_van_feature_t *words = _publish_surf_words (img_saliency, pdata->utime, tdata);
        if (words != NULL)
        {
            perllcm_van_feature_t_publish (shm->lcm, VAN_WORDS_CHANNEL, words);
            perllcm_van_feature_t_destroy (words);
        }
    }

    // save unwarped image...
    // -------------------------------------------------------------- //
    vis_cvu_iplimg_save (shm->logdir, pdata->utime, img_warp);

    // clean up
    // -------------------------------------------------------------- //
    perllcm_van_feature_collection_t_destroy (fc);
    if (img_8bit != pdata->img_gray)
        cvReleaseImage (&img_8bit);
    if (img_warp)          cvReleaseImage (&img_warp);
    if (img_8bit_no_clahs) cvReleaseImage (&img_8bit_no_clahs);
    if (img_saliency)      cvReleaseImage (&img_saliency);

    pooldata_free (pdata);
}

gpointer
feature_thread (gpointer user)
{
    printf ("Spawning\n");

    // feature_thread state
    thread_data_t *tdata = calloc (1, sizeof (*tdata));
    tdata->bathy_l = gsl_matrix_alloc (4, BATHY_POINTS_MAX);
    gsl_matrix_set_all (tdata->bathy_l, GSL_NEGINF);

    // config file
    int n_ftypes = bot_param_get_array_len (shm->param, "rtvan.feature_thread.ftypes");
    if (n_ftypes > 0)
    {
        char **ftypes = bot_param_get_str_array_alloc (shm->param, "rtvan.feature_thread.ftypes");
        for (size_t n=0; n<n_ftypes; n++)
        {
            if (0==strcasecmp (ftypes[n], "cvsurf"))
                tdata->ftypes |= PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF;
            else if (0==strcasecmp (ftypes[n], "harris"))
                tdata->ftypes |= PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVHARRIS;
            else if (0==strcasecmp (ftypes[n], "siftgpu"))
                tdata->ftypes |= PERLLCM_VAN_FEATURE_T_ATTRTYPE_SIFTGPU;
        }
    }
    else
    {
        ERROR ("rtvan.feature_thread.ftypes is unspecified in config file");
        exit (EXIT_FAILURE);
    }

    // patch parameters (circular patch around a feature point as a descriptor)
    size_t w = 5;
    bot_param_get_int (shm->param, "rtvan.feature_thread.feat_patch_w", (int*)&w);

    if (0!=bot_param_get_str (shm->param, "rtvan.feature_thread.siftgpu_server", &tdata->siftgpu_server))
        tdata->siftgpu_server = NULL;

    if (0!=bot_param_get_str (shm->param, "rtvan.feature_thread.surfgpu_server", &tdata->surfgpu_server))
        tdata->surfgpu_server = NULL;

    if (0!=bot_param_get_int (shm->param, "rtvan.feature_thread.use_surfgpu", &tdata->use_surfgpu))
        tdata->use_surfgpu = 0;

    tdata->featpatch_sampler = vis_feature_patch_sampler_alloc (w);

    // image process step
    tdata->imgcounter = 0;

    // helper thread pool
    tdata->pool = g_thread_pool_new (feature_pool_thread, tdata, POOL_THREADS_MAX, 1, NULL);

    // lcm subscriptions
    bot_core_image_t_subscription_t *bot_core_image_t_subUw =
        bot_core_image_t_subscribe (shm->lcm, shm->bot_core_image_t_channelUw,
                                    &bot_core_image_t_callback, tdata);
    bot_core_image_t_subscription_t *bot_core_image_t_subPeri =
        bot_core_image_t_subscribe (shm->lcm, shm->bot_core_image_t_channelPeri,
                                    &bot_core_image_t_callback, tdata);

    perllcm_pose3d_t_subscription_t *perllcm_pose3d_t_subUw =
        perllcm_pose3d_t_subscribe (shm->lcm, VAN_CAMERA_POSE_UW_CHANNEL,
                                    &perllcm_pose3d_t_uw_callback, tdata);
    perllcm_pose3d_t_subscription_t *perllcm_pose3d_t_subPeri =
        perllcm_pose3d_t_subscribe (shm->lcm, VAN_CAMERA_POSE_PERI_CHANNEL,
                                    &perllcm_pose3d_t_peri_callback, tdata);

    perllcm_rdi_bathy_t_subscription_t *perllcm_rdi_bathy_t_sub =
        perllcm_rdi_bathy_t_subscribe (shm->lcm, VAN_DVL_BATHY_CHANNEL,
                                       &perllcm_rdi_bathy_t_callback, tdata);

    while (!shm->done)
    {
        struct timeval timeout =
        {
            .tv_sec = 0,
            .tv_usec = 500000,
        };
        lcmu_handle_timeout (shm->lcm, &timeout);
    }
    cvDestroyAllWindows ();

    // wait for thread pool to expire
    g_thread_pool_free (tdata->pool, 1, 1);

    // clean up
    bot_core_image_t_unsubscribe (shm->lcm, bot_core_image_t_subUw);
    bot_core_image_t_unsubscribe (shm->lcm, bot_core_image_t_subPeri);
    perllcm_pose3d_t_unsubscribe (shm->lcm, perllcm_pose3d_t_subUw);
    perllcm_pose3d_t_unsubscribe (shm->lcm, perllcm_pose3d_t_subPeri);
    perllcm_rdi_bathy_t_unsubscribe (shm->lcm, perllcm_rdi_bathy_t_sub);
    gslu_matrix_free (tdata->featpatch_sampler);
    free (tdata);

    printf ("Exiting\n");
    g_thread_exit (0);
    return NULL;
}
