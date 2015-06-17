#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h> // needed for PRId64 macros

// external linking req'd
#include <glib.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "perls-common/cache.h"
#include "perls-common/common.h"
#include "perls-common/bot_util.h"
#include "perls-common/lcm_util.h"

#include "perls-math/gsl_util.h"

#include "perls-vision/feature.h"
#include "perls-vision/plot.h"

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/perllcm_van_feature_collection_t.h"
#include "perls-lcmtypes/perllcm_van_options_t.h"
#include "perls-lcmtypes/perllcm_van_plot_debug_t.h"
#include "perls-lcmtypes/perllcm_van_verify_ack_t.h"
#include "perls-lcmtypes/perllcm_van_saliency_t.h"
#include "perls-lcmtypes/perllcm_isam_cmd_t.h"

#include "shared_memory.h"
#include "plot_thread.h"

#define printf(format, ...)                             \
    printf ("%-12s " format, "[plot]", ## __VA_ARGS__)

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define FC_CACHE_MAX         100

typedef struct thread_data thread_data_t;
struct thread_data
{
    cache_t     *fc_cache; // feature collection cache

    bool plot_vice_versa;
    float opencv_scale;

    // gnuplot pipe descriptor
    FILE *gp_relpose;
    FILE *gp_3pts;
    FILE *gp_relpose_3pts;

    // for gnuplot in while loop
    gsl_vector *pose;
    gsl_vector *nav;
    gsl_matrix *X;
    double gnuplot_scale;

    // @TODO move this into van_opts (need to fix mit-svn, too)
    bool plot_saliency;
};

static void
glist_destroyer (gpointer data, gpointer user)
{
    free (data);
}

static void
perllcm_van_feature_collection_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
        const perllcm_van_feature_collection_t *fc, void *user)
{
    thread_data_t *tdata = user;
    IplImage *img_warp = NULL;  // read image from cache/disk

    int cv_waitkey_msec = shm->van_opts.vis_plot_waitkey ? 0 : 5;

    // plot features...
    // -------------------------------------------------------------- //
    if (shm->van_opts.vis_plot_features)
    {
        img_warp = vis_cvu_iplimg_pop (shm->imgcache, shm->logdir, fc->utime);

        if (!img_warp)
        {
            img_warp = vis_cvu_iplimg_pop (shm->imgcache, shm->logdir, fc->utime);
            printf ("null");
        }

        for (size_t n=0; n < fc->ntypes; n++)
        {
            const perllcm_van_feature_t *f = &fc->f[n];
            switch (f->attrtype)
            {
            case PERLLCM_VAN_FEATURE_T_ATTRTYPE_SURFGPU:
            case PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF:
                vis_plot_feature (img_warp, f, PLOT_WIN_SURF, tdata->opencv_scale, fc->utime);
                break;

            case PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVHARRIS:
                vis_plot_feature (img_warp, f, PLOT_WIN_HARRIS, tdata->opencv_scale, fc->utime);
                break;

            case PERLLCM_VAN_FEATURE_T_ATTRTYPE_SIFTGPU:
                vis_plot_feature (img_warp, f, PLOT_WIN_SIFTGPU, tdata->opencv_scale, fc->utime);
                break;

            default:
                ERROR ("unrecognized attrtype");
                exit (-1);
            }
        }
        cvWaitKey (cv_waitkey_msec);
    }

    if (shm->van_opts.vis_plot_scene_prior)
    {
        if (!img_warp)
            img_warp = vis_cvu_iplimg_pop (shm->imgcache, shm->logdir, fc->utime);

        vis_plot_scene_prior (img_warp, &fc->scene_prior, PLOT_WIN_SCENEPRIOR, fc->utime, tdata->opencv_scale);
        cvWaitKey (cv_waitkey_msec);
    }

    // clean up
    if (img_warp)
        cvReleaseImage (&img_warp);
}

static void
_plot_window_cleanup (const perllcm_van_options_t *plotopt)
{
    if (!plotopt->vis_plot_features)
    {
        cvDestroyWindow (PLOT_WIN_SURF);
        cvDestroyWindow (PLOT_WIN_SIFTGPU);
        cvDestroyWindow (PLOT_WIN_HARRIS);
    }

    if (!plotopt->vis_plot_scene_prior)
        cvDestroyWindow (PLOT_WIN_SCENEPRIOR);

    if (!plotopt->vis_plot_put_corr)
        cvDestroyWindow (PLOT_WIN_PUTCORR);

    if (!plotopt->vis_plot_search_ellipses)
    {
        cvDestroyWindow (PLOT_WIN_SEARCH_ELLIPSES1);
        cvDestroyWindow (PLOT_WIN_SEARCH_ELLIPSES2);
    }

    if (!plotopt->vis_plot_in_and_out)
        cvDestroyWindow (PLOT_WIN_IN_OUT);

    if (!plotopt->vis_plot_inliers)
        cvDestroyWindow (PLOT_WIN_IN);

    if (!plotopt->vis_plot_summary)
        cvDestroyWindow (PLOT_WIN_SUMMARY);

    if (!plotopt->vis_use_saliency)
        cvDestroyWindow (PLOT_WIN_SALIENCY);

//    if (!plotopt->vis_plot_relpose)

//    if (!plotopt->vis_plot_3pts)

//    if (!plotopt->vis_plot_relpose_3pts)

}

static void
perllcm_van_options_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                const perllcm_van_options_t *msg, void *user)
{
    if (!msg->self)
        shm->van_opts = *msg;

    _plot_window_cleanup (&(shm->van_opts));
}

static void
perllcm_van_options_t_broadcast (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const perllcm_heartbeat_t *msg, void *user)
{
    shm->van_opts.self = 1;
    perllcm_van_options_t_publish (shm->lcm, VAN_OPTIONS_CHANNEL, &shm->van_opts);
}

static void
publish_verify_ack (int64_t utime1, int64_t utime2, int result)
{
    bool valid = false;
    bool stop = false;
    if (result == PERLLCM_VAN_PLOT_DEBUG_T_MSG_VALID)
        valid = true;
    else if (result == PERLLCM_VAN_PLOT_DEBUG_T_MSG_QUIT)
    {
        stop = true;
        shm->van_opts.manual_corr = false;
        shm->van_opts.n_plinks = 3;
        perllcm_van_options_t_publish (shm->lcm, VAN_OPTIONS_CHANNEL, &shm->van_opts);
    }

    perllcm_van_verify_ack_t v_ack =
    {
        .utime1 = utime1,
        .utime2 = utime2,
        .valid = valid,
        .stop_verifying = stop,
    };

    perllcm_van_verify_ack_t_publish (shm->lcm, VAN_VERIFY_CHANNEL, &v_ack);

    if (result == PERLLCM_VAN_PLOT_DEBUG_T_MSG_BATCH)
    {
        char savepath[] = "";
        char loadpath[] = "";
        perllcm_isam_cmd_t p = {0};
        p.mode = PERLLCM_ISAM_CMD_T_MODE_BATCH;
        p.savepath = savepath;
        p.graphfile = loadpath;
        perllcm_isam_cmd_t_publish (shm->lcm, SE_OPTION_CMD_CHANNEL, &p);
    }

}

static void
perllcm_van_plot_debug_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                   const perllcm_van_plot_debug_t *dp, void *user)
{

    thread_data_t *tdata = user;
    int cv_waitkey_msec = shm->van_opts.vis_plot_waitkey ? 0 : 5;

    if (!dp->n_in_pccs && !dp->dof && !dp->npts3d && !dp->plt_ellipses)
    {
        // nothing to plot
        return;
    }

    // check plot options
    bool plt_manual_verification = dp->n_in && shm->van_opts.manual_corr;
    bool plt_pccs                = dp->n_in_pccs && shm->van_opts.vis_plot_put_corr;
    bool plt_ellipses            = dp->plt_ellipses;
    bool plt_inout               = dp->n_in && shm->van_opts.vis_plot_in_and_out;
    bool plt_inliers             = dp->n_in && shm->van_opts.vis_plot_inliers;
    bool plt_summary             = dp->n_in && shm->van_opts.vis_plot_summary && dp->reg_result == PERLLCM_VAN_PLOT_DEBUG_T_REG_SUCC;

    bool need_uv = plt_manual_verification || plt_pccs || plt_ellipses || plt_inout || plt_inliers || plt_summary;
    bool need_inliers = plt_pccs || plt_manual_verification || plt_inout || plt_inliers || plt_summary;

    // prepare data needed
    // ---------------------------------------------------
    IplImage *imgi = NULL, *imgj = NULL;
    gsl_matrix *uvic = NULL, *uvjc = NULL;
    gsl_matrix *uvi_sel = NULL, *uvj_sel = NULL;
    gslu_index *selic = NULL, *seljc = NULL;

    if (need_uv)
    {
        // load image and uv points
        // fci
        perllcm_van_feature_collection_t *fci = cache_pop (tdata->fc_cache, dp->utime_i);
        if (!fci)
        {
            char filename[PATH_MAX];
            snprintf (filename, sizeof filename, "%s/%"PRId64".feat", shm->logdir, dp->utime_i);
            int32_t ret = LCMU_FREAD (filename, &fci, perllcm_van_feature_collection_t);
            if (ret < 0)
                ERROR ("couldn't read %s from disk!  ret=%d", filename, ret);
            else
                cache_push (tdata->fc_cache, dp->utime_i, fci);
        }

        // fcj
        perllcm_van_feature_collection_t *fcj = cache_pop (tdata->fc_cache, dp->utime_j);
        if (!fcj)
        {
            char filename[PATH_MAX];
            snprintf (filename, sizeof filename, "%s/%"PRId64".feat", shm->logdir, dp->utime_j);
            int32_t ret = LCMU_FREAD (filename, &fcj, perllcm_van_feature_collection_t);
            if (ret < 0)
                ERROR ("couldn't read %s from disk!  ret=%d", filename, ret);
            else
                cache_push (tdata->fc_cache, dp->utime_j, fcj);
        }

        if (!fci || !fcj)
        {
            return;
        }

        // get images ready
        IplImage *_imgi = vis_cvu_iplimg_pop (shm->imgcache, shm->logdir, dp->utime_i);
        IplImage *_imgj = vis_cvu_iplimg_pop (shm->imgcache, shm->logdir, dp->utime_j);

        if (!_imgi || !_imgj)
        {
            if(_imgi) cvReleaseImage (&_imgi);
            if(_imgj) cvReleaseImage (&_imgj);
            printf ("ERROR: images are NULL\n");
            return;
        }
        // duplicate images to work with (this should really done inside the individual vis_plot family of functions)
        if (_imgi->nChannels == 1)   // gray scale
        {
            imgi = cvCreateImage (cvGetSize (_imgi), IPL_DEPTH_8U, 3);
            imgj = cvCreateImage (cvGetSize (_imgj), IPL_DEPTH_8U, 3);
            cvCvtColor (_imgi, imgi, CV_GRAY2BGR);
            cvCvtColor (_imgj, imgj, CV_GRAY2BGR);
        }
        else   // color
        {
            imgi = cvCloneImage (_imgi);
            imgj = cvCloneImage (_imgj);
        }
        cvReleaseImage (&_imgi);
        cvReleaseImage (&_imgj);

        // add utime label
        CvScalar color = CV_RGB (255, 255, 0);
        vis_plot_add_utime (imgi, dp->utime_i, NULL, NULL, &color);
        vis_plot_add_utime (imgj, dp->utime_j, NULL, NULL, &color);

        // all points from feature collection
        uvic = vis_feature_get_uvc_alloc (fci);
        uvjc = vis_feature_get_uvc_alloc (fcj);
    }

    if (need_inliers)
    {
        // populate inliers when plotting inliers and correspondences
        selic = gslu_index_alloc (dp->n_in_pccs);
        seljc = gslu_index_alloc (dp->n_in_pccs);

        for (size_t ii=0; ii<dp->n_in_pccs; ii++)
        {
            gslu_index_set (selic, ii, dp->isel_pccs_i[ii]);
            gslu_index_set (seljc, ii, dp->isel_pccs_j[ii]);
        }

        uvi_sel = gslu_matrix_selcol_alloc (uvic, selic);
        uvj_sel = gslu_matrix_selcol_alloc (uvjc, seljc);
    }

    int64_t dt = dp->utime_j - dp->utime_i;

    // plot pccs, f_inliers, h_inliers and search bound
    // ---------------------------------------------------
    // search bound ellipses
    if (plt_ellipses)
    {
        vis_plot_pccs_ellipses_plotdebug (dp, imgi, imgj, uvic, uvjc, tdata->plot_vice_versa, tdata->opencv_scale, dt);
        cvWaitKey (cv_waitkey_msec);
    }

    // plot debug windows
    // ---------------------------------------------------
    // NOTE: we call cvWaitKey inside, and publish se verified link
    if (plt_manual_verification)
    {
        int result = vis_plot_manual_verification_debugplot (dp, imgi, imgj, uvi_sel, uvj_sel, tdata->opencv_scale);
        publish_verify_ack (dp->utime_i, dp->utime_j, result);
        cvDestroyWindow (PLOT_WIN_VERIFY);
    }

    // pccs put corr
    if (plt_pccs)
    {
        vis_plot_pccs_debugplot (dp, imgi, imgj, selic, seljc, uvic, uvjc, tdata->opencv_scale);
        cvWaitKey (cv_waitkey_msec);
    }

    // both inliers and outliers
    if (plt_inout)
    {
        vis_plot_in_and_outliers_debugplot (dp, imgi, imgj, uvi_sel, uvj_sel, VIS_PLOT_INOUT, tdata->opencv_scale);
        cvWaitKey (cv_waitkey_msec);
    }

    // inliers only
    if (plt_inliers)
    {
        vis_plot_in_and_outliers_debugplot (dp, imgi, imgj, uvi_sel, uvj_sel, VIS_PLOT_IN, tdata->opencv_scale);
        cvWaitKey (cv_waitkey_msec);
    }

    // summary motion
    if (plt_summary)
    {
        vis_plot_summary_debugplot (dp, imgi, imgj, uvi_sel, uvj_sel, tdata->opencv_scale);
        cvWaitKey (cv_waitkey_msec);
    }

    // clean up
    if(imgi) cvReleaseImage (&imgi);
    if(imgj) cvReleaseImage (&imgj);
    gslu_matrix_free (uvic);
    gslu_matrix_free (uvjc);
    gslu_matrix_free (uvi_sel);
    gslu_matrix_free (uvj_sel);
    gslu_index_free (selic);
    gslu_index_free (seljc);

    // plot relative pose
    // ---------------------------------------------------
    if (dp->dof)
    {
        if (!tdata->pose)
            tdata->pose = gsl_vector_alloc (dp->dof);

        gsl_vector_const_view x21_view = gsl_vector_const_view_array (dp->x21, dp->dof);
        gsl_vector_memcpy (tdata->pose, &x21_view.vector);
        tdata->gnuplot_scale = dp->x21[5];

        if (!tdata->nav)
            tdata->nav = gsl_vector_alloc (6);

        gsl_vector_const_view nav21_view = gsl_vector_const_view_array (dp->nav21, 6);
        gsl_vector_memcpy (tdata->nav, &nav21_view.vector);

    }

    if (dp->npts3d)
    {
        gslu_matrix_free (tdata->X);
        tdata->X= gsl_matrix_alloc (3, dp->npts3d);

        for (size_t i=0; i<dp->npts3d; i++)
        {
            gsl_matrix_set (tdata->X, 0, i, (double) dp->x[i]);
            gsl_matrix_set (tdata->X, 1, i, (double) dp->y[i]);
            gsl_matrix_set (tdata->X, 2, i, (double) dp->z[i]);
        }
    }
}


static void
perllcm_van_saliency_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const perllcm_van_saliency_t *sal, void *user)
{
    thread_data_t *tdata = user;

    int cv_waitkey_msec = shm->van_opts.vis_plot_waitkey ? 0 : 5;

    if (shm->van_opts.vis_use_saliency)
    {
        // load image
        IplImage *img = vis_cvu_iplimg_pop (shm->imgcache, shm->logdir, sal->utime);

        // plot saliency
        if (img)
        {
            vis_plot_saliency (sal, img, PLOT_WIN_SALIENCY, tdata->opencv_scale);
            cvWaitKey (cv_waitkey_msec);
            cvReleaseImage (&img);
        }
    }
}

void
gnuplot (thread_data_t *tdata)
{

    if (tdata->pose)
    {
        if (shm->van_opts.vis_plot_relpose)
            vis_plot_relpose (tdata->gp_relpose, tdata->pose, tdata->gnuplot_scale, tdata->nav);
    }

    if (tdata->X)
    {
        if (shm->van_opts.vis_plot_3pts)
            vis_plot_3dpts (tdata->gp_3pts, tdata->X);
    }

    if (tdata->pose && tdata->X)
    {
        if (shm->van_opts.vis_plot_relpose_3pts)
            vis_plot_relpose_3dpts (tdata->gp_relpose_3pts, tdata->pose, tdata->X, tdata->gnuplot_scale, tdata->nav);
    }

}

static void
perllcm_van_feature_collection_t_destroy_wrapper (void *value)
{
    perllcm_van_feature_collection_t_destroy (value);
}

gpointer
plot_thread (gpointer user)
{
    printf ("Spawning\n");

    // plot_thread state
    thread_data_t *tdata = calloc (1, sizeof (*tdata));

    // plot options
    double opencv_scale_double = 1.0;
    bot_param_get_double (shm->param, "rtvan.plot_thread.scale", &opencv_scale_double);
    tdata->opencv_scale = opencv_scale_double;

    tdata->plot_vice_versa = 1;
    botu_param_get_boolean_to_bool (shm->param, "rtvan.plot_thread.plot_vice_versa", &tdata->plot_vice_versa);

    // plot options
    tdata->gp_relpose = popen("gnuplot","w"); // 'gp' is the pipe descriptor-persist
    tdata->gp_3pts = popen("gnuplot","w"); // 'gp' is the pipe descriptor-persist
    tdata->gp_relpose_3pts = popen("gnuplot","w"); // 'gp' is the pipe descriptor-persist
    tdata->plot_saliency = 0;
    botu_param_get_boolean_to_bool (shm->param, "rtvan.plot_thread.vis_plot_saliency", &tdata->plot_saliency);

    if (!tdata->gp_relpose || !tdata->gp_3pts || !tdata->gp_relpose_3pts)
        printf("Error opening pipe to GNU plot. sudo apt-get install gnuplot \n");

    tdata->fc_cache = cache_new (FC_CACHE_MAX, NULL, &perllcm_van_feature_collection_t_destroy_wrapper);

    // lcm subscriptions
    perllcm_van_options_t_subscription_t *perllcm_van_options_t_sub =
        perllcm_van_options_t_subscribe (shm->lcm, VAN_OPTIONS_CHANNEL, perllcm_van_options_t_callback, tdata);

    char *hb1_channel = lcmu_channel_get_heartbeat (NULL, 1);
    perllcm_heartbeat_t_subscription_t *perllcm_heartbeat_t_sub =
        perllcm_heartbeat_t_subscribe (shm->lcm, hb1_channel, perllcm_van_options_t_broadcast, tdata);
    free (hb1_channel);

    perllcm_van_feature_collection_t_subscription_t *perllcm_van_feature_collection_t_sub =
        perllcm_van_feature_collection_t_subscribe (shm->lcm, VAN_FEATURE_COLLECTION_CHANNEL,
                &perllcm_van_feature_collection_t_callback, tdata);

    perllcm_van_plot_debug_t_subscription_t *perllcm_van_plot_debug_t_sub =
        perllcm_van_plot_debug_t_subscribe (shm->lcm, VAN_PLOT_DEBUG_CHANNEL, &perllcm_van_plot_debug_t_callback, tdata);

    perllcm_van_saliency_t_subscription_t *perllcm_van_saliency_t_sub =
        perllcm_van_saliency_t_subscribe (shm->lcm, VAN_SALIENCY_CHANNEL, &perllcm_van_saliency_t_callback, tdata);

    int cv_waitkey_msec = shm->van_opts.vis_plot_waitkey ? 0 : 5;
    while (!shm->done)
    {
        gnuplot(tdata);
        struct timeval timeout =
        {
            .tv_sec = 0,
            .tv_usec = 500000,
        };
        if (0==lcmu_handle_timeout (shm->lcm, &timeout))
            cvWaitKey (cv_waitkey_msec);
    }

    // clean up
    perllcm_van_options_t_unsubscribe (shm->lcm, perllcm_van_options_t_sub);
    perllcm_heartbeat_t_unsubscribe (shm->lcm, perllcm_heartbeat_t_sub);
    perllcm_van_feature_collection_t_unsubscribe (shm->lcm, perllcm_van_feature_collection_t_sub);
    perllcm_van_plot_debug_t_unsubscribe (shm->lcm, perllcm_van_plot_debug_t_sub);
    perllcm_van_saliency_t_unsubscribe (shm->lcm, perllcm_van_saliency_t_sub);
    cache_destroy (tdata->fc_cache);

    // gnu pipe close
    if (tdata->gp_relpose != NULL)
        fclose (tdata->gp_relpose);

    if (tdata->gp_3pts != NULL)
        fclose (tdata->gp_3pts);

    if (tdata->gp_relpose_3pts != NULL)
        fclose (tdata->gp_relpose_3pts);

    printf ("Exiting\n");
    g_thread_exit (0);
    return NULL;
}
