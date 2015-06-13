#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h> // needed for PRId64 macros

// external linking req'd
#include <glib.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_statistics_float.h>

#include "perls-common/lcm_util.h"
#include "perls-common/units.h"

#include "perls-math/gsl_util.h"
#include "perls-math/ssc.h"

#include "perls-vision/feature.h"

#include "perls-lcmtypes/perllcm_pose3d_t.h"
#include "perls-lcmtypes/perllcm_van_feature_collection_t.h"
#include "perls-lcmtypes/perllcm_van_plink_t.h"

#include "shared_memory.h"
#include "link_thread.h"

#define printf(format, ...)                             \
    printf ("%-12s " format, "[link]", ## __VA_ARGS__)

#define MAX_HYPOTHESES  1

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

typedef struct thread_data thread_data_t;
struct thread_data
{
    GList     *imglist; // list of image events
    perllcm_pose3d_t x_lc; // latest camera pose

    // min and max image overlap percentage
    double min_overlap;
    double max_overlap;
};

static void
imglist_printf (gpointer data, gpointer user_data)
{
    printf ("utime=%"PRId64"\n", *(int64_t*)data);
}

static void
perllcm_pose3d_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                           const perllcm_pose3d_t *msg, void *user)
{
    thread_data_t *tdata = user;
    tdata->x_lc = *msg;
}


/* Generate a set of new candidate link hypotheses once we know that the image features
 * have been extracted
 */
static void
perllcm_van_feature_collection_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
        const perllcm_van_feature_collection_t *fc, void *user)
{
    thread_data_t *tdata = user;

    // insert imgage event into a linked list for quick lookup
    perllcm_pose3d_t *x_lcj = perllcm_pose3d_t_copy (&tdata->x_lc);
    x_lcj->utime = fc->utime;
    GList *eventj __attribute__((unused))= tdata->imglist = g_list_prepend (tdata->imglist, x_lcj);

    // get camera offset from bathy
    double camoffset = vis_feature_get_scenedepth (&fc->f[0]);

    // propose the last MAX_HYPOTHESES candidates
    GList *eventi = g_list_next (tdata->imglist);
    size_t imglistlen = g_list_length (tdata->imglist);
    for (size_t i=0; i < GSL_MIN (MAX_HYPOTHESES, imglistlen-1); i++, eventi = g_list_next (eventi))
    {
        perllcm_pose3d_t *x_lci = eventi->data;

        // a priori relative pose of ci w.r.t. cj
        perllcm_pose3d_t x_cjci = {.utime = x_lcj->utime};
        ssc_tail2tail (x_cjci.mu, NULL, x_lcj->mu, x_lci->mu);

        // check min/max image overlap
        double overlap = 0;
        double dx = fabs (x_cjci.mu[0]), dy = fabs (x_cjci.mu[1]);
        double fov_w = fc->calib.fov[0];
        double fov_h = fc->calib.fov[1];
        if (dx > dy) // overlap w.r.t. fov_w (horizontal motion)
            overlap = (2*camoffset*tan(0.5*fov_w) - dx) / (2*camoffset*tan(0.5*fov_w));
        else
            overlap = (2*camoffset*tan(0.5*fov_h) - dy) / (2*camoffset*tan(0.5*fov_h));
        //printf ("camoffset = %g [m], overlap = %g%%\n", camoffset, overlap);

        if (overlap < tdata->max_overlap && tdata->min_overlap < overlap)
        {
            // fudge covariance estimate for now
            const double sigma_x = 0.001*x_cjci.mu[0];   // 0.01 oceanus testing
            const double sigma_y = 0.001*x_cjci.mu[1];
            const double sigma_z = 0.001*x_cjci.mu[2];
            const double sigma_r = 0.1*DTOR;
            const double sigma_p = 0.1*DTOR;
            const double sigma_h = 0.1*DTOR;
            GSLU_VECTOR_VIEW (x_cjci_variance, 6,
            {
                sigma_x*sigma_x, sigma_y*sigma_y, sigma_z*sigma_z,
                sigma_r*sigma_r, sigma_p*sigma_p, sigma_h*sigma_h
            });
            gsl_matrix_view Sigma = gsl_matrix_view_array (x_cjci.Sigma, 6, 6);
            gsl_vector_view Sigma_diag = gsl_matrix_diagonal (&Sigma.matrix);
            gsl_vector_memcpy (&Sigma_diag.vector, &x_cjci_variance.vector);

            // stuff and send plink lcm msg
            perllcm_van_plink_t plink =
            {
                .utime_i = x_lci->utime,
                .utime_j = x_lcj->utime,
                .prior   = 1,
                .x_ji    = x_cjci,
            };
            perllcm_van_plink_t_publish (shm->lcm, VAN_PLINK_CHANNEL, &plink);
        }
    }

    // keep list at finite length
    tdata->imglist = g_list_reverse (tdata->imglist); /* (reverse list so that we delete
                                                       * elements off tail) */
    while (imglistlen > MAX_HYPOTHESES)
    {
        GList *eventk = tdata->imglist;
        perllcm_pose3d_t_destroy (eventk->data);
        tdata->imglist = g_list_delete_link (tdata->imglist, eventk);
        imglistlen--;
    }
    tdata->imglist = g_list_reverse (tdata->imglist);

    //g_list_foreach (tdata->imglist, &imglist_printf, NULL);
}


gpointer
link_thread (gpointer user)
{
    printf ("Spawning\n");

    // link_thread state
    thread_data_t *tdata = calloc (1, sizeof (*tdata));
    tdata->imglist = NULL;

    // min and max image overlap percentage
    tdata->min_overlap = 0.0;
    bot_param_get_double (shm->param, "rtvan.link_thread.min_overlap", &tdata->min_overlap);

    tdata->max_overlap = 1.0;
    bot_param_get_double (shm->param, "rtvan.link_thread.max_overlap", &tdata->max_overlap);

    // lcm subscriptions
    perllcm_van_feature_collection_t_subscription_t *perllcm_van_feature_collection_t_sub =
        perllcm_van_feature_collection_t_subscribe (shm->lcm, VAN_FEATURE_COLLECTION_CHANNEL,
                &perllcm_van_feature_collection_t_callback, tdata);

    perllcm_pose3d_t_subscription_t *perllcm_pose3d_t_subUw =
        perllcm_pose3d_t_subscribe (shm->lcm, VAN_CAMERA_POSE_UW_CHANNEL,
                                    &perllcm_pose3d_t_callback, tdata);
    perllcm_pose3d_t_subscription_t *perllcm_pose3d_t_subPeri =
        perllcm_pose3d_t_subscribe (shm->lcm, VAN_CAMERA_POSE_PERI_CHANNEL,
                                    &perllcm_pose3d_t_callback, tdata);

    while (!shm->done)
    {
        struct timeval timeout =
        {
            .tv_sec = 0,
            .tv_usec = 500000,
        };
        lcmu_handle_timeout (shm->lcm, &timeout);
    }

    // clean up
    perllcm_van_feature_collection_t_unsubscribe (shm->lcm, perllcm_van_feature_collection_t_sub);
    perllcm_pose3d_t_unsubscribe (shm->lcm, perllcm_pose3d_t_subUw);
    perllcm_pose3d_t_unsubscribe (shm->lcm, perllcm_pose3d_t_subPeri);
    free (tdata);

    printf ("Exiting\n");
    g_thread_exit (0);
    return NULL;
}
