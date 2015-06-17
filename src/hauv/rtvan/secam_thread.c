#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h> // needed for PRId64 macros
#include <glib.h>
#include <sys/param.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_sort_vector.h>
#include <gsl/gsl_cdf.h>

#include "perls-common/common.h"
#include "perls-math/math.h"
#include "perls-vision/vision.h"

#include "perls-lcmtypes/perllcm_van_feature_collection_t.h"
#include "perls-lcmtypes/perllcm_pose3d_t.h"
#include "perls-lcmtypes/perllcm_van_plink_t.h"
#include "perls-lcmtypes/hauv_bs_rnv_2_t.h"

// communicate with seserver
#include "perls-lcmtypes/se_add_node_t.h"
#include "perls-lcmtypes/se_add_node_ack_t.h"
#include "perls-lcmtypes/se_request_state_t.h"
#include "perls-lcmtypes/se_return_state_t.h"
#include "perls-lcmtypes/perllcm_isam_cmd_t.h"
#include "perls-lcmtypes/se_goto_t.h"
#include "perls-lcmtypes/se_save_isam_t.h"

#include "shared_memory.h"
#include "secam_thread.h"


#define printf(format, ...)                             \
    printf ("%-12s " format, "[secam]", ## __VA_ARGS__)

#define DTOR UNITS_DEGREE_TO_RADIAN
#define RTOD UNITS_RADIAN_TO_DEGREE

#define MAX_NUM_TO_RQ_COV  1000
#define MAX_NUM_PLINKS     1000
#define FORCE_SEQ_PLINK    0           // we usually want to add a seq pair
#define COVRQ_DIST         3.0         // meters
#define USE_MDIST_COVRQ    0           // or use euclidean based one

//#define SALIENCY_FILE "./saliency.dat"

typedef struct thread_data thread_data_t;
struct thread_data
{

    // about the graph
    bool   initialized;

    // temporary storage for current vehicle state
    perllcm_pose3d_t x_vcUw_current, x_vcPeri_current;  // temp storage for vehicle to camera
    perllcm_pose3d_t x_lcUw_current, x_lcPeri_current;  // temp storage for local to camera
    perllcm_pose3d_t x_lcUw_last, x_lcPeri_last;        // last x_vc that has been added
    double camoffset;               // temp storage for camera offset
    double pathlen_add_node;

    // remember history
    GHashTable   *x_vc;              // store vehicle to camera sensor xform
    GHashTable   *S_ii;              // store block diagonal matrix
    GHashTable   *camoffset_list;    // store cam offset to calculate overlap
    gsl_matrix   *S_recent;          // pointer to the most recent covariance

    // plink list waiting for feature event
    GSList     *plinklist;           // plink list
    int32_t     link_id;             // seserver require link_id (should be unique)

    // min and max image overlap percentage
    double min_overlap;
    double max_overlap;
    double add_node_thresh;
    bool   use_sal_add_node;        // use saliency to add node?
    double S_L_thresh;              // local saliency thresh
    double Ig_thresh;               // geometric informationg gain thresh

    bool uncertain_prior;           // uncertain mode after loading a graph?
    bool mdist_plink;               // use mdist to propose link?
    bool force_seq_plink;

    // process every Nth frame
    int    imgstep;       // current image step to skip in every n image
    int    imgstep_cfg;   // value read from config
    size_t imgcounter;
    int    batchstep;

    // saving an isam graph
    bool save_mode;
    bool saving_done;
    FILE *fid_save;

    // manual plink provided (from a file plinklist.dat)
    int         nplist_manual;        // number of manual links
    GHashTable  *plist_manual;        // manual plink list

    int64_t     current_utime;
    size_t      plink_remaining;

    // saliency
    GHashTable *bowE;
    size_t      vocab_len;

};

static void
imglist_printf (gpointer data, gpointer user_data)
{
    printf ("utime=%"PRId64"\n", *(int64_t*)data);
}

static void
perllcm_pose3d_t_destroy_wrapper (gpointer data)
{
    perllcm_pose3d_t_destroy (data);
}

static void
perllcm_plink_t_destroy_wrapper (gpointer data)
{
    perllcm_van_plink_t_destroy (data);
}

static void
gslu_matrix_free_wrapper (gpointer data)
{
    gslu_matrix_free (data);
}

static void
glist_destroyer (gpointer data, gpointer user)
{
    free (data);
}


// update the current state of the vehicle: x_vs, x_lc, cam_offset
static void
perllcm_pose3d_t_x_vc_uw_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                   const perllcm_pose3d_t *x_vc, void *user)
{
    thread_data_t *tdata = user;
    tdata->x_vcUw_current = *x_vc;
}

static void
perllcm_pose3d_t_x_vc_peri_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                     const perllcm_pose3d_t *x_vc, void *user)
{
    thread_data_t *tdata = user;
    tdata->x_vcPeri_current = *x_vc;
}

static void
perllcm_pose3d_t_x_lc_uw_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                   const perllcm_pose3d_t *x_lc, void *user)
{
    thread_data_t *tdata = user;
    tdata->x_lcUw_current = *x_lc;
}

static void
perllcm_pose3d_t_x_lc_peri_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                     const perllcm_pose3d_t *x_lc, void *user)
{
    thread_data_t *tdata = user;
    tdata->x_lcPeri_current = *x_lc;
}

static void
hauv_bs_rnv_2_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                          const hauv_bs_rnv_2_t *msg, void *user)
{
    thread_data_t *tdata = user;
    tdata->camoffset = msg->distance;
}

static bool
_check_overlap_criteria (const double x_cjci[6], int64_t dt, double offset, thread_data_t *tdata)
{
    bool valid_plink = false;

    double fov_w = 40*DTOR; //fc->calib.fov[0];
    double fov_h = 30*DTOR; //fc->calib.fov[1];
    double dx = fabs (x_cjci[0]), dy = fabs (x_cjci[1]), overlap = 0;
    if (dx > dy)
        overlap = (2*(offset)*tan(0.5*fov_w) - dx) / (2*(offset)*tan(0.5*fov_w));
    else
        overlap = (2*(offset)*tan(0.5*fov_h) - dy) / (2*(offset)*tan(0.5*fov_h));

    if (tdata->min_overlap < overlap && overlap < tdata->max_overlap && dt > 5E6) valid_plink = true;

    return valid_plink;
}

static bool
_check_mdist_criteria (const gsl_vector *x_c21, const gsl_matrix *S_c21, int64_t dt, thread_data_t *tdata)
{
    bool valid_plink = false;

    GSLU_MATRIX_VIEW (nav_cov_inv, 6, 6);
    GSLU_VECTOR_VIEW (zero, 6);
    GSLU_INDEX_VIEW (c, 3, {3, 4, 5});

    gslu_matrix_inv (&nav_cov_inv.matrix, S_c21);
    double mdist = gslu_vector_mahal_circ_dist (x_c21, &zero.vector, &nav_cov_inv.matrix, &c.vector);

    printf ("mdist = %g dt = %"PRId64"\n", mdist, dt);
    if (mdist < 100000.0 && dt > 60E6)
        valid_plink = true;

    return valid_plink;
}


static void
_publish_plinks (thread_data_t *tdata, int64_t utime, bool is_salient)
{
    size_t plinklistlen = g_slist_length (tdata->plinklist);

    if (is_salient)
    {
        for (size_t i=0, idx=0; i < plinklistlen; i++)
        {
            GSList *event = g_slist_nth (tdata->plinklist, idx);
            perllcm_van_plink_t *x_ji = event->data;

            if (utime >= x_ji->utime_j && x_ji->utime_j > 0)
            {
                if (shm->van_opts.vis_use_saliency)
                {
                    double *bowE = g_hash_table_lookup (tdata->bowE, &(x_ji->utime_i));
                    if (bowE) x_ji->S_L = (*bowE) / log2 ((double) tdata->vocab_len);
                }
                else
                    x_ji->S_L = 1.0;

                // publish & destroy
                if (x_ji->S_L > tdata->S_L_thresh)
                {
                    printf ("plink publish %"PRId64"-%"PRId64"\n", x_ji->utime_i, x_ji->utime_j);
                    perllcm_van_plink_t_publish (shm->lcm, VAN_PLINK_CHANNEL, x_ji);
                }
                perllcm_van_plink_t_destroy (event->data);
                tdata->plinklist = g_slist_delete_link (tdata->plinklist, event);
            }
            else
                idx++;
        }
    }
    else
    {
        printf ("NOT SALIENT: No links proposed\n");

        if (tdata->force_seq_plink)  //(FORCE_SEQ_PLINK) {
        {
            for (size_t i=0, idx=0; i < plinklistlen; i++)
            {
                GSList *event = g_slist_nth (tdata->plinklist, idx);
                perllcm_van_plink_t *x_ji = event->data;

                if (utime >= x_ji->utime_j && x_ji->Ig > 100.0)
                {
                    // publish & destroy
                    perllcm_van_plink_t_publish (shm->lcm, VAN_PLINK_CHANNEL, x_ji);
                    perllcm_van_plink_t_destroy (event->data);
                    tdata->plinklist = g_slist_delete_link (tdata->plinklist, event);
                }
                else
                    idx++;
            }
        }

        for (size_t i=0, idx=0; i < plinklistlen; i++)
        {
            GSList *event = g_slist_nth (tdata->plinklist, idx);
            perllcm_van_plink_t *x_ji = event->data;

            if (utime >= x_ji->utime_j && x_ji->utime_j > 0)
            {
                // publish & destroy
                // printf ("removing plink publish %"PRId64"-%"PRId64"\n", x_ji->utime_i, x_ji->utime_j);
                perllcm_van_plink_t_destroy (event->data);
                tdata->plinklist = g_slist_delete_link (tdata->plinklist, event);
            }
            else
                idx++;
        }
    }
    // make sure everything is freed.
    if (g_slist_length (tdata->plinklist) == 0) g_slist_free (tdata->plinklist);
}

/*  on each bot_core_image_t event
 *  add a node & remember sensor transformation
 */
static void
bot_core_image_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                           const bot_core_image_t *msg, void *user)
{
    thread_data_t *tdata = user;

    perllcm_pose3d_t *x_lc_current, *x_lc_last;
    if (strcmp (channel, shm->bot_core_image_t_channelUw) == 0)
    {
        x_lc_current = &(tdata->x_lcUw_current);
        x_lc_last = &(tdata->x_lcUw_last);
    }
    else if (strcmp (channel, shm->bot_core_image_t_channelPeri) == 0)
    {
        x_lc_current = &(tdata->x_lcPeri_current);
        x_lc_last = &(tdata->x_lcPeri_last);
    }
    else
    {
        ERROR ("Unkown bot_core_image_t channel name");
        exit (1);
    }

    if (!tdata->use_sal_add_node || !shm->van_opts.vis_use_saliency)
    {
        if (!shm->active)
            return;

        // process only every nth frame
        tdata->imgcounter++;
        if ((tdata->imgcounter % shm->imgstep))
        {
            printf ("bot_core_image_t event %"PRId64" - skipping add node\n", msg->utime);
            //printf ("imgstep=%d\n", shm->imgstep);
            return;
        }
        //printf ("bot_core_image_t event %"PRId64" - processing\n", msg->utime);

        bool add_node = false;           // HACK to add all node (for path planning implementation)
        /*// CURTISS HACK----------
          if ((1296927189967721 < msg->utime && msg->utime < 1296927305966933) ||
          (1296932968335437 < msg->utime && msg->utime < 1296933179333844) ||
          (1286128310793854 < msg->utime && msg->utime < 1286128328292624)) {
          return;
          }
          if (msg->utime == 1296927305966933 || msg->utime == 1296933179333844 || msg->utime == 1286128328292624)
          add_node = true;
          // CURTISS HACK----------*/

        // request a batch update every so often
        if ((tdata->imgcounter % tdata->batchstep) == 0)
        {
            se_add_node_t batch_node =
            {
                .utime = msg->utime - 10, // give the batch node a different utime than the image node
                .node_type = SE_ADD_NODE_T_NODE_POSE3D,
                .sensor_id = SE_ADD_NODE_T_NODE_BATCH,
            };
            se_add_node_t_publish (shm->lcm, SE_ADD_NODE_CHANNEL, &batch_node);
            printf ("Sending a batch request\n");
        }

        if (tdata->initialized)
        {
            double x_cji[6];
            ssc_tail2tail (x_cji, NULL, x_lc_current->mu, x_lc_last->mu);      // j = temp = cur, i = last

            double dist2 = x_cji[0]*x_cji[0] + x_cji[1]*x_cji[1] + x_cji[2]*x_cji[2];
            tdata->pathlen_add_node = sqrt (dist2);

            if (tdata->pathlen_add_node > tdata->add_node_thresh || tdata->uncertain_prior)
                add_node = true;
            else
                printf ("x_cji too close -- skipping (d=%g)\n", tdata->pathlen_add_node);
        }
        else
        {
            printf ("CHECK! reinitialized!\n");
            add_node = true;
        }

        if (add_node)
        {
            // add node
            se_add_node_t node;
            node.utime = msg->utime;
            node.node_type = SE_ADD_NODE_T_NODE_POSE3D;
            node.sensor_id = SE_ADD_NODE_T_NODE_CAMERA;
            se_add_node_t_publish (shm->lcm, SE_ADD_NODE_CHANNEL, &node);

            //shm->utimelist = g_list_append (shm->utimelist, gu_dup (&msg->utime, sizeof msg->utime));

            // store sensor transformation
            perllcm_pose3d_t *x_vc = g_malloc (sizeof (*x_vc));
            if (strcmp (channel, shm->bot_core_image_t_channelUw))
                memcpy (x_vc, &tdata->x_vcUw_current, sizeof (*x_vc));
            else if (strcmp (channel, shm->bot_core_image_t_channelPeri))
                memcpy (x_vc, &tdata->x_vcPeri_current, sizeof (*x_vc));
            else
            {
                ERROR ("unknown bot_core_image_t channel name");
                exit (1);
            }

            g_hash_table_insert (tdata->x_vc, gu_dup (&msg->utime, sizeof msg->utime), x_vc);

            // store camera offset
            double *offset = g_malloc (sizeof (*offset));
            *offset = tdata->camoffset;
            g_hash_table_insert (tdata->camoffset_list, gu_dup (&msg->utime, sizeof msg->utime), offset);

            // last pose that has been added as a node
            *x_lc_last = *x_lc_current;

            tdata->pathlen_add_node = 0.0;
        }
    }
}


/*  node has been added and update, then notice from seserver has been received
 */
static void
se_add_node_ack_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                            const se_add_node_ack_t *msg, void *user)
{
    thread_data_t *tdata = user;

    if (msg->sensor_id & SE_ADD_NODE_T_NODE_CAMERA)
    {
        // check if seserver is initialized and sync with se_camclient
        if (!tdata->initialized)
        {
            size_t utimelistlen = g_slist_length (shm->utimelist);
            for (size_t i=0, idx=0; i<utimelistlen; i++)
            {
                GSList *event = g_slist_nth (shm->utimelist, idx);
                int64_t *utime = event->data;
                if (*utime == msg->utime)
                    idx++;
                else
                {
                    printf ("not initialized - deleting %"PRId64"\n", *utime);
                    free (event->data);
                    shm->utimelist = g_slist_delete_link (shm->utimelist, event);
                }
            }
            tdata->initialized = true;
        }

        // bookkeeping utimelist for images
        shm->utimelist = g_slist_append (shm->utimelist, gu_dup (&msg->utime, sizeof msg->utime));
        size_t utimelist_len = g_slist_length (shm->utimelist);

        // request entire state
        se_request_state_t *se_rq_st = calloc (1, sizeof (*se_rq_st));
        se_rq_st->state_type = SE_REQUEST_STATE_T_POSE;
        se_rq_st->n = utimelist_len;
        se_rq_st->variables = malloc (se_rq_st->n * sizeof (*se_rq_st->variables));
        //printf ("requesting entire graph %zu nodes\n", utimelist_len);
        printf ("utimelist add_node_ack_t_callback: %"PRId64"\n", msg->utime);
        for (size_t i=0; i<utimelist_len; i++)
        {
            int64_t *utime_i = g_slist_nth_data (shm->utimelist, i);
            se_rq_st->variables[i] = (*utime_i);
        }
        se_request_state_t_publish (shm->lcm, SE_REQUEST_ST_CHANNEL, se_rq_st);
        se_request_state_t_destroy (se_rq_st);

        // propose a link if manually provided
        if (tdata->nplist_manual > 0)
        {
            perllcm_van_plink_t *plink = g_hash_table_lookup (tdata->plist_manual, &msg->utime);
            if (plink)
            {
                if (plink->prior)
                {
                    printf ("=== FEEDING MANUAL PLINK ===\n");
                    plink->link_id = tdata->link_id;
                    tdata->link_id ++;
                    tdata->plinklist = g_slist_prepend (tdata->plinklist, plink);
                }
            }
        }
    }
}

/*
 */
static void
perllcm_van_feature_collection_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
        const perllcm_van_feature_collection_t *fc, void *user)
{
    thread_data_t *tdata = user;

    bool salient = true;

    if (!shm->van_opts.vis_use_saliency)
        _publish_plinks (tdata, fc->utime, salient);
}

static void
perllcm_van_saliency_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const perllcm_van_saliency_t *sal, void *user)
{
    thread_data_t *tdata = user;

    if (tdata->use_sal_add_node)
    {
        bool add_node = false;

        if (!shm->active || !shm->van_opts.vis_use_saliency)
            return;

        // CURTISS HACK----------
        if ((1296927189967721 < sal->utime && sal->utime < 1296927305966933) ||
                (1296932968335437 < sal->utime && sal->utime < 1296933179333844) ||
                (1286128310793854 < sal->utime && sal->utime < 1286128328292624))
        {
            tdata->force_seq_plink = true;
        }
        else
            tdata->force_seq_plink = false;

        //if (sal->utime == 1296927305966933 || sal->utime == 1296933179333844 || sal->utime == 1286128328292624)
        //    add_node = true;
        // CURTISS HACK----------

        if (tdata->initialized)
        {
            double x_cji[6];
            /* Use underwater camera to compute distance */
            ssc_tail2tail (x_cji, NULL, tdata->x_lcUw_current.mu, tdata->x_lcUw_last.mu);      // j = temp = cur, i = last

            double dist2 = x_cji[0]*x_cji[0] + x_cji[1]*x_cji[1] + x_cji[2]*x_cji[2];
            tdata->pathlen_add_node = sqrt (dist2);

            if (tdata->pathlen_add_node > tdata->add_node_thresh && sal->is_S_L)
                add_node = true;
            else
                printf ("x_cji too close or not salient -- skipping node (d=%g)\n", tdata->pathlen_add_node);

            if (tdata->uncertain_prior) add_node = true;
        }
        else
        {
            printf ("CHECK! reinitialized!\n");
            add_node = true;
        }

        if (add_node)
        {
            tdata->imgcounter++;

            // add node
            se_add_node_t node;
            node.utime = sal->utime;
            node.node_type = SE_ADD_NODE_T_NODE_POSE3D;
            node.sensor_id = SE_ADD_NODE_T_NODE_CAMERA;
            se_add_node_t_publish (shm->lcm, SE_ADD_NODE_CHANNEL, &node);

            //shm->utimelist = g_list_append (shm->utimelist, gu_dup (&sal->utime, sizeof sal->utime));

            // store sensor transformation
            perllcm_pose3d_t *x_vc = g_malloc (sizeof (*x_vc));
            /* TODO: use both underwater and periscope saliency */
            memcpy (x_vc, &tdata->x_vcUw_current, sizeof (*x_vc));
            g_hash_table_insert (tdata->x_vc, gu_dup (&sal->utime, sizeof sal->utime), x_vc);

            // store camera offset
            double *offset = g_malloc (sizeof (*offset));
            *offset = tdata->camoffset;
            g_hash_table_insert (tdata->camoffset_list, gu_dup (&sal->utime, sizeof sal->utime), offset);

            // last pose that has been added as a node
            tdata->x_lcUw_last = tdata->x_lcUw_current;
            tdata->pathlen_add_node = 0.0;

            // request a batch update every so often
            if ((tdata->imgcounter % tdata->batchstep) == 0)
            {
                se_add_node_t batch_node =
                {
                    .utime = sal->utime - 10, // give the batch node a different utime than the image node
                    .node_type = SE_ADD_NODE_T_NODE_POSE3D,
                    .sensor_id = SE_ADD_NODE_T_NODE_BATCH,
                };
                se_add_node_t_publish (shm->lcm, SE_ADD_NODE_CHANNEL, &batch_node);
                printf ("Sending a batch request\n");
            }
        }
    }

    if (shm->van_opts.vis_use_saliency)
    {
        // store saliency score
        double *bowE = g_malloc (sizeof (*bowE));
        *bowE = sal->bowE;
        g_hash_table_insert (tdata->bowE, gu_dup (&sal->utime, sizeof sal->utime), bowE);
        tdata->vocab_len = sal->vocab_len;

        _publish_plinks (tdata, sal->utime, sal->is_S_L);
        //_publish_plinks (tdata, sal->utime, true);
    }
}

static void
_informative_link_hypothesis (const se_return_state_t *state, thread_data_t *tdata)
{
    size_t n = state->n;
    size_t npair = n - 1;

    int64_t utime_j = state->timestamps[npair];
    printf ("cov return: %zu nodes returned to propose (%zu pair) - %"PRId64"\n", n, npair, utime_j);

    // do the manual list first
    int64_t manual_utime_i = 0;
    perllcm_van_plink_t *plink_manual = NULL;
    if (tdata->nplist_manual > 0)
    {
        perllcm_van_plink_t *plink = g_hash_table_lookup (tdata->plist_manual, &utime_j);
        if (plink)
        {
            if (!plink->prior)
            {
                manual_utime_i = plink->utime_i;
                plink_manual = plink;
            }
        }
    }

    // current utime and pose
    perllcm_pose3d_t x_cjci;
    se_pose_t x_lvj = state->poses[npair];
    perllcm_pose3d_t *x_vjc = g_hash_table_lookup (tdata->x_vc, &utime_j);

    // remember the most recent block diagonal
    double *cov = state->covariance;
    gsl_matrix_view cov_rr = gsl_matrix_view_array (cov+36*npair, 6, 6);
    gsl_matrix *S_vrr = gslu_matrix_clone (&cov_rr.matrix);
    g_hash_table_insert (tdata->S_ii, gu_dup (&utime_j, sizeof utime_j), S_vrr);
    tdata->S_recent = S_vrr;    // passing the pointer

    // relative covariance
    GSLU_MATRIX_VIEW (S_vij_marginal, 12, 12);
    gsl_matrix_view S_sub_vii = gsl_matrix_submatrix (&S_vij_marginal.matrix, 0, 0, 6, 6);
    gsl_matrix_view S_sub_vij = gsl_matrix_submatrix (&S_vij_marginal.matrix, 0, 6, 6, 6);
    gsl_matrix_view S_sub_vji = gsl_matrix_submatrix (&S_vij_marginal.matrix, 6, 0, 6, 6);
    gsl_matrix_view S_sub_vjj = gsl_matrix_submatrix (&S_vij_marginal.matrix, 6, 6, 6, 6);
    gsl_matrix_memcpy (&S_sub_vjj.matrix, &cov_rr.matrix);

    perllcm_van_plink_t plinklist_sort[MAX_NUM_PLINKS];
    GSLU_VECTOR_VIEW (distlist, MAX_NUM_PLINKS);
    gsl_vector_set_all (&distlist.vector, GSL_POSINF);

    // some workspaces
    GSLU_MATRIX_VIEW (J, 6, 12);
    GSLU_MATRIX_VIEW (work, 6, 12);
    GSLU_MATRIX_VIEW (H, 5, 12);                // observaion model jacobian
    GSLU_MATRIX_VIEW (R, 5, 5);                 // measurement uncertainty
    gsl_matrix_set_identity (&R.matrix);
    gsl_matrix_scale (&R.matrix, 1E-4);
    GSLU_MATRIX_VIEW (SplusR, 5, 5);            // innovation S + R
    GSLU_MATRIX_VIEW (work_Ig, 5, 12);

    // for each x_lci
    size_t num_valid_plinks = 0;
    for (size_t i=0; i<npair; i++)
    {

        x_cjci.utime = utime_j;
        int64_t utime_i = state->timestamps[i];
        perllcm_pose3d_t *x_vic = g_hash_table_lookup (tdata->x_vc, &utime_i);
        se_pose_t x_lvi = state->poses[i];

        // build cov S_cjci and put them in plinklist
        gsl_matrix_view cov_ij = gsl_matrix_view_array (cov+36*i, 6, 6);
        gsl_matrix_memcpy (&S_sub_vij.matrix, &cov_ij.matrix);
        gsl_matrix_transpose_memcpy (&S_sub_vji.matrix, &cov_ij.matrix);
        gsl_matrix *S_vii = g_hash_table_lookup (tdata->S_ii, &utime_i);

        if (S_vii == NULL)
        {
            printf ("Sii null for %"PRId64"\n", utime_i);
            return;  // ERROR printf ("Sii null for %"PRId64"\n", utime_i);
        }

        gsl_matrix_memcpy (&S_sub_vii.matrix, S_vii);
        ssc_relative_sensor_pose (x_cjci.mu, J.data, x_lvj.mu, x_lvi.mu, x_vjc->mu, x_vic->mu);
        gsl_matrix_view S_c21 = gsl_matrix_view_array (x_cjci.Sigma, 6, 6);
        gslu_blas_mmmT (&S_c21.matrix, &J.matrix, &S_vij_marginal.matrix, &J.matrix, &work.matrix);

        // information gain
        ssc_jacobian_camera_aerph (H.data, x_lvj.mu, x_lvi.mu, x_vjc->mu, x_vic->mu);
        gslu_blas_mmmT (&SplusR.matrix, &H.matrix, &S_vij_marginal.matrix, &H.matrix, &work_Ig.matrix);
        gsl_matrix_add (&SplusR.matrix, &R.matrix);

        double det_R = 1E-20;
        double Ig = 0.5 * log (gslu_matrix_det (&SplusR.matrix) / det_R);
        //printf ("Ig = %g\n", Ig);

        // saliency score
        double *bowE = g_hash_table_lookup (tdata->bowE, &utime_i);
        double saliency = 0.0;
        if (bowE) saliency = (*bowE) / log2 ((double) tdata->vocab_len);

        double I = Ig;
        if (shm->van_opts.vis_use_saliency) I = Ig * saliency;

        int64_t dt = utime_j - utime_i;
        bool valid_plink = false;
        if (tdata->uncertain_prior || tdata->mdist_plink)
        {
            gsl_vector_view x_cji_gsl = gsl_vector_view_array (x_cjci.mu, 6);
            valid_plink = _check_mdist_criteria (&x_cji_gsl.vector, &S_c21.matrix, dt, tdata);
        }
        else
        {
            //double *offset = g_hash_table_lookup (tdata->camoffset_list, &utime_i);
            //valid_plink = _check_overlap_criteria (x_cjci.mu, dt, *offset, tdata);
            if (Ig > tdata->Ig_thresh) valid_plink = true;
        }

        if (tdata->force_seq_plink && i == 0 && dt < 3E6)   //FORCE_SEQ_PLINK
        {
            valid_plink = true;    // force a seq pair ?
            I = 1000.0;
        }
        //printf ("cov req: %"PRId64"-%"PRId64":", utime_i, utime_j);
        //printf ("Ig=%g, S=%g, I=%g\n", Ig, saliency, I);

        if (manual_utime_i == utime_i)
        {
            printf ("=== FEEDING MANUAL PLINK ===\n");
            plink_manual->x_ji = x_cjci;
            tdata->plinklist = g_slist_prepend (tdata->plinklist, perllcm_van_plink_t_copy (plink_manual));
        }

        if (valid_plink && num_valid_plinks < MAX_NUM_PLINKS)
        {
            gsl_vector_set (&distlist.vector, num_valid_plinks, 1.0/I);
            plinklist_sort[num_valid_plinks].utime_i = utime_i;
            plinklist_sort[num_valid_plinks].utime_j = utime_j;
            plinklist_sort[num_valid_plinks].prior = 1;
            plinklist_sort[num_valid_plinks].x_ji = x_cjci;
            plinklist_sort[num_valid_plinks].link_id = 0;
            plinklist_sort[num_valid_plinks].Ig = Ig;
            plinklist_sort[num_valid_plinks].S_L = saliency;
            num_valid_plinks++;
        }
    } // END of for each utime i

    if (num_valid_plinks > 0)
    {
        // sort based upon distance
        gsl_permutation *perm = gsl_permutation_alloc (MAX_NUM_PLINKS);
        gsl_sort_vector_index (perm, &distlist.vector);

        for (size_t i=0; i < shm->van_opts.n_plinks; i++)
        {
            size_t idx = perm->data[i];

            // propose! but wait until feature_t event comes in
            perllcm_van_plink_t *plink = malloc (sizeof (*plink));
            plink->utime_i = plinklist_sort[idx].utime_i;
            plink->utime_j = plinklist_sort[idx].utime_j;
            plink->prior   = plinklist_sort[idx].prior;
            plink->x_ji    = plinklist_sort[idx].x_ji;
            plink->link_id = tdata->link_id;
            tdata->link_id ++;
            plink->Ig      = plinklist_sort[idx].Ig;
            plink->S_L     = plinklist_sort[idx].S_L;
            tdata->plinklist = g_slist_prepend (tdata->plinklist, plink);
        }
        gsl_permutation_free (perm);
    }

}

static void
_sort_cov_request (const se_return_state_t *state, thread_data_t *tdata)
{
    size_t n = state->n;
    size_t npair = n - 1;

    int64_t utime_j = state->timestamps[npair];
    printf ("mu return: %zu nodes in the graph. request cov - %"PRId64"\n", n, utime_j);

    // request for the first node covariance and return wo sorting
    if (n == 1)
    {
        se_request_state_t *se_rq_st = calloc (1, sizeof(*se_rq_st));
        se_rq_st->state_type = SE_REQUEST_STATE_T_COV_RIGHTCOL | SE_REQUEST_STATE_T_POSE;
        se_rq_st->n = 1;
        se_rq_st->variables = malloc (se_rq_st->n * sizeof (se_rq_st->variables));
        se_rq_st->variables[0] = utime_j;
        se_request_state_t_publish (shm->lcm, SE_REQUEST_ST_CHANNEL, se_rq_st);
        se_request_state_t_destroy (se_rq_st);
        return;
    }

    // current utime and pose
    perllcm_pose3d_t x_lci, x_lcj, x_cjci;
    se_pose_t x_lvj = state->poses[npair];
    perllcm_pose3d_t *x_vjc = g_hash_table_lookup (tdata->x_vc, &utime_j);
    int64_t rq_cov_utimelist[MAX_NUM_TO_RQ_COV];
    GSLU_VECTOR_VIEW (distlist, MAX_NUM_TO_RQ_COV);
    gsl_vector_set_all (&distlist.vector, GSL_POSINF);
    double J[6*12];
    ssc_head2tail (x_lcj.mu, J, x_lvj.mu, x_vjc->mu);                // x_lcj

#if USE_MDIST_COVRQ
    // recent covariance
    bool use_mdist = false;
    gsl_matrix_view J_view = gsl_matrix_view_array (J, 6, 12);
    gsl_matrix_view Jsub = gsl_matrix_submatrix (&J_view.matrix, 0, 0, 6, 6);
    GSLU_MATRIX_VIEW (S_cjj_inv, 6, 6);
    gsl_matrix_set_zero (&S_cjj_inv.matrix);
    GSLU_MATRIX_VIEW (work, 6,6);
    gslu_blas_mmmT (&S_cjj_inv.matrix, &Jsub.matrix, tdata->S_recent, &Jsub.matrix, &work.matrix);

    /*// verify
      gsl_vector_view x_lvj_view = gsl_vector_view_array (x_lvj.mu, 6);
      gsl_vector_view x_vjc_view = gsl_vector_view_array (x_vjc->mu, 6);
      gslu_vector_printf (&x_lvj_view.vector, "x_lvj");
      gslu_vector_printf (&x_vjc_view.vector, "x_vjc");
      gslu_matrix_printf (&Jsub.matrix, "J");
      gslu_matrix_printf (tdata->S_recent, "S");
      gslu_matrix_printf (&S_cjj_inv.matrix, "Scjj");*/
    //
    GSLU_VECTOR_VIEW (zero, 6);
    GSLU_INDEX_VIEW (c, 3, {3, 4, 5});
    if (tdata->S_recent)   // when covariance is available
    {
        //gslu_matrix_printf (tdata->S_recent, "S_rr");
        gsl_error_handler_t *default_handler = gsl_set_error_handler_off ();
        if (gsl_linalg_cholesky_decomp (&S_cjj_inv.matrix) != GSL_EDOM)
        {
            gsl_linalg_cholesky_invert (&S_cjj_inv.matrix);
            use_mdist = true;
            //gslu_matrix_printf (&S_cjj_inv.matrix, "inv Scjj");
        }
        gsl_set_error_handler (default_handler);
    }
#endif

    // request covariance
    size_t num_request = 0;
    // do the manual list first
    if (tdata->nplist_manual > 0)
    {
        perllcm_van_plink_t *plink = g_hash_table_lookup (tdata->plist_manual, &utime_j);
        if (plink)
        {
            if (!plink->prior)
            {
                printf ("=== MANUAL PLINK COV REQUEST ===\n");
                rq_cov_utimelist[num_request] = plink->utime_i;
                gsl_vector_set (&distlist.vector, num_request, 0.0);    // set 0 distance to manual links
                num_request++;
            }
        }
    }

    // then, do normal candidates
    for (size_t i=0; i<npair; i++)   // for each x_lci
    {
        //int64_t *utime_i = g_list_nth_data (shm->utimelist, i);
        int64_t utime_i = state->timestamps[i];
        perllcm_pose3d_t *x_vic = g_hash_table_lookup (tdata->x_vc, &utime_i);
        se_pose_t x_lvi = state->poses[i];
        ssc_head2tail (x_lci.mu, NULL, x_lvi.mu, x_vic->mu);
        ssc_tail2tail (x_cjci.mu, NULL, x_lcj.mu, x_lci.mu);      // x_cjci

#if USE_MDIST_COVRQ
        perllcm_pose3d_t x_vjvi;
        ssc_tail2tail (x_vjvi.mu, NULL, x_lvj.mu, x_lvi.mu);      // x_cjci
        gsl_vector_view x_v21_view = gsl_vector_view_array (x_vjvi.mu, 6);  // prepare gsl view

        double mdist = gslu_vector_mahal_circ_dist (&x_v21_view.vector, &zero.vector, &S_cjj_inv.matrix, &c.vector);

        if (mdist < 10000.0 && (num_request < MAX_NUM_TO_RQ_COV))
        {
            // request again with covariance
            //printf ("re-requesting %"PRId64",%"PRId64"\n", utime_i, utime_j);
            rq_cov_utimelist[num_request] = utime_i;
            gsl_vector_set (&distlist.vector, num_request, mdist);
            num_request++;
        }
#else
        double xc21_dist2 = x_cjci.mu[0]*x_cjci.mu[0] + x_cjci.mu[1]*x_cjci.mu[1] + x_cjci.mu[2]*x_cjci.mu[2];
        int64_t dt = utime_j - utime_i;
        //if (num_request == 0) printf ("dist = %g dt = %"PRId64"\n", xc21_dist2, dt);

        double range_thresh2 = COVRQ_DIST * COVRQ_DIST;
        bool dt_check_pass = true;

        // when uncertain checkbox is on from the viewer: map merging.
        if (tdata->uncertain_prior)
        {
            range_thresh2 = 10.0 * 10.0;            // larger range thresh
            if (dt < 60E6) dt_check_pass = false;   // dt should be large for map merging
        }

        //printf ("mu for pair (%"PRId64",%"PRId64")\n", utime_i, utime_j);
        if ((xc21_dist2 < range_thresh2) && dt_check_pass && (num_request < MAX_NUM_TO_RQ_COV))
        {
            // request again with covariance
            //printf ("re-requesting %"PRId64",%"PRId64"\n", utime_i, utime_j);
            rq_cov_utimelist[num_request] = utime_i;
            gsl_vector_set (&distlist.vector, num_request, sqrt (xc21_dist2));
            num_request++;
        }
#endif
    } // for each x_lci

    //printf ("mindist = %g\n", mindist);
    if (num_request > 0)
    {
        // sort based upon distance
        gsl_permutation *perm = gsl_permutation_alloc (MAX_NUM_TO_RQ_COV);
        gsl_sort_vector_index (perm, &distlist.vector);

        // request again
        se_request_state_t *se_rq_st = calloc (1, sizeof(*se_rq_st));
        se_rq_st->state_type = SE_REQUEST_STATE_T_COV_RIGHTCOL | SE_REQUEST_STATE_T_POSE;
        se_rq_st->n = num_request+1;
        se_rq_st->variables = malloc (se_rq_st->n * sizeof (se_rq_st->variables));
        for (size_t i=0; i<num_request; i++)
        {
            se_rq_st->variables[i] = rq_cov_utimelist[perm->data[i]];
        }
        se_rq_st->variables[num_request] = utime_j;
        se_request_state_t_publish (shm->lcm, SE_REQUEST_ST_CHANNEL, se_rq_st);
        se_request_state_t_destroy (se_rq_st);

        gsl_permutation_free (perm);
    }
}

static void
se_return_state_t_callback (const lcm_recv_buf_t *buf, const char *channel,
                            const se_return_state_t *msg, void *user)
{
    thread_data_t *tdata = user;

    if ((msg->state_type & SE_REQUEST_STATE_T_POSE) &&
            (msg->state_type & SE_REQUEST_STATE_T_COV_RIGHTCOL))
    {

        // CASE 1: return state == mu+cov: propose links
        if (msg->n == 1)
        {
            int64_t utime_j = msg->timestamps[msg->n-1];
            printf ("cov return: %d nodes returned to be just added - %"PRId64"\n", msg->n, utime_j);

            // remember the most recent block diagonal
            double *cov = msg->covariance;
            gsl_matrix_view cov_rr = gsl_matrix_view_array (cov, 6, 6);
            gsl_matrix *S_vrr = gslu_matrix_clone (&cov_rr.matrix);
            g_hash_table_insert (tdata->S_ii, gu_dup (&utime_j, sizeof utime_j), S_vrr);
            tdata->S_recent = S_vrr;    // passing the pointer
        }
        else
            _informative_link_hypothesis (msg, tdata);

    }
    else if (msg->state_type & SE_REQUEST_STATE_T_POSE)
    {
        // CASE 2: return state == mu only: request again for cov
        _sort_cov_request (msg, tdata);
    }
    else if (msg->state_type & SE_REQUEST_STATE_T_COV_BLOCK)
    {
        // CASE 3: return state == cov block only: when a graph is loaded
        double *cov = msg->covariance;
        size_t n_nodes = msg->m / 36;
        for (size_t i=0; i<n_nodes; i++)
        {
            int64_t utime = msg->timestamps[i];
            gsl_matrix_view cov_rr = gsl_matrix_view_array (cov+36*i, 6, 6);
            gsl_matrix *S_vrr = gslu_matrix_clone (&cov_rr.matrix);
            g_hash_table_insert (tdata->S_ii, gu_dup (&utime, sizeof utime), S_vrr);
        }

        tdata->initialized = true;
        printf ("Done loading %zu nodes. Ready!\n", n_nodes);
    }
}


static void
perllcm_isam_cmd_t_ack_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const perllcm_isam_cmd_t *msg, void *user )
{
    thread_data_t *tdata = user;

    // SAVE via local file
    if (msg->mode == PERLLCM_ISAM_CMD_T_MODE_SAVE)
    {
        if (msg->savepath)
        {
            vanu_isam_save_rtvan_appended (msg->savepath, tdata->camoffset_list, tdata->x_vc, tdata->S_ii);
        }
    }

    // LOAD
    size_t n_nodes = 0;
    if (msg->mode == PERLLCM_ISAM_CMD_T_MODE_LOAD)
    {
        if (msg->graphfile)
        {
            shm->utimelist = vanu_load_isam_graph (msg->graphfile, shm->utimelist, tdata->camoffset_list,
                                                   tdata->x_vc, tdata->S_ii, &n_nodes);

            if (shm->utimelist && n_nodes != 0)
            {
                // quickly check if we can load all feat+ppm from logged folder first
                size_t valid_node = 0;
                for (size_t i=0; i<n_nodes; i++)
                {
                    int64_t *utime_i = g_slist_nth_data (shm->utimelist, i);
                    IplImage *img_warp = vis_cvu_iplimg_load (shm->logdir, (*utime_i));
                    if (img_warp)
                    {
                        valid_node++;
                        cvReleaseImage (&img_warp);
                    }
                    else
                        printf ("Failed to load image for thumbnailing %"PRId64"\n", *utime_i);
                }

                // request S_ii
                se_request_state_t *se_rq_st = calloc (1, sizeof (*se_rq_st));
                se_rq_st->state_type = SE_REQUEST_STATE_T_COV_BLOCK;
                se_rq_st->n = valid_node;
                se_rq_st->variables = malloc (se_rq_st->n * sizeof (*se_rq_st->variables));

                /*for (size_t i=0; i<valid_node;) {
                  GList *utime_event = g_list_nth (shm->utimelist, i);
                  int64_t *utime_i = utime_event->data;

                  // prepare image cache
                  IplImage *img_warp = vis_cvu_iplimg_load (shm->logdir, (*utime_i));
                  if (img_warp) {
                  se_rq_st->variables[i] = (*utime_i);
                  cache_push (shm->imgcache, (*utime_i), cvCloneImage (img_warp));

                  // publish for the sample images (TODO: implement dx or step)
                  double scale = 0.25;
                  CvSize size = {img_warp->width*scale, img_warp->height*scale};
                  IplImage *img_tn = cvCreateImage (size, img_warp->depth, img_warp->nChannels);
                  cvResize (img_warp, img_tn, CV_INTER_LINEAR);
                  bot_core_image_t *botimg = vis_iplimage_to_botimage_copy (img_tn);
                  botimg->utime = (*utime_i);
                  bot_core_image_t_publish (shm->lcm, "PROSILICA_M", botimg);

                  bot_core_image_t_destroy (botimg);
                  cvReleaseImage (&img_warp);
                  cvReleaseImage (&img_tn);
                  i++;
                  }
                  else {
                  // cannot load the feat and ppm from last mission
                  free (utime_event->data);
                  shm->utimelist = g_list_delete_link (shm->utimelist, utime_event);
                  }
                  }*/
                se_request_state_t_publish (shm->lcm, SE_REQUEST_ST_CHANNEL, se_rq_st);
                se_request_state_t_destroy (se_rq_st);
                n_nodes = valid_node;
                printf ("requested state for %zu nodes\n", valid_node);
            }
            else
                printf ("ERROR: loaded null utimelist!\n");
        }
    }
}

static void
perllcm_isam_cmd_t_cmd_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const perllcm_isam_cmd_t *msg, void *user )
{
    thread_data_t *tdata = user;

    if (msg->mode == PERLLCM_ISAM_CMD_T_MODE_START)
    {
        shm->active = true;
    }
    else if (msg->mode == PERLLCM_ISAM_CMD_T_MODE_WAIT)
    {
        shm->active = false;
    }
    else if (msg->mode == PERLLCM_ISAM_CMD_T_MODE_SAVE)   // SAVE via lcm
    {
        if (msg->savepath)
        {
            char graph_path[MAXPATHLEN];
            snprintf (graph_path, strlen (msg->savepath)+12, "%s/graph.isam\n", msg->savepath);

            printf ("-- Saving to %s ...\n", graph_path);
            // open up a file to write, close it in se_save_isam_callback
            tdata->fid_save = fopen (graph_path, "w");
            if (tdata->fid_save != NULL)
            {
                tdata->save_mode = true;        // change flags to saving mode
                tdata->saving_done = false;     // i calloced but just in case
            }
            else
            {
                printf ("ERROR: cannot save %s.\n The directory does exist on the machine that you run rtvan!\n Instead saving to ./graph.isam",graph_path);

                char graph_path[MAXPATHLEN];
                snprintf (graph_path, 13, "./graph.isam\n");

                printf ("-- Saving to %s ...\n", graph_path);
                // open up a file to write, close it in se_save_isam_callback
                tdata->fid_save = fopen (graph_path, "w");
                if (tdata->fid_save != NULL)
                {
                    tdata->save_mode = true;        // change flags to saving mode
                    tdata->saving_done = false;     // i calloced but just in case
                }
            }
        }
    }

}

static void
perllcm_van_options_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                const perllcm_van_options_t *msg, void *user)
{
    thread_data_t *tdata = user;
    tdata->uncertain_prior = msg->manual_corr;
    tdata->mdist_plink = msg->mdist_plink;

    if (!msg->manual_corr)
        shm->imgstep = 1;
    else
        shm->imgstep = tdata->imgstep_cfg;

}

static int64_t
find_closest_utime (int64_t time_given, GSList *utimelist)
{
    int64_t utime_img = 0;
    double min_dt = GSL_POSINF;

    size_t utimelistlen = g_slist_length (utimelist);
    for (size_t i=0; i<utimelistlen; i++)
    {
        int64_t *utime_i = g_slist_nth_data (utimelist, i);
        double dt = abs((*utime_i) - time_given);
        if (dt < min_dt)
        {
            utime_img = *utime_i;
            min_dt = dt;
        }
    }

    return utime_img;
}

static void
se_goto_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                    const se_goto_t *msg, void *user)
{
    //thread_data_t *tdata = user;

    // load the image at utime target
    int64_t utime_target = find_closest_utime (msg->utime_target, shm->utimelist);
    IplImage *img_target = vis_cvu_iplimg_load (shm->logdir, utime_target);
    printf ("target =%"PRId64" and imgutime=%"PRId64"\n", msg->utime_target, utime_target);

    if (img_target)
    {
        bot_core_image_t *botimg = vis_iplimage_to_botimage_copy (img_target);
        botimg->utime = utime_target;
        bot_core_image_t_publish (shm->lcm, VAN_PLOT_CAM_TARGET, botimg);
        bot_core_image_t_destroy (botimg);
        cvReleaseImage (&img_target);
    }
    else
    {
        // loading error OR the target node is not related to images
        printf ("ERROR: loading target image from disk (%"PRId64")\n", msg->utime_target);
    }
}

static void
se_save_isam_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                         const se_save_isam_t *msg, void *user)
{
    thread_data_t *tdata = user;

    if (!tdata->save_mode)
        return;

    if (msg->type == SE_SAVE_ISAM_T_TYPE_DONE)
    {
        tdata->save_mode = false;
        fclose (tdata->fid_save);

        printf ("-- isam result has been saved to graph.isam\n");
    }
    else
    {
        // save this line
        se_save_isam_t *isam_line = se_save_isam_t_copy (msg);
        vanu_isam_save_rtvan_appended_via_lcm (tdata->fid_save, isam_line, tdata->camoffset_list, tdata->x_vc, tdata->S_ii);
        se_save_isam_t_destroy (isam_line);
    }
}

gpointer
secam_thread (gpointer user)
{
    printf ("Spawning\n");

    // se_link_thread state
    thread_data_t *tdata = calloc (1, sizeof (*tdata));

    tdata->x_vc             = g_hash_table_new_full (&gu_int64_hash, &gu_int64_equal, &free, &perllcm_pose3d_t_destroy_wrapper);
    tdata->S_ii             = g_hash_table_new_full (&gu_int64_hash, &gu_int64_equal, &free, &gslu_matrix_free_wrapper);
    tdata->camoffset_list   = g_hash_table_new_full (&gu_int64_hash, &gu_int64_equal, &free, &free);
    tdata->bowE             = g_hash_table_new_full (&gu_int64_hash, &gu_int64_equal, &free, &free);
    tdata->plist_manual     = g_hash_table_new_full (&gu_int64_hash, &gu_int64_equal, &free, &perllcm_plink_t_destroy_wrapper);
    tdata->S_recent         = NULL;
    tdata->Ig_thresh = 0.0;
    bot_param_get_double (shm->param, "rtvan.link_thread.Ig_thresh", &tdata->Ig_thresh);
    tdata->S_L_thresh = 0.0;
    bot_param_get_double (shm->param, "rtvan.saliency_thread.S_L_thresh", &tdata->S_L_thresh);

    tdata->use_sal_add_node = false;
    bot_param_get_boolean (shm->param, "rtvan.link_thread.use_sal_add_node", (int*)&tdata->use_sal_add_node);

    // rtvan internal subscriptions
    perllcm_van_feature_collection_t_subscription_t *perllcm_van_feature_collection_t_sub =
        perllcm_van_feature_collection_t_subscribe (shm->lcm, VAN_FEATURE_COLLECTION_CHANNEL, &perllcm_van_feature_collection_t_callback, tdata);

    perllcm_van_options_t_subscription_t *perllcm_van_options_t_sub =
        perllcm_van_options_t_subscribe (shm->lcm, VAN_OPTIONS_CHANNEL, &perllcm_van_options_t_callback, tdata);

    perllcm_pose3d_t_subscription_t *perllcm_pose3d_t_x_vc_subUw =
        perllcm_pose3d_t_subscribe (shm->lcm, VAN_V2C_POSE_UW_CHANNEL, &perllcm_pose3d_t_x_vc_uw_callback, tdata);
    perllcm_pose3d_t_subscription_t *perllcm_pose3d_t_x_vc_subPeri =
        perllcm_pose3d_t_subscribe (shm->lcm, VAN_V2C_POSE_PERI_CHANNEL, &perllcm_pose3d_t_x_vc_peri_callback, tdata);

    perllcm_pose3d_t_subscription_t *perllcm_pose3d_t_x_lc_subUw =
        perllcm_pose3d_t_subscribe (shm->lcm, VAN_CAMERA_POSE_UW_CHANNEL, &perllcm_pose3d_t_x_lc_uw_callback, tdata);
    perllcm_pose3d_t_subscription_t *perllcm_pose3d_t_x_lc_subPeri =
        perllcm_pose3d_t_subscribe (shm->lcm, VAN_CAMERA_POSE_PERI_CHANNEL, &perllcm_pose3d_t_x_lc_peri_callback, tdata);

    // DATA lcm subscriptions
    hauv_bs_rnv_2_t_subscription_t *hauv_bs_rnv_2_t_sub =
        hauv_bs_rnv_2_t_subscribe (shm->lcm, "HAUV_BS_RNV_2", &hauv_bs_rnv_2_t_callback, tdata);

    bot_core_image_t_subscription_t *bot_core_image_t_subUw =
        bot_core_image_t_subscribe (shm->lcm, shm->bot_core_image_t_channelUw, &bot_core_image_t_callback, tdata);
    bot_core_image_t_subscription_t *bot_core_image_t_subPeri =
        bot_core_image_t_subscribe (shm->lcm, shm->bot_core_image_t_channelPeri, &bot_core_image_t_callback, tdata);

    // SESERVER lcm subscriptions
    se_return_state_t_subscription_t *se_return_state_t_sub =
        se_return_state_t_subscribe (shm->lcm, SE_RETURN_ST_CHANNEL, &se_return_state_t_callback, tdata);

    se_add_node_ack_t_subscription_t *se_add_node_ack_t_sub =
        se_add_node_ack_t_subscribe (shm->lcm, SE_ADD_MODE_ACK_CHANNEL, &se_add_node_ack_t_callback, tdata);

    perllcm_isam_cmd_t_subscription_t *perllcm_isam_cmd_t_ack_sub =
        perllcm_isam_cmd_t_subscribe (shm->lcm, SE_OPTION_ACK_CHANNEL, &perllcm_isam_cmd_t_ack_callback, tdata);

    perllcm_isam_cmd_t_subscription_t *perllcm_isam_cmd_t_cmd_sub =
        perllcm_isam_cmd_t_subscribe (shm->lcm, SE_OPTION_CMD_CHANNEL, &perllcm_isam_cmd_t_cmd_callback, tdata);

    se_goto_t_subscription_t *se_goto_t_sub =
        se_goto_t_subscribe (shm->lcm, SE_GOTO_CHANNEL, &se_goto_t_callback, tdata);

    se_save_isam_t_subscription_t *se_save_isam_t_sub =
        se_save_isam_t_subscribe (shm->lcm, SE_SAVE_ISAM_CHANNEL, &se_save_isam_t_callback, tdata);

    perllcm_van_saliency_t_subscription_t *perllcm_van_saliency_t_sub =
        perllcm_van_saliency_t_subscribe (shm->lcm, VAN_SALIENCY_CHANNEL, &perllcm_van_saliency_t_callback, tdata);

    // min and max image overlap percentage
    tdata->min_overlap = 0.0;
    bot_param_get_double (shm->param, "rtvan.link_thread.min_overlap", &tdata->min_overlap);
    tdata->max_overlap = 1.0;
    bot_param_get_double (shm->param, "rtvan.link_thread.max_overlap", &tdata->max_overlap);

    tdata->add_node_thresh = 0.1;
    bot_param_get_double (shm->param, "rtvan.link_thread.add_node_thresh", &tdata->add_node_thresh);

    // image process step
    tdata->imgcounter = 0;
    tdata->imgstep_cfg = shm->imgstep;
    tdata->batchstep = 20;
    bot_param_get_int (shm->param, "seserver.batchstep", &tdata->batchstep);

    // manual plink provided?
    tdata->nplist_manual = 0;
    bot_param_get_int (shm->param, "rtvan.link_thread.use_manual_plinks", &tdata->nplist_manual);
    if (tdata->nplist_manual > 0)
    {
        const char *plink_manual_fname = "./plink_manual.list";
        //tdata->nplist_manual = vanu_load_manual_plink (plink_manual_fname, tdata->plist_manual);
        tdata->nplist_manual = vanu_load_manual_plink_wo_prior (plink_manual_fname, tdata->plist_manual);
    }

    while (!shm->done)
    {
        struct timeval timeout =
        {
            .tv_sec = 0,
            .tv_usec = 500000,
        };
        lcmu_handle_timeout (shm->lcm, &timeout);
    }

    // clean up: unsubscribe
    perllcm_van_feature_collection_t_unsubscribe (shm->lcm, perllcm_van_feature_collection_t_sub);
    perllcm_van_options_t_unsubscribe (shm->lcm, perllcm_van_options_t_sub);
    perllcm_pose3d_t_unsubscribe (shm->lcm, perllcm_pose3d_t_x_vc_subUw);
    perllcm_pose3d_t_unsubscribe (shm->lcm, perllcm_pose3d_t_x_vc_subPeri);
    perllcm_pose3d_t_unsubscribe (shm->lcm, perllcm_pose3d_t_x_lc_subUw);
    perllcm_pose3d_t_unsubscribe (shm->lcm, perllcm_pose3d_t_x_lc_subPeri);
    perllcm_van_saliency_t_unsubscribe (shm->lcm, perllcm_van_saliency_t_sub);
    hauv_bs_rnv_2_t_unsubscribe (shm->lcm, hauv_bs_rnv_2_t_sub);
    bot_core_image_t_unsubscribe (shm->lcm, bot_core_image_t_subUw);
    bot_core_image_t_unsubscribe (shm->lcm, bot_core_image_t_subPeri);

    se_return_state_t_unsubscribe (shm->lcm, se_return_state_t_sub);
    se_add_node_ack_t_unsubscribe (shm->lcm, se_add_node_ack_t_sub);
    perllcm_isam_cmd_t_unsubscribe (shm->lcm, perllcm_isam_cmd_t_ack_sub);
    perllcm_isam_cmd_t_unsubscribe (shm->lcm, perllcm_isam_cmd_t_cmd_sub);
    se_goto_t_unsubscribe (shm->lcm, se_goto_t_sub);
    se_save_isam_t_unsubscribe (shm->lcm, se_save_isam_t_sub);

    // clean up: history in GHash and GList
    g_hash_table_unref (tdata->x_vc);               // free vehicle to sensor xform
    g_hash_table_unref (tdata->S_ii);               // free block diagonal covariance matrix
    g_hash_table_unref (tdata->camoffset_list);     // free camoffset list
    g_hash_table_unref (tdata->plist_manual);               // free vehicle to sensor xform
    g_slist_foreach (tdata->plinklist, &glist_destroyer, NULL);
    g_slist_free (shm->utimelist);
    g_slist_free (tdata->plinklist);

    printf ("Exiting\n");
    g_thread_exit (0);
    return NULL;
}
