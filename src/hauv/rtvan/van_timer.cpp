#include <map>
#include <iostream>

#include "perls-common/error.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"

#include <lcm/lcm.h>
#include "perls-lcmtypes/bot_core_image_t.h"

#include "perls-lcmtypes/perllcm_van_feature_collection_t.h"
#include "perls-lcmtypes/perllcm_van_saliency_t.h"
#include "perls-lcmtypes/perllcm_van_plink_t.h"

#include "perls-lcmtypes/se_add_node_ack_t.h"
#include "perls-lcmtypes/se_add_node_t.h"
#include "perls-lcmtypes/perllcm_isam_cmd_t.h"
#include "perls-lcmtypes/se_publish_link_t.h"
#include "perls-lcmtypes/se_request_state_t.h"
#include "perls-lcmtypes/se_return_state_t.h"

#include "shared_memory.h"

#define IMAGE_T_CHANNEL "PROSILICA_M"

typedef struct _data data_t;
struct _data {
    std::map <int64_t, int64_t> utime_img;
    std::map <int64_t, int64_t> utime_cvsurf;
    std::map <int64_t, int64_t> utime_feat;
    std::map <int64_t, int64_t> utime_sal;

    std::map <int64_t, int64_t> utime_add_node;
    std::map <int64_t, int64_t> utime_add_node_ack;
    std::map <int64_t, int64_t> utime_murq;
    std::map <int64_t, int64_t> utime_murt;
    std::map <int64_t, int64_t> utime_covrq;
    std::map <int64_t, int64_t> utime_covrt;
    std::map <int64_t, int64_t> utime_plink;
    std::map <int64_t, int64_t> utime_vlink;
};

static void
bot_core_image_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                           const bot_core_image_t *img, void *user)
{
    data_t *dt_data = (data_t*) user;
    dt_data->utime_img[img->utime] = timestamp_now ();

    std::cout << "timestamp on img " << img->utime << std::endl;
}

static void
perllcm_van_feature_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                const perllcm_van_feature_t *f, void *user)
{
    if (f->attrtype != PERLLCM_VAN_FEATURE_T_ATTRTYPE_CVSURF &&
        f->attrtype != PERLLCM_VAN_FEATURE_T_ATTRTYPE_SURFGPU)
        return;

    data_t *dt_data = (data_t*) user;
    dt_data->utime_cvsurf[f->utime] = timestamp_now ();
}

static void
perllcm_van_feature_collection_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                           const perllcm_van_feature_collection_t *fc, void *user)
{
    data_t *dt_data = (data_t*) user;
    dt_data->utime_feat[fc->utime] = timestamp_now ();
}

// saliency thread timer
static void
perllcm_van_saliency_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const perllcm_van_saliency_t *sal, void *user)
{
    data_t *dt_data = (data_t*) user;
    dt_data->utime_sal[sal->utime] = timestamp_now ();
}

// secam thread timer
static void
se_add_node_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                        const se_add_node_t *addnode, void *user)
{
    data_t *dt_data = (data_t*) user;
    dt_data->utime_add_node[addnode->utime] = timestamp_now ();
}

static void
se_add_node_ack_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                            const se_add_node_ack_t *ack, void *user)
{
    data_t *dt_data = (data_t*) user;
    dt_data->utime_add_node_ack[ack->utime] = timestamp_now ();
}

static void
se_request_state_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                             const se_request_state_t *strq, void *user)
{
    data_t *dt_data = (data_t*) user;
    int64_t utime = strq->variables[strq->n-1];

    if ((strq->state_type & SE_REQUEST_STATE_T_POSE) && 
        (strq->state_type & SE_REQUEST_STATE_T_COV_RIGHTCOL)) {
        dt_data->utime_covrq[utime] = timestamp_now ();
    }
    else if (strq->state_type & SE_REQUEST_STATE_T_POSE) {
        dt_data->utime_murq[utime] = timestamp_now ();
    }
}

static void
se_return_state_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                            const se_return_state_t *strt, void *user)
{
    data_t *dt_data = (data_t*) user;

    int64_t utime = strt->timestamps[strt->n-1];


    if ((strt->state_type & SE_REQUEST_STATE_T_POSE) && 
        (strt->state_type & SE_REQUEST_STATE_T_COV_RIGHTCOL)) {
        dt_data->utime_covrt[utime] = timestamp_now ();
    }
    else if (strt->state_type & SE_REQUEST_STATE_T_POSE) {
        dt_data->utime_murt[utime] = timestamp_now ();
    }
}

static void
perllcm_van_plink_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                              const perllcm_van_plink_t *plink, void *user)
{
    data_t *dt_data = (data_t*) user;
    dt_data->utime_plink[plink->utime_j] = timestamp_now ();
}

// twoview thread timer
static void
se_publish_link_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                            const se_publish_link_t *vlink, void *user)
{
    data_t *dt_data = (data_t*) user;
    dt_data->utime_vlink[vlink->utime2] = timestamp_now ();
    std::cout << "timestamp on vlink " << vlink->utime2 << std::endl;
}

static void
perllcm_isam_cmd_t_cmd_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const perllcm_isam_cmd_t *msg, void *user )
{
    data_t *dt_data = (data_t*) user;

    // option_cmd is from viewer
    if (msg->mode == PERLLCM_ISAM_CMD_T_MODE_SAVE) {
        printf ("saving rtvan timing to ./timer.dat\n");

        FILE *stream = fopen ("./timer.dat", "w");
        if (!stream) {
            ERROR ("unable to create file ./timer.dat!");
        }

        std::map<int64_t, int64_t>::iterator it_imgutime;
        for (it_imgutime = dt_data->utime_img.begin(); it_imgutime != dt_data->utime_img.end(); it_imgutime++) {
            int64_t utime_id = it_imgutime->first;
            int64_t t_img = it_imgutime->second;
     
            if (t_img > 0) {
                int64_t t_feat    = (dt_data->utime_feat.find(utime_id)==dt_data->utime_feat.end())?0:dt_data->utime_feat[utime_id];
                int64_t t_sal     = (dt_data->utime_sal.find(utime_id)==dt_data->utime_sal.end())?0:dt_data->utime_sal[utime_id];
                int64_t t_addnode = (dt_data->utime_add_node.find(utime_id)==dt_data->utime_add_node.end())?0:dt_data->utime_add_node[utime_id];
                int64_t t_ack     = (dt_data->utime_add_node_ack.find(utime_id)==dt_data->utime_add_node_ack.end())?0:dt_data->utime_add_node_ack[utime_id];
                int64_t t_murq    = (dt_data->utime_murq.find(utime_id)==dt_data->utime_murq.end())?0:dt_data->utime_murq[utime_id];
                int64_t t_murt    = (dt_data->utime_murt.find(utime_id)==dt_data->utime_murt.end())?0:dt_data->utime_murt[utime_id];
                int64_t t_covrq   = (dt_data->utime_covrq.find(utime_id)==dt_data->utime_covrq.end())?0:dt_data->utime_covrq[utime_id];
                int64_t t_covrt   = (dt_data->utime_covrt.find(utime_id)==dt_data->utime_covrt.end())?0:dt_data->utime_covrt[utime_id];
                int64_t t_plink   = (dt_data->utime_plink.find(utime_id)==dt_data->utime_plink.end())?0:dt_data->utime_plink[utime_id];
                int64_t t_vlink   = (dt_data->utime_vlink.find(utime_id)==dt_data->utime_vlink.end())?0:dt_data->utime_vlink[utime_id];
                int64_t t_surf    = (dt_data->utime_cvsurf.find(utime_id)==dt_data->utime_cvsurf.end())?0:dt_data->utime_cvsurf[utime_id];

                fprintf (stream, "%ld\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\n", t_img, t_feat, t_sal, t_addnode, t_ack,
                         t_murq, t_murt, t_covrq, t_covrt, t_plink, t_vlink, t_surf);
                //printf ("%ld\t%ld\t%ld\t%ld\t%ld%ld\t%ld\t%ld\t%ld\t%ld\t%ld\n", t_img, t_feat, t_sal, t_addnode, t_ack,
                //                                                                          t_murq, t_murt, t_covrq, t_covrt, t_plink, t_vlink);
            }
        }

        fclose (stream);
    }
}


int
main (int argc, char *argv[])
{
    // lcm
    lcm_t *lcm = lcm_create (NULL);
    if (!lcm) {
        printf ("lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    }

    data_t *dt_data = new data_t();

    // lcm subscriptions
    bot_core_image_t_subscription_t *bot_core_image_t_sub = 
        bot_core_image_t_subscribe (lcm, IMAGE_T_CHANNEL, &bot_core_image_t_callback, dt_data);

    perllcm_van_feature_collection_t_subscription_t *perllcm_van_feature_collection_t_sub = 
        perllcm_van_feature_collection_t_subscribe (lcm, VAN_FEATURE_COLLECTION_CHANNEL, &perllcm_van_feature_collection_t_callback, dt_data);

    perllcm_van_feature_t_subscription_t *perllcm_van_feature_t_sub =
        perllcm_van_feature_t_subscribe (lcm, VAN_WORDS_CHANNEL, &perllcm_van_feature_t_callback, dt_data);

    perllcm_van_saliency_t_subscription_t *perllcm_van_saliency_t_sub =
        perllcm_van_saliency_t_subscribe (lcm, VAN_SALIENCY_CHANNEL, &perllcm_van_saliency_t_callback, dt_data);

    se_add_node_t_subscription_t *se_add_node_t_sub = 
        se_add_node_t_subscribe (lcm, SE_ADD_NODE_CHANNEL, &se_add_node_t_callback, dt_data);

    se_add_node_ack_t_subscription_t *se_add_node_ack_t_sub = 
        se_add_node_ack_t_subscribe (lcm, SE_ADD_MODE_ACK_CHANNEL, &se_add_node_ack_t_callback, dt_data);

    se_request_state_t_subscription_t *se_request_state_t_sub = 
        se_request_state_t_subscribe (lcm, SE_REQUEST_ST_CHANNEL, &se_request_state_t_callback, dt_data);

    se_return_state_t_subscription_t *se_return_state_t_sub = 
        se_return_state_t_subscribe (lcm, SE_RETURN_ST_CHANNEL, &se_return_state_t_callback, dt_data);

    perllcm_van_plink_t_subscription_t *perllcm_van_plink_t_sub = 
        perllcm_van_plink_t_subscribe (lcm, VAN_PLINK_CHANNEL, &perllcm_van_plink_t_callback, dt_data);

    se_publish_link_t_subscription_t *se_publish_link_t_sub =
        se_publish_link_t_subscribe (lcm, SE_VLINK_CHANNEL, &se_publish_link_t_callback, dt_data);

    perllcm_isam_cmd_t_subscription_t *perllcm_isam_cmd_t_cmd_sub = 
        perllcm_isam_cmd_t_subscribe (lcm, SE_OPTION_CMD_CHANNEL, &perllcm_isam_cmd_t_cmd_callback, dt_data);

    while (1) {
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 500000;
        lcmu_handle_timeout (lcm, &timeout);
    }

    // clean up
    bot_core_image_t_unsubscribe (lcm, bot_core_image_t_sub);

    perllcm_van_feature_collection_t_unsubscribe (lcm, perllcm_van_feature_collection_t_sub);
    perllcm_van_feature_t_unsubscribe (lcm, perllcm_van_feature_t_sub);
    perllcm_van_plink_t_unsubscribe (lcm, perllcm_van_plink_t_sub);
    perllcm_van_saliency_t_unsubscribe (lcm, perllcm_van_saliency_t_sub);

    se_add_node_ack_t_unsubscribe (lcm, se_add_node_ack_t_sub);
    se_add_node_t_unsubscribe (lcm, se_add_node_t_sub);
    perllcm_isam_cmd_t_unsubscribe (lcm, perllcm_isam_cmd_t_cmd_sub);
    se_publish_link_t_unsubscribe (lcm, se_publish_link_t_sub);    
    se_request_state_t_unsubscribe (lcm, se_request_state_t_sub);
    se_return_state_t_unsubscribe (lcm, se_return_state_t_sub);
}

