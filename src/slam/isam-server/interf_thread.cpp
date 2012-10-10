#include <iostream>
#include <sys/param.h>

// external linking req'd
#include <glib.h>

// lcmtypes
#include "perls-lcmtypes/perllcm_isam_init_t.h"
#include "perls-lcmtypes/perllcm_isam_add_node_t.h"
#include "perls-lcmtypes/perllcm_isam_add_node_ack_t.h"
#include "perls-lcmtypes/perllcm_isam_vlink_t.h"

#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"

#include "interf_thread.h"
#include "isam_server.h"    


// callbacks
//------------------------------------------------------------------------

static void
init_t_callback (const lcm_recv_buf_t *rbuf, const char * channel,
                 const perllcm_isam_init_t * msg, void *user)
{
    interf_tdata_t *tdata = (interf_tdata_t *)user;
    
    if (tdata->verbose & VERB_LCM)
        std::cout << "[interf]\tReceived init_t message for utime = " << msg->utime << std::endl;

    queue_element_t *qe = (queue_element_t *)calloc (1, sizeof (*qe));
    qe->msg = perllcm_isam_init_t_copy (msg);
    if (msg->utime > 0)
        qe->utime = msg->utime;
    else
        qe->utime = timestamp_now ();
    qe->type = QE_INIT;

    g_async_queue_push (tdata->gq, qe);
}

static void
add_node_t_callback (const lcm_recv_buf_t *rbuf, const char * channel,
                     const perllcm_isam_add_node_t *msg, void * user)
{
    interf_tdata_t *tdata = (interf_tdata_t *)user;
    
    if (tdata->verbose & VERB_LCM)
        std::cout << "[interf]\tReceived add node message for utime = " << msg->utime << std::endl;

    // add to queue
    queue_element_t *qe = (queue_element_t *)calloc (1, sizeof (*qe));
    qe->msg = perllcm_isam_add_node_t_copy (msg);
    if (msg->utime > 0)
        qe->utime = msg->utime;
    else
        qe->utime = timestamp_now ();

    qe->type = QE_NODE;
    
    g_async_queue_push (tdata->gq, qe);
}

static void
vlink_t_callback (const lcm_recv_buf_t *rbuf, const char * channel,
                  const perllcm_isam_vlink_t *vlink, void * user)
{
    interf_tdata_t *tdata = (interf_tdata_t *)user;

    if (tdata->verbose & VERB_LCM)
        std::cout << "[interf]\tReceived vlink message for utime = " << vlink->utime << std::endl;

    if (!vlink->accept)
        return;
    
    // add to queue
    queue_element_t *qe = (queue_element_t *)calloc (1, sizeof (*qe));
    qe->msg = perllcm_isam_vlink_t_copy (vlink);
    if (vlink->utime > 0)
        qe->utime = vlink->utime;
    else
        qe->utime = timestamp_now ();
    qe->type = QE_VLINK;
    
    g_async_queue_push (tdata->gq, qe);

    //only send out an acknowledgement for factors and priors
    if (vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_PRIOR ||
        vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_POSE3D ||
        vlink->link_type == PERLLCM_ISAM_VLINK_T_LINK_POSE2D) {
        perllcm_isam_add_node_ack_t *ack = new perllcm_isam_add_node_ack_t ();
        ack->sensor_id = vlink->creator_of_id2;
        ack->utime = vlink->id2;
        perllcm_isam_add_node_ack_t_publish (tdata->lcm, tdata->chname_add_node_ack, ack);
        delete ack;
    }
}

static void
request_state_t_callback (const lcm_recv_buf_t *rbuf, const char * channel,
                          const perllcm_isam_request_state_t * msg, void * user)
{
    interf_tdata_t *tdata = (interf_tdata_t *)user;
    
    if (tdata->verbose & VERB_LCM)
        std::cout << "[interf]\tReceived state request for utime = " << msg->utime << std::endl;
    
    queue_element_t *qe = (queue_element_t *)calloc (1, sizeof (*qe));
    qe->msg = perllcm_isam_request_state_t_copy (msg);
    if (msg->utime > 0)
        qe->utime = msg->utime;
    else
        qe->utime = timestamp_now ();
    qe->type = QE_STATE;
    
    g_async_queue_push (tdata->gq, qe);

}

static void
isam_cmd_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                     const perllcm_isam_cmd_t *cmd, void *user)
{
    interf_tdata_t *tdata = (interf_tdata_t *)user;
    
    if (tdata->verbose & VERB_LCM)
        std::cout << "[interf]\tReceived cmd message for utime = " << cmd->utime << std::endl;
    
    queue_element_t *qe = (queue_element_t *)calloc (1, sizeof (*qe));
    qe->msg = perllcm_isam_cmd_t_copy (cmd);
    if (cmd->utime > 0)
        qe->utime = cmd->utime;
    else
        qe->utime = timestamp_now ();
    qe->type = QE_CMD;
    
    g_async_queue_push (tdata->gq, qe);    
}

gpointer
interf_thread (gpointer user)
{
    std::cout << "[interf]\tSpawning" << std::endl;
    
    interf_tdata_t *tdata = (interf_tdata_t *)user;
    
    // identify that we are using queue in this thread
    g_async_queue_ref (tdata->gq);
    
    // subscribe lcm channels
    perllcm_isam_init_t_subscription_t *perllcm_isam_init_t_sub = 
        perllcm_isam_init_t_subscribe (tdata->lcm, tdata->chname_init_isam, &init_t_callback, tdata);

    perllcm_isam_add_node_t_subscription_t *perllcm_isam_add_node_t_sub = 
        perllcm_isam_add_node_t_subscribe (tdata->lcm, tdata->chname_add_node, &add_node_t_callback, tdata);

    perllcm_isam_vlink_t_subscription_t *perllcm_isam_vlink_t_sub = 
        perllcm_isam_vlink_t_subscribe (tdata->lcm, tdata->chname_vlink, &vlink_t_callback, tdata);
        
    perllcm_isam_request_state_t_subscription_t *perllcm_isam_request_state_t_sub = 
        perllcm_isam_request_state_t_subscribe (tdata->lcm, tdata->chname_request_st, &request_state_t_callback, tdata);

    perllcm_isam_cmd_t_subscription_t *perllcm_isam_cmd_t_sub = 
        perllcm_isam_cmd_t_subscribe (tdata->lcm, tdata->chname_cmd, &isam_cmd_t_callback, tdata);

    while (!*(tdata->done)) {
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 500000;
        lcmu_handle_timeout (tdata->lcm, &timeout);
    }

    // clean up
    perllcm_isam_init_t_unsubscribe (tdata->lcm, perllcm_isam_init_t_sub);
    perllcm_isam_add_node_t_unsubscribe (tdata->lcm, perllcm_isam_add_node_t_sub);
    perllcm_isam_vlink_t_unsubscribe (tdata->lcm, perllcm_isam_vlink_t_sub);
    perllcm_isam_request_state_t_unsubscribe (tdata->lcm, perllcm_isam_request_state_t_sub);
    perllcm_isam_cmd_t_unsubscribe (tdata->lcm, perllcm_isam_cmd_t_sub);

    g_async_queue_unref (tdata->gq);

    std::cout << "[interf]\tExiting" << std::endl;
    g_thread_exit (0);
    return NULL;
}
