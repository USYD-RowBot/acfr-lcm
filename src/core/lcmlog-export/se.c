#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "perls-common/textread.h"

#include "lcmlog_export.h"
#include "se.h"

void
se_add_node_ack_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const se_add_node_ack_t *msg, void *user)
{

    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",            "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "node_type",        "%d",      msg->node_type);
    TEXTREAD_ADD_FIELD (tr, "sensor_id",        "%d",      msg->sensor_id);

    for (int i = 0; i < 6; i++) {
        char tmp[128];
        snprintf (tmp, sizeof tmp, "mu(:,%d)", i+1);
        TEXTREAD_ADD_FIELD (tr, tmp, "%.15f", msg->mu[i]);
    }

    for (int i = 0; i < 36; i++) {
        char tmp[128];
        snprintf (tmp, sizeof tmp, "cov(:,%d)", i+1);
        TEXTREAD_ADD_FIELD (tr, tmp, "%.15f", msg->cov[i]);
    }

    textread_stop (tr);

}

void
se_add_node_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const se_add_node_t *msg, void *user)
{
    // TODO: implement me
}

void
se_goto_state_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                         const se_goto_state_t *msg, void *user)
{
    // TODO: implement me
}

void
se_goto_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                   const se_goto_t *msg, void *user)
{

    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime_target",      "%"PRId64, msg->utime_target);
    TEXTREAD_ADD_FIELD (tr, "mode",              "%d",      msg->mode);
    textread_stop (tr);
}

void
se_info_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                   const se_info_t *msg, void *user)
{
    // TODO: implement me
}

void
se_option_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                     const se_option_t *msg, void *user)
{
    // TODO: implement me
}

void
se_pose_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                   const se_pose_t *msg, void *user)
{
    // TODO: implement me
}

void
se_propose_link_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const se_propose_link_t *msg, void *user)
{
    // TODO: implement me
}

void
se_publish_link_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const se_publish_link_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    int max_n_len = 5;  // camera = 5, sonar = 3, failed = 0;

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime1",            "%"PRId64, msg->utime1);
    TEXTREAD_ADD_FIELD (tr, "utime2",            "%"PRId64, msg->utime2);
    TEXTREAD_ADD_FIELD (tr, "n",                 "%d",      msg->n);
    for (int i = 0; i < max_n_len; i++) {
        char tmp[128];
        snprintf (tmp, sizeof tmp, "measurement(:,%d)", i+1);
        TEXTREAD_ADD_FIELD (tr, tmp, "%.15f", i < msg->n ? msg->measurement[i] : 0.0);
    }
    TEXTREAD_ADD_FIELD (tr, "n2",                "%d",      msg->n2);
    for (int i = 0; i < max_n_len*max_n_len; i++) {
        char tmp[128];
        snprintf (tmp, sizeof tmp, "sigma(:,%d)", i+1);
        TEXTREAD_ADD_FIELD (tr, tmp, "%.15f", i < msg->n2 ? msg->sigma[i] : 0.0);
    }

    TEXTREAD_ADD_FIELD (tr, "link_type",        "%d",      msg->link_type);
    TEXTREAD_ADD_FIELD (tr, "publisher_id",     "%d",      msg->publisher_id);
    TEXTREAD_ADD_FIELD (tr, "sensor_id",        "%d",      msg->sensor_id);
    TEXTREAD_ADD_FIELD (tr, "link_id",          "%d",      msg->link_id);
    TEXTREAD_ADD_FIELD (tr, "accept",           "%d",      msg->accept);
    TEXTREAD_ADD_FIELD (tr, "accept_code",      "%d",      msg->accept_code);
    //    char*      comment; 
    textread_stop (tr);

}

void
se_request_state_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                            const se_request_state_t *msg, void *user)
{
    // TODO: implement me
}

void
se_return_state_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const se_return_state_t *msg, void *user)
{

    // TODO: this is not the best way... how could i get the last return_state?
    // ayoung kim 2011.10.18
    int max_n_pose = 2000;
    if (msg->state_type == SE_RETURN_STATE_T_POSE && msg->n > 1985) {
        // store mu for now
        lcmlog_export_t *lle = user;
        textread_t *tr = lle_get_textread (lle, channel);

        textread_start (tr);
        TEXTREAD_ADD_FIELD (tr, "utime",            "%"PRId64, msg->utime);
        TEXTREAD_ADD_FIELD (tr, "n",                "%d",      msg->n);

        char tmp[128];
        for (int i = 0; i < max_n_pose; i++) {
            for (int j = 0; j < 6; j++) {
                snprintf (tmp, sizeof tmp, "mu(%d,%d)", j+1, i+1);

                if (i < msg->n) {
                    se_pose_t x_lv = msg->poses[i]; 
                    TEXTREAD_ADD_FIELD (tr, tmp, "%.15f", x_lv.mu[j]);
                }
                else {
                    TEXTREAD_ADD_FIELD (tr, tmp, "%.15f", 0.0);
                }
            }
        }

        TEXTREAD_ADD_FIELD (tr, "utime",            "%"PRId64, msg->utime);
        TEXTREAD_ADD_FIELD (tr, "n",                "%d",      msg->n);

        for (int i = 0; i < max_n_pose; i++) {
            snprintf (tmp, sizeof tmp, "time(:,%d)", i+1);
            TEXTREAD_ADD_FIELD (tr, tmp, "%"PRId64, i < msg->n ? msg->timestamps[i] : 0);
        }

        //int32_t    m;
        //double     *covariance;
        textread_stop (tr);
    }
}

void
se_save_isam_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                        const se_save_isam_t *msg, void *user)
{
    // TODO: implement me
}
  
void
se_save_waypoint_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                            const se_save_waypoint_t *msg, void *user)
{
    // TODO: implement me
}

