/* ================================================================
** remotehelm.[ch]
**
** Support functions for Ocean Server Remote Helm comms
** ================================================================
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include <bot_core/bot_core.h>

#include "perls-lcmtypes/senlcm_uvc_ack_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/lcm_util.h"
#include "perls-common/nmea.h"
#include "perls-common/units.h"
#include "perls-common/timestamp.h"

#include "remotehelm.h"

typedef struct _ack_t ack_t;
struct _ack_t {
    int msg_type;
    int status;
    int err_num;
    lcm_t *lcm;
};

static void
ack_callback (const lcm_recv_buf_t *rbuf, const char *channel,
              const senlcm_uvc_ack_t *msg, void *user)
{
    ack_t *ack = user;
    if (msg->msg_type == ack->msg_type) {
        ack->status = msg->status;
        ack->err_num = msg->err_num;
    }
}

static ack_t *
ack_init (int msg_type, BotParam *param)
{
    static char *ack_channel = NULL;
    if (!ack_channel) {
        ack_channel = lcmu_channel_get_os_conduit (param, LCMU_CHANNEL_OS_CONDUIT_ACK);
    }

    ack_t *ack = calloc (1, sizeof (*ack));
    ack->msg_type = msg_type;
    ack->status = -1;
    ack->lcm = lcm_create (NULL);
    if (!ack->lcm) {
        ERROR ("lcm_create() failed");
        return NULL;
    }
    senlcm_uvc_ack_t_subscribe (ack->lcm, ack_channel, &ack_callback, ack);
    return ack;
}

static void
ack_free (ack_t *ack)
{
    if (ack) {
        lcm_destroy (ack->lcm);
        free (ack);
    }
}

static int
ack_handle (ack_t *ack, const char *msg_type, struct timeval *timeout)
{
    if (!ack->lcm)
        return -1;

    int ret = 1;
    //int64_t start_listen = timestamp_now (), ack_duration = 0;
    while (ret > 0) {
        ret = lcmu_handle_timeout (ack->lcm, timeout);
        if (ack->status != -1)
            break;
    }
    //ack_duration = timestamp_now () - start_listen;
    //printf ("ACK REQUIRED %"PRId64" microsecs\n", ack_duration); 
    
    switch (ack->status) {
    case -1:
        fprintf (stderr, "Warning: timed out on %s ACK\n", msg_type);
        break;
    case 0:
        break;
    case 1:
    default:
        fprintf (stderr, "Warning: error on %s ACK [%d]\n", msg_type, ack->err_num);
    }
    return ack->status;
}


int
iver_rh_pub_ojw (lcm_t *lcm, const char *channel, senlcm_uvc_ojw_t *cmd, BotParam *param, struct timeval *timeout)
{
    senlcm_uvc_ojw_t_publish (lcm, channel, cmd);
    if (timeout) {
        ack_t *ack = ack_init (SENLCM_UVC_ACK_T_ACK_OJW, param);
        int ret = ack_handle (ack, "OJW", timeout);
        ack_free (ack);
        return ret;
    }
    else
        return 0;
}


int
iver_rh_pub_omp (lcm_t *lcm, const char *channel, senlcm_uvc_omp_t *cmd, BotParam *param, struct timeval *timeout)
{
    // bound check primitive commands
    if (cmd->yaw_top > 255) {
        fprintf (stderr, "Warning: clipping OMP yaw_top to 255 [%d]\n", cmd->yaw_top);
        cmd->yaw_top = 255;
    }
    if (cmd->yaw_top < 0) {
        fprintf (stderr, "Warning: clipping OMP yaw_top to 0 [%d]\n", cmd->yaw_top);
        cmd->yaw_top = 0;
    }
    if (cmd->yaw_bot > 255) {
        fprintf (stderr, "Warning: clipping OMP yaw_bot to 255 [%d]\n", cmd->yaw_bot);
        cmd->yaw_bot = 255;
    }
    if (cmd->yaw_bot < 0) {
        fprintf (stderr, "Warning: clipping OMP yaw_bot to 0 [%d]\n", cmd->yaw_bot);
        cmd->yaw_bot = 0;
    }
    if (cmd->pitch_left > 255) {
        fprintf (stderr, "Warning: clipping OMP pitch_left to 255 [%d]\n", cmd->pitch_left);
        cmd->pitch_left = 255;
    }
    if (cmd->pitch_left < 0) {
        fprintf (stderr, "Warning: clipping OMP pitch_left to 0 [%d]\n", cmd->pitch_left);
        cmd->pitch_left = 0;
    }
    if (cmd->pitch_right > 255) {
        fprintf (stderr, "Warning: clipping OMP pitch_right to 255 [%d]\n", cmd->pitch_right);
        cmd->pitch_right = 255;
    }
    if (cmd->pitch_right < 0) {
        fprintf (stderr, "Warning: clipping OMP pitch_right to 0 [%d]\n", cmd->pitch_right);
        cmd->pitch_right = 0;
    }

    senlcm_uvc_omp_t_publish (lcm, channel, cmd);
    if (timeout) {
        ack_t *ack = ack_init (SENLCM_UVC_ACK_T_ACK_OMP, param);
        int ret = ack_handle (ack, "OMP", timeout);
        ack_free (ack);
        return ret;
    }
    else
        return 0;
}


int
iver_rh_pub_omload (lcm_t *lcm, const char *channel, senlcm_uvc_omload_t *cmd, BotParam *param, struct timeval *timeout)
{
    senlcm_uvc_omload_t_publish (lcm, channel, cmd);
    if (timeout) {
        ack_t *ack = ack_init (SENLCM_UVC_ACK_T_ACK_OMLOAD, param);
        int ret = ack_handle (ack, "OMLOAD", timeout);
        ack_free (ack);
        return ret;
    }
    else
        return 0;
}


int
iver_rh_pub_omstart (lcm_t *lcm, const char *channel, senlcm_uvc_omstart_t *cmd, BotParam *param, struct timeval *timeout)
{
    senlcm_uvc_omstart_t_publish (lcm, channel, cmd);
    if (timeout) {
        ack_t *ack = ack_init (SENLCM_UVC_ACK_T_ACK_OMSTART, param);
        int ret = ack_handle (ack, "OMSTART", timeout);
        ack_free (ack);
        return ret;
    }
    else
        return 0;
}


int
iver_rh_pub_omstop (lcm_t *lcm, const char *channel, senlcm_uvc_omstop_t *cmd, BotParam *param, struct timeval *timeout)
{
    senlcm_uvc_omstop_t_publish (lcm, channel, cmd);
    if (timeout) {
        ack_t *ack = ack_init (SENLCM_UVC_ACK_T_ACK_OMSTOP, param);
        int ret = ack_handle (ack, "OMSTOP", timeout);
        ack_free (ack);
        return ret;
    }
    else 
        return 0;
}

int
iver_rh_pub_omw (lcm_t *lcm, const char *channel, senlcm_uvc_omw_t *cmd, BotParam *param, struct timeval *timeout)
{
    senlcm_uvc_omw_t_publish (lcm, channel, cmd);
    if (timeout) {
        ack_t *ack = ack_init (SENLCM_UVC_ACK_T_ACK_OMW, param);
        int ret = ack_handle (ack, "OMW", timeout);
        ack_free (ack);
        return ret;
    }
    else
        return 0;
}

int
iver_rh_pub_opos (lcm_t *lcm, const char *channel, senlcm_uvc_opos_t *cmd, BotParam *param, struct timeval *timeout)
{
    senlcm_uvc_opos_t_publish (lcm, channel, cmd);
    if (timeout) {
        ack_t *ack = ack_init (SENLCM_UVC_ACK_T_ACK_OPOS, param);
        int ret = ack_handle (ack, "OPOS", timeout);
        ack_free (ack);
        return ret;
    }
    else
        return 0;
}
