#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "perls-common/textread.h"

#include "lcmlog_export.h"
#include "acfrlcm.h"

void
acfrlcm_auv_acfr_nav_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                             const acfrlcm_auv_acfr_nav_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "latitude", "%f",      msg->latitude);
    TEXTREAD_ADD_FIELD (tr, "longitude", "%f",      msg->longitude);
    TEXTREAD_ADD_FIELD (tr, "depth", "%f",      msg->depth);
    TEXTREAD_ADD_FIELD (tr, "roll","%f",      msg->roll);
    TEXTREAD_ADD_FIELD (tr, "pitch","%f",      msg->pitch);
    TEXTREAD_ADD_FIELD (tr, "heading","%f",      msg->heading);
    TEXTREAD_ADD_FIELD (tr, "vx","%f",      msg->vx);
    TEXTREAD_ADD_FIELD (tr, "vy","%f",      msg->vy);
    TEXTREAD_ADD_FIELD (tr, "vz","%f",      msg->vz);
    textread_stop (tr);
}

void
acfrlcm_auv_path_command_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const acfrlcm_auv_path_command_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "goal_id",    "%d", msg->goal_id);
    TEXTREAD_ADD_FIELD (tr, "depth_mode",    "%"PRId8, msg->depth_mode);
    TEXTREAD_ADD_FIELD (tr, "waypoint(:,1)",    "%f", msg->waypoint[0]);
    TEXTREAD_ADD_FIELD (tr, "waypoint(:,2)",    "%f", msg->waypoint[1]);
    TEXTREAD_ADD_FIELD (tr, "waypoint(:,3)",    "%f", msg->waypoint[2]);
    TEXTREAD_ADD_FIELD (tr, "waypoint(:,4)",    "%f", msg->waypoint[3]);
    TEXTREAD_ADD_FIELD (tr, "waypoint(:,5)",    "%f", msg->waypoint[4]);
    TEXTREAD_ADD_FIELD (tr, "waypoint(:,6)",    "%f", msg->waypoint[5]);
    TEXTREAD_ADD_FIELD (tr, "waypoint(:,7)",    "%f", msg->waypoint[6]);
   
    textread_stop (tr);
}



void
acfrlcm_auv_path_response_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const acfrlcm_auv_path_response_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "goal_id",    "%d", msg->goal_id);
    TEXTREAD_ADD_FIELD (tr, "are_we_there_yet",    "%d", msg->are_we_there_yet);
    TEXTREAD_ADD_FIELD (tr, "distance",    "%f", msg->distance);
    textread_stop (tr);
}

   
