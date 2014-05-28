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
    TEXTREAD_ADD_FIELD (tr, "altitude","%f",      msg->altitude);
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

void
acfrlcm_auv_control_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const acfrlcm_auv_control_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "depth_mode",    "%d", msg->depth_mode);
    TEXTREAD_ADD_FIELD (tr, "run_mode",    "%d", msg->run_mode);
    TEXTREAD_ADD_FIELD (tr, "heading",    "%f", msg->heading);
    TEXTREAD_ADD_FIELD (tr, "depth",    "%f", msg->depth); 
    TEXTREAD_ADD_FIELD (tr, "altitude",    "%f", msg->altitude); 
    TEXTREAD_ADD_FIELD (tr, "pitch",    "%f", msg->pitch); 
    TEXTREAD_ADD_FIELD (tr, "vx",    "%f", msg->vx);  
    textread_stop (tr);
}
     
void
acfrlcm_auv_iver_motor_command_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const acfrlcm_auv_iver_motor_command_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "top",    "%f", msg->top);
    TEXTREAD_ADD_FIELD (tr, "bottom",    "%f", msg->bottom);  
    TEXTREAD_ADD_FIELD (tr, "port",    "%f", msg->port);  
    TEXTREAD_ADD_FIELD (tr, "starboard",    "%f", msg->starboard);  
    TEXTREAD_ADD_FIELD (tr, "main",    "%f", msg->main);    
    TEXTREAD_ADD_FIELD (tr, "source",    "%d", msg->source);  
    
    textread_stop (tr);
}

void
acfrlcm_auv_global_planner_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const acfrlcm_auv_global_planner_t *msg, void *user)
{
    lcmlog_export_t *lle = user;
    textread_t *tr = lle_get_textread (lle, channel);

    textread_start (tr);
    TEXTREAD_ADD_FIELD (tr, "utime",    "%"PRId64, msg->utime);
    TEXTREAD_ADD_FIELD (tr, "id",    "%d", msg->id);
    TEXTREAD_ADD_FIELD (tr, "command",    "%d", msg->command);
    TEXTREAD_ADD_FIELD (tr, "point1(:,1)",    "%f", msg->point1[0]);
    TEXTREAD_ADD_FIELD (tr, "point1(:,2)",    "%f", msg->point1[1]);
    TEXTREAD_ADD_FIELD (tr, "point1(:,3)",    "%f", msg->point1[2]);
    TEXTREAD_ADD_FIELD (tr, "point1(:,4)",    "%f", msg->point1[3]);
    TEXTREAD_ADD_FIELD (tr, "point1(:,5)",    "%f", msg->point1[4]);
    TEXTREAD_ADD_FIELD (tr, "point1(:,6)",    "%f", msg->point1[5]);
    TEXTREAD_ADD_FIELD (tr, "point2(:,1)",    "%f", msg->point2[0]);
    TEXTREAD_ADD_FIELD (tr, "point2(:,2)",    "%f", msg->point2[1]);
    TEXTREAD_ADD_FIELD (tr, "point2(:,3)",    "%f", msg->point2[2]);
    TEXTREAD_ADD_FIELD (tr, "point2(:,4)",    "%f", msg->point2[3]);
    TEXTREAD_ADD_FIELD (tr, "point2(:,5)",    "%f", msg->point2[4]);
    TEXTREAD_ADD_FIELD (tr, "point2(:,6)",    "%f", msg->point2[5]);
    TEXTREAD_ADD_FIELD (tr, "velocity(:,1)",    "%f", msg->velocity[0]);
    TEXTREAD_ADD_FIELD (tr, "velocity(:,2)",    "%f", msg->velocity[1]);
    TEXTREAD_ADD_FIELD (tr, "velocity(:,3)",    "%f", msg->velocity[2]);
    TEXTREAD_ADD_FIELD (tr, "timeout",    "%f", msg->timeout);
    TEXTREAD_ADD_FIELD (tr, "var_d(:,1)",    "%f", msg->var_d[0]);
    TEXTREAD_ADD_FIELD (tr, "var_d(:,2)",    "%f", msg->var_d[1]);
    TEXTREAD_ADD_FIELD (tr, "var_d(:,3)",    "%f", msg->var_d[2]);
    TEXTREAD_ADD_FIELD (tr, "var_d(:,4)",    "%f", msg->var_d[3]);
    TEXTREAD_ADD_FIELD (tr, "var_d(:,5)",    "%f", msg->var_d[4]);
    TEXTREAD_ADD_FIELD (tr, "var_d(:,6)",    "%f", msg->var_d[5]);
    TEXTREAD_ADD_FIELD (tr, "var_i(:,1)",    "%d", msg->var_i[0]);
    TEXTREAD_ADD_FIELD (tr, "var_i(:,2)",    "%d", msg->var_i[1]);
    TEXTREAD_ADD_FIELD (tr, "var_i(:,3)",    "%d", msg->var_i[2]);
    TEXTREAD_ADD_FIELD (tr, "var_i(:,4)",    "%d", msg->var_i[3]);
    TEXTREAD_ADD_FIELD (tr, "var_i(:,5)",    "%d", msg->var_i[4]);
    TEXTREAD_ADD_FIELD (tr, "var_i(:,6)",    "%d", msg->var_i[5]);
    TEXTREAD_ADD_FIELD (tr, "str",    "%s", msg->str);   
    textread_stop (tr);
}
