#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <bot_core/bot_core.h>

#include "perls-common/error.h"
#include "perls-common/units.h"

#include "perls-math/ssc.h"

#include "renderer_robot_pose.h"
#include "pose_data_sources.h"

void
os_conduit_osi_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                   const senlcm_uvc_osi_t *msg, void *user)
{
    RendererRobotPose *self = (RendererRobotPose*) user;
    
    //convert lat lon rads to local xy
    double ll_deg[2] = {msg->latitude  * UNITS_RADIAN_TO_DEGREE,
                        msg->longitude * UNITS_RADIAN_TO_DEGREE};
    double yx[2] = {0};
    bot_gps_linearize_to_xy (self->llxy, ll_deg, yx); // NOTE returns yx -> returns in ENU not NED
    
    //update pose
    self->pose.utime = msg->utime;
    self->pose.xyzrph[0] = yx[1];
    self->pose.xyzrph[1] = yx[0];
    
    self->altitude = msg->altimeter;
    self->next_waypoint = msg->nextwp;   
    self->dist_to_next_waypoint = msg->dist_to_nextwp;
    self->speed = msg->speed;
    
    update_history (msg->utime, self);
}

void
os_conduit_opi_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                   const senlcm_uvc_opi_t *msg, void *user)
{
    RendererRobotPose *self = (RendererRobotPose*) user;

    self->pose.utime = msg->utime;
    self->batt_percent = msg->percent;

    update_history (msg->utime, self);
}

//void
//os_conduit_cb (const lcm_recv_buf_t *rbuf, const char *channel,
//               const perllcm_auv_os_conduit_t *msg, void *user)
//{
//    RendererRobotPose *self = (RendererRobotPose*) user;
//    
//    double heading, pitch, roll, temp, depth = 0;
//    int ret = sscanf (msg->compass.value, "$C%lfP%lfR%lfT%lfD%lf", 
//                      &heading, &pitch, &roll, &temp, &depth);
//    if (ret == 5) {
//        if (depth > 255)
//            self->pose.xyzrph[2] = 255;
//        self->pose.utime = msg->utime;
//        self->pose.xyzrph[2] = depth * UNITS_FEET_TO_METER;
//        self->pose.xyzrph[3] = roll * UNITS_DEGREE_TO_RADIAN;
//        self->pose.xyzrph[4] = pitch * UNITS_DEGREE_TO_RADIAN;
//        self->pose.xyzrph[5] = heading * UNITS_DEGREE_TO_RADIAN;
//    }
//    
//    update_history (msg->utime, self);
//}

void
navigator_cb (const lcm_recv_buf_t *rbuf, const char *channel,
              const perllcm_auv_navigator_t *msg, void *user)
{
    RendererRobotPose *self = (RendererRobotPose*) user;
    
    if (msg->mu_len) {
        self->pose.utime = msg->utime;
        self->pose.xyzrph[0] = (-1 == msg->index.x) ? 0 : msg->mu[msg->index.x];
        self->pose.xyzrph[1] = (-1 == msg->index.y) ? 0 : msg->mu[msg->index.y];
        self->pose.xyzrph[2] = (-1 == msg->index.z) ? 0 : msg->mu[msg->index.z];
        self->pose.xyzrph[3] = (-1 == msg->index.r) ? 0 : msg->mu[msg->index.r];
        self->pose.xyzrph[4] = (-1 == msg->index.p) ? 0 : msg->mu[msg->index.p];
        self->pose.xyzrph[5] = (-1 == msg->index.h) ? 0 : msg->mu[msg->index.h];
        
        if (-1 != msg->index.x && -1 != msg->index.y) {
            self->pose.xyzrph_cov[0] = msg->Sigma[msg->index.x*msg->mu_len + msg->index.x];
            self->pose.xyzrph_cov[1] = msg->Sigma[msg->index.x*msg->mu_len + msg->index.y];
            self->pose.xyzrph_cov[6] = msg->Sigma[msg->index.y*msg->mu_len + msg->index.x];
            self->pose.xyzrph_cov[7] = msg->Sigma[msg->index.y*msg->mu_len + msg->index.y];
        }
        
        update_history (msg->utime, self);
    }
}

void
segway_navigator_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                     const perllcm_segway_navigator_t *msg, void *user)
{
    RendererRobotPose *self = (RendererRobotPose*) user;
    
    if (msg->mu_len) {
        self->pose.utime = msg->utime;
        self->pose.xyzrph[0] = (-1 == msg->index.x) ? 0 : msg->mu[msg->index.x];
        self->pose.xyzrph[1] = (-1 == msg->index.y) ? 0 : msg->mu[msg->index.y];
        self->pose.xyzrph[2] = (-1 == msg->index.z) ? 0 : msg->mu[msg->index.z];
        self->pose.xyzrph[3] = (-1 == msg->index.r) ? 0 : msg->mu[msg->index.r];
        self->pose.xyzrph[4] = (-1 == msg->index.p) ? 0 : msg->mu[msg->index.p];
        self->pose.xyzrph[5] = (-1 == msg->index.h) ? 0 : msg->mu[msg->index.h];
     
        // fill velocities
        self->pose.xyzrph_dot[0] = (-1 == msg->index.x_dot) ? 0 : msg->mu[msg->index.x_dot];
        self->pose.xyzrph_dot[1] = (-1 == msg->index.y_dot) ? 0 : msg->mu[msg->index.y_dot];
        self->pose.xyzrph_dot[2] = (-1 == msg->index.z_dot) ? 0 : msg->mu[msg->index.z_dot];
        self->pose.xyzrph_dot[3] = (-1 == msg->index.r_dot) ? 0 : msg->mu[msg->index.r_dot];
        self->pose.xyzrph_dot[4] = (-1 == msg->index.p_dot) ? 0 : msg->mu[msg->index.p_dot];
        self->pose.xyzrph_dot[5] = (-1 == msg->index.h_dot) ? 0 : msg->mu[msg->index.h_dot];
     
        if (-1 != msg->index.x && -1 != msg->index.y) {
            self->pose.xyzrph_cov[0] = msg->Sigma[msg->index.x*msg->mu_len + msg->index.x];
            self->pose.xyzrph_cov[1] = msg->Sigma[msg->index.x*msg->mu_len + msg->index.y];
            self->pose.xyzrph_cov[6] = msg->Sigma[msg->index.y*msg->mu_len + msg->index.x];
            self->pose.xyzrph_cov[7] = msg->Sigma[msg->index.y*msg->mu_len + msg->index.y];
        }
        
        update_history (msg->utime, self);
    }
}

void
os_compass_cb (const lcm_recv_buf_t *rbuf, const char *channel,
               const senlcm_os_compass_t *msg, void *user)
{
    RendererRobotPose *self = (RendererRobotPose*) user;
    
    self->pose.utime = msg->utime;
    self->pose.xyzrph[3] = msg->rph[0];
    self->pose.xyzrph[4] = msg->rph[1];
    self->pose.xyzrph[5] = msg->rph[2];
    
    update_history (msg->utime, self);
}

void
gpsd_cb (const lcm_recv_buf_t *rbuf, const char *channel,
         const senlcm_gpsd_t *msg, void *user)
{
    RendererRobotPose *self = (RendererRobotPose*) user;
    
    //convert lat lon rads to local xy
    double ll_deg[2] = {msg->latitude  * UNITS_RADIAN_TO_DEGREE,
                        msg->longitude * UNITS_RADIAN_TO_DEGREE};
    double yx[2] = {0};
    bot_gps_linearize_to_xy(self->llxy, ll_deg, yx); // NOTE returns yx -> returns in ENU not NED
    
    //update pose
    self->pose.utime = msg->utime;
    self->pose.xyzrph[0] = yx[1];
    self->pose.xyzrph[1] = yx[0];
    
    update_history (msg->utime, self);
}

void
gpsd3_cb (const lcm_recv_buf_t *rbuf, const char *channel,
          const senlcm_gpsd3_t *msg, void *user)
{
    RendererRobotPose *self = (RendererRobotPose*) user;

    if (msg->fix.mode >= SENLCM_GPSD3_FIX_T_MODE_2D) {
        
        //convert lat lon rads to local xy
        double ll_deg[2] = {msg->fix.latitude  * UNITS_RADIAN_TO_DEGREE,
                            msg->fix.longitude * UNITS_RADIAN_TO_DEGREE};
        double yx[2] = {0};
        bot_gps_linearize_to_xy (self->llxy, ll_deg, yx); // NOTE returns yx -> returns in ENU not NED
        
        //update pose
        self->pose.utime = msg->utime;
        self->pose.xyzrph[0] = yx[1];
        self->pose.xyzrph[1] = yx[0];
        if (msg->fix.mode >= SENLCM_GPSD3_FIX_T_MODE_3D && self->use_alt) {
            self->pose.xyzrph[2] = -(msg->fix.altitude- self->org_alt);
        }
        
        update_history (msg->utime, self);
    }
}

void
gpsd3_raw_cb (const lcm_recv_buf_t *rbuf, const char *channel,
             const senlcm_raw_t *msg, void *user) {
   
    RendererRobotPose *self = (RendererRobotPose*) user;

    if (0==strncmp ((char *)(msg->data), "$GPGLL", 6)) {
        
        double ll_deg[2] = {0};
        
        char ns, ew;
        double lat, lon;
        sscanf ((char *)msg->data,"$GPGLL,%lf,%c,%lf,%c,", &lat, &ns, &lon, &ew);

        double lat_deg = floor(lat/100.0);
        ll_deg[0] =  lat_deg + (lat - lat_deg*100)/60;
        double lon_deg = floor(lon/100.0);
        ll_deg[1] =  lon_deg + (lon - lon_deg*100)/60;
        if (ns == 'S') 
            ll_deg[0] = -ll_deg[0];
        
        if (ew == 'W') 
            ll_deg[1] = -ll_deg[1];
        
        //convert lat lon rads to local xy
        double yx[2] = {0};
        //double ll_deg[2] = {42.293215, -83.709662};
        bot_gps_linearize_to_xy (self->llxy, ll_deg, yx); // NOTE returns yx -> returns in ENU not NED     
        
        //update pose
        self->pose.utime = msg->utime;
        self->pose.xyzrph[0] = yx[1];
        self->pose.xyzrph[1] = yx[0];
        
        if (self->x_wl_adjust) ssc_head2tail (self->pose.xyzrph, NULL, self->x_wl, self->pose.xyzrph);
        update_history (msg->utime, self);
    } 
    
}

void
generic_position_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                     const perllcm_position_t *msg, void *user)
{
    RendererRobotPose *self = (RendererRobotPose*) user;
    
    self->pose.utime = msg->utime;
    memcpy (self->pose.xyzrph, msg->xyzrph, sizeof (double) * 6);
    memcpy (self->pose.xyzrph_cov, msg->xyzrph_cov, sizeof (double) * 6*6);
    memcpy (self->pose.xyzrph_dot, msg->xyzrph_dot, sizeof (double) * 6);
    
    update_history (msg->utime, self);
}


void
perllcm_auv_iver_state_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                             const perllcm_auv_iver_state_t *msg, void *user) {
    
    RendererRobotPose *self = (RendererRobotPose*) user;
    
    self->pose.utime = msg->position.utime;
    memcpy (self->pose.xyzrph, msg->position.xyzrph, sizeof (double) * 6);
    memcpy (self->pose.xyzrph_cov, msg->position.xyzrph_cov, sizeof (double) * 6*6);
    memcpy (self->pose.xyzrph_dot, msg->position.xyzrph_dot, sizeof (double) * 6);
    
    update_history (msg->position.utime, self);  
}

void
perllcm_auv_acomms_iver_state_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                                    const perllcm_auv_acomms_iver_state_t *msg, void *user) {
    
    RendererRobotPose *self = (RendererRobotPose*) user;
    
    self->pose.utime = msg->utime;
    //self->pose.utime = msg->position.utime;
    memcpy (self->pose.xyzrph, msg->position.xyzrph, sizeof (double) * 6);
    memcpy (self->pose.xyzrph_cov, msg->position.xyzrph_cov, sizeof (double) * 6*6);
    memcpy (self->pose.xyzrph_dot, msg->position.xyzrph_dot, sizeof (double) * 6);
    
    self->batt_percent = msg->uvc_battery_percent;
    self->altitude = msg->altitude;
    self->next_waypoint = msg->uvc_next_waypoint;   
    self->dist_to_next_waypoint = msg->uvc_dist_to_next_waypoint;
    self->abort = msg->abort_state;
    self->error = msg->uvc_error;
    
    update_history (msg->utime, self);  
    //update_history (msg->position.utime, self);  
}


void
perllcm_auv_acomms_mini_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                              const perllcm_auv_acomms_mini_t *msg, void *user) {
    
    RendererRobotPose *self = (RendererRobotPose*) user;
    
    if (msg->mini_type == PERLLCM_AUV_ACOMMS_MINI_T_MINI_WAYPOINT)
        self->next_waypoint = msg->uvc_next_waypoint;
    else if (msg->mini_type == PERLLCM_AUV_ACOMMS_MINI_T_MINI_ERROR)   
        self->error = msg->uvc_error;
    
    update_history (msg->utime, self);  
    
}

void
mocap_position_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                     const senlcm_mocap_t *msg, void *user)
{
    if (!msg->valid) return;

    RendererRobotPose *self = (RendererRobotPose*) user;
    
    self->pose.utime = msg->utime;
    memcpy (self->pose.xyzrph, msg->xyzrph, sizeof (double) * 6);
    
    update_history (msg->utime, self);
}

// switchyard to subscribe based on config file keys
void
pose_data_source_subscribe (const char *data_source_key, RendererRobotPose *self, const char *lcm_channel)
{
    
    if (0==strcmp ("auv_os_conduit_osi", data_source_key))
        senlcm_uvc_osi_t_subscribe (self->lcm, lcm_channel, os_conduit_osi_cb, self);
    
    //else if (0==strcmp ("auv_os_conduit_t", data_source_key))
    //    perllcm_auv_os_conduit_t_subscribe (self->lcm, lcm_channel, &os_conduit_cb, self);
    
    else if (0==strcmp ("auv_navigator", data_source_key))
        perllcm_auv_navigator_t_subscribe (self->lcm, lcm_channel, &navigator_cb, self);
            
    else if (0==strcmp ("os_compass", data_source_key))
        senlcm_os_compass_t_subscribe (self->lcm, lcm_channel, &os_compass_cb, self);
            
    else if (0==strcmp ("gpsd", data_source_key))
        senlcm_gpsd_t_subscribe (self->lcm, lcm_channel, &gpsd_cb, self);
            
    else if (0==strcmp ("gpsd3", data_source_key))
        senlcm_gpsd3_t_subscribe (self->lcm, lcm_channel, &gpsd3_cb, self);
            
    else if (0==strcmp ("segway_navigator", data_source_key))
        perllcm_segway_navigator_t_subscribe (self->lcm, lcm_channel, &segway_navigator_cb, self);
        
    else if (0==strcmp ("position", data_source_key))
        perllcm_position_t_subscribe (self->lcm, lcm_channel, &generic_position_cb, self);
        
    else if (0==strcmp ("acomms_iver_state", data_source_key))
        perllcm_auv_acomms_iver_state_t_subscribe (self->lcm, lcm_channel, &perllcm_auv_acomms_iver_state_t_cb, self);
        
    else if (0==strcmp ("acomms_mini", data_source_key))
        perllcm_auv_acomms_mini_t_subscribe (self->lcm, lcm_channel, &perllcm_auv_acomms_mini_t_cb, self);
        
    else if (0==strcmp ("iver_state", data_source_key))
        perllcm_auv_iver_state_t_subscribe (self->lcm, lcm_channel, &perllcm_auv_iver_state_t_cb, self);
    
    else if (0==strcmp ("gpsd3_raw", data_source_key))
        senlcm_raw_t_subscribe (self->lcm, lcm_channel, &gpsd3_raw_cb, self);    
    
    else if (0==strcmp ("mocap", data_source_key))
        senlcm_mocap_t_subscribe (self->lcm, lcm_channel, &mocap_position_cb, self);    

    else {
        ERROR ("\tData source %s not found \n", data_source_key);
        return;
    }

    printf ("\tAdded data source: %s \n", data_source_key);
}
