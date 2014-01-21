#ifndef __VIEWER_POSE_DATA_SOURCES_H__
#define __VIEWER_POSE_DATA_SOURCES_H__

#include "perls-lcmtypes/perllcm_auv_navigator_t.h"
#include "perls-lcmtypes/senlcm_uvc_opi_t.h"
#include "perls-lcmtypes/senlcm_uvc_osi_t.h"
//#include "perls-lcmtypes/perllcm_auv_os_conduit_t.h"
#include "perls-lcmtypes/perllcm_position_t.h"
#include "perls-lcmtypes/perllcm_segway_navigator_t.h"
#include "perls-lcmtypes/perllcm_auv_iver_state_t.h"
#include "perls-lcmtypes/perllcm_auv_acomms_iver_state_t.h"
#include "perls-lcmtypes/perllcm_auv_acomms_mini_t.h"

#include "perls-lcmtypes/senlcm_gpsd3_t.h"
#include "perls-lcmtypes/senlcm_gpsd_t.h"
#include "perls-lcmtypes/senlcm_os_compass_t.h"
#include "perls-lcmtypes/senlcm_raw_t.h"
#include "perls-lcmtypes/senlcm_mocap_t.h"

#include "renderer_robot_pose.h"

#ifdef __cpluplus
extern "C" {
#endif

void
pose_data_source_subscribe (const char *data_source_key, RendererRobotPose *self, const char *lcm_channel);

void
os_conduit_osi_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                   const senlcm_uvc_osi_t *msg, void *user);

void
os_conduit_opi_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                   const senlcm_uvc_opi_t *msg, void *user);

//void
//os_conduit_cb (const lcm_recv_buf_t *rbuf, const char *channel,
//               const perllcm_auv_os_conduit_t *msg, void *user);

void
navigator_cb (const lcm_recv_buf_t *rbuf, const char *channel,
              const perllcm_auv_navigator_t *msg, void *user);

void
segway_navigator_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                     const perllcm_segway_navigator_t *msg, void *user);

void
os_compass_cb (const lcm_recv_buf_t *rbuf, const char *channel,
               const senlcm_os_compass_t *msg, void *user);

void
gpsd_cb (const lcm_recv_buf_t *rbuf, const char *channel,
         const senlcm_gpsd_t *msg, void *user);

void
gpsd3_cb (const lcm_recv_buf_t *rbuf, const char *channel,
          const senlcm_gpsd3_t *msg, void *user);

void
gpsd3_raw_cb (const lcm_recv_buf_t *rbuf, const char *channel,
              const senlcm_raw_t *msg, void *user); 

void
generic_position_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                     const perllcm_position_t *msg, void *user);

void
mocap_position_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                     const senlcm_mocap_t *msg, void *user);

void
perllcm_auv_iver_state_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                             const perllcm_auv_iver_state_t *msg, void *user);

void
perllcm_auv_acomms_iver_state_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                                    const perllcm_auv_acomms_iver_state_t *msg, void *user);

void
perllcm_auv_acomms_mini_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                              const perllcm_auv_acomms_mini_t *msg, void *user);

#ifdef __cpluplus
}
#endif

#endif // __VIEWER_POSE_DATA_SOURCES_H__
