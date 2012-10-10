#ifndef __HANDLERS_SE_H__
#define __HANDLERS_SE_H__

#include "perls-lcmtypes/lcmtypes.h"

void
se_add_node_ack_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const se_add_node_ack_t *msg, void *user);

void
se_add_node_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const se_add_node_t *msg, void *user);

void
se_goto_state_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                         const se_goto_state_t *msg, void *user);

void
se_goto_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                   const se_goto_t *msg, void *user);

void
se_info_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                   const se_info_t *msg, void *user);

void
se_option_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                     const se_option_t *msg, void *user);

void
se_pose_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                   const se_pose_t *msg, void *user);

void
se_propose_link_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const se_propose_link_t *msg, void *user);

void
se_publish_link_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const se_publish_link_t *msg, void *user);

void
se_request_state_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                            const se_request_state_t *msg, void *user);

void
se_return_state_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const se_return_state_t *msg, void *user);

void
se_save_isam_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                        const se_save_isam_t *msg, void *user);
  
void
se_save_waypoint_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                            const se_save_waypoint_t *msg, void *user);


#endif //__HANDLERS_SE_H__
