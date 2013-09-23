#ifndef __HANDLERS_ACFRLCM_H__
#define __HANDLERS_ACFRLCM_H__



#include "perls-lcmtypes/lcmtypes.h"

void
acfrlcm_auv_acfr_nav_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const acfrlcm_auv_acfr_nav_t *msg, void *user);

void
acfrlcm_auv_path_command_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const acfrlcm_auv_path_command_t *msg, void *user);

void
acfrlcm_auv_path_response_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const acfrlcm_auv_path_response_t *msg, void *user);

void
acfrlcm_auv_control_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const acfrlcm_auv_control_t *msg, void *user);

void
acfrlcm_auv_iver_motor_command_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const acfrlcm_auv_iver_motor_command_t *msg, void *user);

void
acfrlcm_auv_global_planner_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const acfrlcm_auv_global_planner_t *msg, void *user);


#endif
