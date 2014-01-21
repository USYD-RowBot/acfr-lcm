#ifndef __HANDLERS_PERLLCM_H__
#define __HANDLERS_PERLLCM_H__

#include "perls-lcmtypes/lcmtypes.h"

void
perllcm_est_navigator_debug_meas_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                                            const perllcm_est_navigator_debug_meas_t *msg, void *user);

void
perllcm_est_navigator_debug_pred_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                                            const perllcm_est_navigator_debug_pred_t *msg, void *user);

void
perllcm_auv_navigator_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const perllcm_auv_navigator_t *msg, void *user);

void
perllcm_segway_navigator_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const perllcm_segway_navigator_t *msg, void *user);

void
perllcm_segway_state_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                                const perllcm_segway_state_t *msg, void *user);

void
perllcm_ardrone_state_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                                const perllcm_ardrone_state_t *msg, void *user);
                                
void
perllcm_ardrone_drive_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                                const perllcm_ardrone_drive_t *msg, void *user);
                                
void
perllcm_position_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                                const perllcm_position_t *msg, void *user);

void
perllcm_van_vlink_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const perllcm_van_vlink_t *msg, void *user);

void
perllcm_van_plink_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                             const perllcm_van_plink_t *msg, void *user);

#endif //__HANDLERS_PERLLCM_H__
