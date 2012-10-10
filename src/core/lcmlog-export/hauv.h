#ifndef __HANDLERS_HAUV_H__
#define __HANDLERS_HAUV_H__

#include "perls-lcmtypes/lcmtypes.h"

void
hauv_bs_cnv_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_cnv_t *msg, void *user);

void
hauv_bs_dvl_2_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                         const hauv_bs_dvl_2_t *msg, void *user);

void
hauv_bs_dvl_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_dvl_t *msg, void *user);

void
hauv_bs_imu_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_imu_t *msg, void *user);

void
hauv_bs_nvg_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_nvg_t *msg, void *user);

void
hauv_bs_nvr_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_nvr_t *msg, void *user);

void
hauv_bs_pit_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_pit_t *msg, void *user);

void
hauv_bs_raw_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_raw_t *msg, void *user);

void
hauv_bs_rbs_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_rbs_t *msg, void *user);

void
hauv_bs_rcm_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_rcm_t *msg, void *user);

void
hauv_bs_rdp_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_rdp_t *msg, void *user);

void
hauv_bs_rnv_2_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                         const hauv_bs_rnv_2_t *msg, void *user);

void
hauv_bs_rnv_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_bs_rnv_t *msg, void *user);

void
hauv_didson_raw_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                           const hauv_didson_raw_t *msg, void *user);

void
hauv_didson_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_didson_t *msg, void *user);

void
hauv_pl_gbp_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_pl_gbp_t *msg, void *user);

void
hauv_pl_ghp_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_pl_ghp_t *msg, void *user);

void
hauv_pl_raw_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_pl_raw_t *msg, void *user);

void
hauv_pl_san_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_pl_san_t *msg, void *user);

void
hauv_pl_sus_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                       const hauv_pl_sus_t *msg, void *user);

void
hauv_vehicle_state_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                              const hauv_vehicle_state_t *msg, void *user);

#endif //__HANDLERS_HAUV_H__
