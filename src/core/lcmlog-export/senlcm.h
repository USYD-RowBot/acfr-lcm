#ifndef __HANDLERS_SENLCM_H__
#define __HANDLERS_SENLCM_H__

#include "perls-lcmtypes/lcmtypes.h"

void
senlcm_acomms_range_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_acomms_range_t *msg, void *user);

void
senlcm_dstar_ssp1_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_dstar_ssp1_t *msg, void *user);

void
senlcm_gpsd3_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_gpsd3_t *msg, void *user);

void
senlcm_gpsd_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_gpsd_t *msg, void *user);

void
senlcm_kvh_dsp3000_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_kvh_dsp3000_t *msg, void *user);
void
senlcm_mocap_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_mocap_t *msg, void *user);
void
senlcm_ms_gx1_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_ms_gx1_t *msg, void *user);

void
senlcm_ms_gx3_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_ms_gx3_t *msg, void *user);

void 
senlcm_ms_gx3_25_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_ms_gx3_25_t *msg, void *user);

void
senlcm_os_compass_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_os_compass_t *msg, void *user);

void
senlcm_ppsboard_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_ppsboard_t *msg, void *user);

void
senlcm_prosilica_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_prosilica_t *msg, void *user);

void
senlcm_rdi_pd4_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_rdi_pd4_t *msg, void *user);

void
senlcm_tritech_es_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
        const senlcm_tritech_es_t *msg, void *user);

void
senlcm_uvc_osi_t_handler (const lcm_recv_buf_t *rbuf, const char *channel,
       const senlcm_uvc_osi_t *msg, void *user);

#endif //__HANDLERS_SENLCM_H__
