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





////////////////////////////////////////////////////////////////////////////////
// Additional handlers added by ACFR
////////////////////////////////////////////////////////////////////////////////

void
senlcm_seabird_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                             const senlcm_seabird_t *msg, void *user);

// FIXME: What else do we need to extract for the processing scripts
void
senlcm_rdi_pd0_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                             const senlcm_rdi_pd0_t *msg, void *user);

void
senlcm_os_power_system_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                             const senlcm_os_power_system_t *msg, void *user);

void
senlcm_ecopuck_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                             const senlcm_ecopuck_t *msg, void *user);

void
senlcm_oas_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                             const senlcm_oas_t *msg, void *user);

void
senlcm_parosci_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                             const senlcm_parosci_t *msg, void *user);

void
senlcm_lq_modem_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                             const senlcm_lq_modem_t *msg, void *user);

void
senlcm_ysi_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                             const senlcm_ysi_t *msg, void *user);

void
senlcm_micron_ping_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                             const senlcm_micron_ping_t *msg, void *user);

void
senlcm_rdi_pd5_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                          const senlcm_rdi_pd5_t *msg, void *user);


void
senlcm_tcm_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                         const senlcm_tcm_t *msg, void *user);
void
senlcm_kvh1750_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                         const senlcm_kvh1750_t *msg, void *user);
                         
void 
senlcm_usb2000_spec_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                              const senlcm_usb2000_spec_t *msg, void *user);

void                              
senlcm_sts_spec_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                              const senlcm_sts_spec_t *msg, void *user);
                              
void 
senlcm_usbl_fix_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                              const senlcm_usbl_fix_t *msg, void *user);

void 
senlcm_novatel_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                              const senlcm_novatel_t *msg, void *user);

void 
senlcm_evologics_usbl_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                              const senlcm_evologics_usbl_t *msg, void *user);                              

void 
senlcm_ahrs_t_handler (const lcm_recv_buf_t *rbuf, const char *channel, 
                              const senlcm_ahrs_t *msg, void *user);                              

#endif //__HANDLERS_SENLCM_H__
