#ifndef __PERLS_SEG_NAVIGATOR_LCM_CALLBACKS_H__
#define __PERLS_SEG_NAVIGATOR_LCM_CALLBACKS_H__

#include "perls-lcmtypes/senlcm_kvh_dsp3000_t.h"
#include "perls-lcmtypes/senlcm_ms_gx3_t.h"
#include "perls-lcmtypes/senlcm_ms_gx3_25_t.h"

#include "navigator.h"

void
free_meas_t (est_meas_t *meas_t);

// microstrain mems attitude sensor (AKA rph)
void
ms_gx3_rph_cb (const lcm_recv_buf_t *rbuf, const char *channel,
               const senlcm_ms_gx3_t *msg, void *user);

// microstrain mems attitude sensor (abc AKA pqr)
void
ms_gx3_abc_cb (const lcm_recv_buf_t *rbuf, const char *channel,
               const senlcm_ms_gx3_t *msg, void *user);

// microstrain mems attitude sensor (AKA rph)
void
ms_gx3_rp_cb (const lcm_recv_buf_t *rbuf, const char *channel,
              const senlcm_ms_gx3_t *msg, void *user);

// microstrain mems attitude sensor (abc AKA pqr)
void
ms_gx3_ab_cb (const lcm_recv_buf_t *rbuf, const char *channel,
              const senlcm_ms_gx3_t *msg, void *user);

// microstrain mems attitude sensor (AKA rph)
void
ms_gx3_rp_25_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                 const senlcm_ms_gx3_25_t *msg, void *user);

// microstrain mems attitude sensor (abc AKA pqr)
void
ms_gx3_ab_25_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                 const senlcm_ms_gx3_25_t *msg, void *user);

#endif //__PERLS_SEG_NAVIGATOR_LCM_CALLBACKS_H__
