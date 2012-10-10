#ifndef __PERLS_NAVIGATOR_LCM_CALLBACKS_H__
#define __PERLS_NAVIGATOR_LCM_CALLBACKS_H__

#include "perls-lcmtypes/senlcm_uvc_osi_t.h"
#include "perls-lcmtypes/senlcm_dstar_ssp1_t.h"
#include "perls-lcmtypes/senlcm_gpsd3_t.h"
#include "perls-lcmtypes/senlcm_gpsd_t.h"
#include "perls-lcmtypes/senlcm_kvh_dsp3000_t.h"
#include "perls-lcmtypes/senlcm_ms_gx1_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd4_t.h"

#include "perls-est/est.h"

void
free_meas_t (est_meas_t *meas_t);


// desert star depth (z) sensor
void
dstar_ssp1_z_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                 const senlcm_dstar_ssp1_t *msg, void *user);

// generic gps position sensor (assumes measurements moved to local coord frame)
void
gpsd_xy_cb (const lcm_recv_buf_t *rbuf, const char *channel,
            const senlcm_gpsd_t *msg, void *user);

// generic gps position sensor (assumes measurements moved to local coord frame)
void
gpsd3_xy_cb (const lcm_recv_buf_t *rbuf, const char *channel,
             const senlcm_gpsd3_t *msg, void *user);

// KVH fiber-optic gyro yaw rate (c) sensor
void
kvh_dsp3000_c_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                  const senlcm_kvh_dsp3000_t *msg, void *user);

// microstrain mems attidude sensor (AKA rph)
void
ms_gx1_rph_cb (const lcm_recv_buf_t *rbuf, const char *channel,
               const senlcm_ms_gx1_t *msg, void *user);

// microstrain mems attidude sensor (abc AKA pqr)
void
ms_gx1_abc_cb (const lcm_recv_buf_t *rbuf, const char *channel,
               const senlcm_ms_gx1_t *msg, void *user);

// rdi doppler velocity log for body-frame velocities (uvw)
void
rdi_pd4_uvw_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                const senlcm_rdi_pd4_t *msg, void *user);

// osi motor count for body-frame forward velocity (u)
void
osi_motor_count_u_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                      const senlcm_uvc_osi_t *msg, void *user);

#endif //__PERLS_NAVIGATOR_LCM_CALLBACKS_H__
