#ifndef __PERLS_EST_OMF_H__
#define __PERLS_EST_OMF_H__

#include <bot_core/bot_core.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-lcmtypes/perllcm_est_navigator_index_t.h"

#include "../est_core.h"

#ifdef __cplusplus
extern "C" {
#endif


// desert star depth (z) sensor
typedef struct _est_omf_dstar_ssp1_z_user_t est_omf_dstar_ssp1_z_user_t;
struct _est_omf_dstar_ssp1_z_user_t
{     
    double X_vs[6]; // transform between vehicle and sensor
};

void
est_omf_dstar_ssp1_z (const est_estimator_t *state,  const gsl_vector *z,
                      const void *index_map, const gsl_matrix *R,
                      gsl_vector *nu, gsl_matrix *H, void *user);
//------------------------------------------------------------------------------


// KVH fiber-optic gyro yaw rate (c) sensor
typedef struct _est_omf_kvh_dsp3000_c_user_t est_omf_kvh_dsp3000_c_user_t;
struct _est_omf_kvh_dsp3000_c_user_t
{     
    double X_vs[6]; // transform between vehicle and sensor
};

void
est_omf_kvh_dsp3000_c (const est_estimator_t *state,  const gsl_vector *z,
                       const void *index_map, const gsl_matrix *R,
                       gsl_vector *nu, gsl_matrix *H, void *user);
//------------------------------------------------------------------------------


// generic gps position sensor (assumes measurements moved to local coord frame)
typedef struct _est_omf_gpsd_xy_user_t est_omf_gpsd_xy_user_t;
struct _est_omf_gpsd_xy_user_t
{
    double X_vs[6]; // transform between vehicle and sensor
    BotGPSLinearize *llxy;  //struct containing gps origin used to convert between local xy  
};

void
est_omf_gpsd_xy (const est_estimator_t *state,  const gsl_vector *z,
                 const void *index_map, const gsl_matrix *R,
                 gsl_vector *nu, gsl_matrix *H, void *user);
//------------------------------------------------------------------------------

// generic gps position sensor (assumes measurements moved to local coord frame)
typedef struct _est_omf_gpsd3_xy_user_t est_omf_gpsd3_xy_user_t;
struct _est_omf_gpsd3_xy_user_t
{     
    double X_vs[6]; // transform between vehicle and sensor
    BotGPSLinearize *llxy;  //struct containing gps origin used to convert between local xy  
};

void
est_omf_gpsd3_xy (const est_estimator_t *state,  const gsl_vector *z,
                  const void *index_map, const gsl_matrix *R,
                  gsl_vector *nu, gsl_matrix *H, void *user);
//------------------------------------------------------------------------------


// microstrain mems attidude sensor (rph )
typedef struct _est_omf_ms_gx1_rph_user_t est_omf_ms_gx1_rph_user_t;
struct _est_omf_ms_gx1_rph_user_t
{     
    double X_vs[6]; // transform between vehicle and sensor
    double X_lr[6]; // transform between local and reference frame
};

void
est_omf_ms_gx1_rph (const est_estimator_t *state,  const gsl_vector *z,
                    const void *index_map, const gsl_matrix *R,
                    gsl_vector *nu, gsl_matrix *H, void *user);
//------------------------------------------------------------------------------


// microstrain mems attidude sensor (abc AKA pqr)
typedef struct _est_omf_ms_gx1_abc_user_t est_omf_ms_gx1_abc_user_t;
struct _est_omf_ms_gx1_abc_user_t
{     
    double X_vs[6]; // transform between vehicle and sensor
};

void
est_omf_ms_gx1_abc (const est_estimator_t *state,  const gsl_vector *z,
                    const void *index_map, const gsl_matrix *R,
                    gsl_vector *nu, gsl_matrix *H, void *user);
//------------------------------------------------------------------------------


// microstrain mems attidude sensor (rp )
typedef struct _est_omf_ms_gx1_rp_user_t est_omf_ms_gx1_rp_user_t;
struct _est_omf_ms_gx1_rp_user_t
{     
    double X_vs[6]; // transform between vehicle and sensor
    double X_lr[6]; // transform between local and reference frame
};

void
est_omf_ms_gx1_rp (const est_estimator_t *state,  const gsl_vector *z,
                      const void *index_map, const gsl_matrix *R,
                      gsl_vector *nu, gsl_matrix *H, void *user);
//------------------------------------------------------------------------------

// microstrain mems attidude sensor (ab AKA pq)
typedef struct _est_omf_ms_gx1_ab_user_t est_omf_ms_gx1_ab_user_t;
struct _est_omf_ms_gx1_ab_user_t
{     
    double X_vs[6]; // transform between vehicle and sensor
};

void
est_omf_ms_gx1_ab (const est_estimator_t *state,  const gsl_vector *z,
                   const void *index_map, const gsl_matrix *R,
                   gsl_vector *nu, gsl_matrix *H, void *user);
//------------------------------------------------------------------------------


// rdi doppler velocity log for body-frame velocities (vuw)
typedef struct _est_omf_rdi_pd4_uvw_user_t est_omf_rdi_pd4_uvw_user_t;
struct _est_omf_rdi_pd4_uvw_user_t
{     
    double X_vs[6]; // transform between vehicle and sensor
};

void
est_omf_rdi_pd4_uvw (const est_estimator_t *state,  const gsl_vector *z,
                     const void *index_map, const gsl_matrix *R,
                     gsl_vector *nu, gsl_matrix *H, void *user);
//------------------------------------------------------------------------------

// osi motor count for body-frame forward velocity (u)
typedef struct _est_omf_osi_motor_count_u_user_t est_omf_osi_motor_count_u_user_t;
struct _est_omf_osi_motor_count_u_user_t
{     
    //double X_vs[6]; // transform between vehicle and sensor
    //not here, convert when packing meas_t
    //int count_min;
    //int count_max;
    //double *mu;
    //double *std;
};

void
est_omf_osi_motor_count_u (const est_estimator_t *state,  const gsl_vector *z,
                           const void *index_map, const gsl_matrix *R,
                           gsl_vector *nu, gsl_matrix *H, void *user);
//------------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif //__PERLS_EST_OMF_H__
