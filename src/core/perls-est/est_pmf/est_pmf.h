#ifndef __PERLS_EST_PMF_H__
#define __PERLS_EST_PMF_H__

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-lcmtypes/perllcm_est_navigator_index_t.h"

#include "../est_core.h"

#ifdef __cplusplus
extern "C" {
#endif

// function typedef to fill index structure
// each process model must have one of these functions
typedef void (*est_pmf_get_index_t) (perllcm_est_navigator_index_t *index_t);

// initializes all index values to -1
void
init_index_t (perllcm_est_navigator_index_t *index);

// -----------------------------------------------------------------------------
// PROCESS MODELS
// -----------------------------------------------------------------------------

// constant velocity process model in body frame -------------------------------
typedef struct _est_pmf_const_vel_body_user_t est_pmf_const_vel_body_user_t;
struct _est_pmf_const_vel_body_user_t
{     
    gsl_matrix *Qv; //continuous process noise
};

est_pmf_const_vel_body_user_t *
est_pmf_const_vel_body_alloc_user_t (void);

void
est_pmf_const_vel_body_free_user_t (void *user);

void
est_pmf_const_vel_body (const est_estimator_t *state, const gsl_vector *u, const double dt, 
                        gsl_vector *x_bar, gsl_matrix *F, gsl_matrix *Q, void *user);
void
est_pmf_const_vel_body_get_index_t (perllcm_est_navigator_index_t *index);
// -----------------------------------------------------------------------------



// differential drive process model for segway like robot frame ----------------

typedef struct _est_pmf_differential_drive_user_t est_pmf_differential_drive_user_t;
struct _est_pmf_differential_drive_user_t
{
    gsl_matrix *Qu;   //control noise
    gsl_matrix *Qadd; //additive noise for elements not effected by control
    double axel_track;
    double wheel_radius;
    double wheel_size_fudge_factor;
};

est_pmf_differential_drive_user_t *
est_pmf_differential_drive_alloc_user_t (void);

void
est_pmf_differential_drive_free_user_t (void *user);

void
est_pmf_differential_drive (const est_estimator_t *state, const gsl_vector *u, const double dt, 
                            gsl_vector *x_bar, gsl_matrix *F, gsl_matrix *Q, void *user);
void
est_pmf_differential_drive_get_index_t (perllcm_est_navigator_index_t *index);
// -----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif //__PERLS_EST_PMF_H__
