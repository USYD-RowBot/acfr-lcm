#ifndef __PERLS_EST_EKF_H__
#define __PERLS_EST_EKF_H__

#include "est_core.h"

/**
 * @defgroup PerlsEstEkf Extended Kalman Filter
 * @brief EKF for libest
 * @ingroup PerlsEst
 * @author Nick Carlevaris-Bianco
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

void
ekf_predict (est_estimator_t *state, est_pred_t *pred);

void
ekf_predict_ds (est_estimator_t *state, est_pred_t *pred, int do_augment);

void
ekf_marginalize_d (est_estimator_t *state, int ds_ind);

void
ekf_correct_raw (est_estimator_t *state, est_meas_t *meas, const gsl_vector *nu, const gsl_matrix *H);

void
ekf_correct (est_estimator_t *state, est_meas_t *meas);

void
ekf_correct_ds (est_estimator_t *state, est_meas_t *meas, int ds_ind);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif


#endif //__PERLS_EST_EKF_H__
