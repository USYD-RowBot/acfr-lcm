#ifndef __PERLS_EST_UKF_H__
#define __PERLS_EST_UKF_H__

#include "est_core.h"

#ifdef __cplusplus
extern "C" {
#endif

void
ukf_predict (est_estimator_t *state, est_pred_t *pred);

void
ukf_correct (est_estimator_t *state, est_meas_t *meas);

#ifdef __cplusplus
}
#endif

#endif //__PERLS_EST_UKF_H__
