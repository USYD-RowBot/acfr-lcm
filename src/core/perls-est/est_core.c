#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

#include "perls-math/gsl_util.h"

#include "error.h"
#include "ekf.h"
#include "ukf.h"
#include "pf.h"

#include "est_core.h"

int
check_pred_t (est_estimator_t *state, est_pred_t *pred_t);

int
check_meas_t (est_estimator_t *state, est_meas_t *meas_t);



// init the estimation engine
est_estimator_t *
est_init (est_method_t est_method, int state_len,
          gsl_vector *ini_state, gsl_matrix *ini_cov,
          int num_particles, int *angle_mask,
          double alpha, double beta, double kappa)
{
    est_estimator_t *state = calloc (1, sizeof (*state)) ;

    state->est_method = est_method;
    state->state_len = state_len;

    // common paprams to all
    state->mu = gsl_vector_calloc (state->state_len);
    state->Sigma = gsl_matrix_calloc (state->state_len, state->state_len);

    //init the state and covariance
    gsl_vector_memcpy (state->mu, ini_state);
    gsl_matrix_memcpy (state->Sigma, ini_cov);

    // defaults unless specificaly set
    state->num_particles = 0;
    state->angle_mask = NULL;
    state->particles = NULL;

    // both EKF and UKF
    if (EST_EKF == state->est_method || EST_UKF == state->est_method)
    {

    }

    // both UKF and PF
    if (EST_UKF == state->est_method || EST_PF == state->est_method)
    {
        state->angle_mask = calloc (1, state->state_len * sizeof (int));
        memcpy (state->angle_mask, angle_mask, state->state_len * sizeof (int));
    }

    // EKF ONLY
    if (EST_EKF == state->est_method)
    {

    }

    // UKF ONLY
    if (EST_UKF == state->est_method)
    {
        state->ukf_weights.alpha = alpha;
        state->ukf_weights.beta = beta;
        state->ukf_weights.kappa = kappa;
    }

    // PF ONLY
    if (EST_PF == state->est_method)
    {
        state->num_particles = num_particles;
        state->particles = gsl_matrix_calloc (state->state_len, num_particles);
        // randomly sample particles based on ini_state and ini_cov
        gsl_rng *rng = gslu_rand_rng_alloc ();
        gslu_rand_gaussian_matrix (rng, state->particles, state->mu, state->Sigma, NULL);
        gsl_rng_free (rng);
    }


    // threshold (in percentage ie 0.99) to test innovation before accepting measurement
    state->last_utime = 0;
    state->last_nis = 0;
    state->last_mahal_innov_passed = 0;
    state->last_nu = NULL;
    state->last_z = NULL;

    return state;
}

// init the estimeation engine: specific for each
est_estimator_t *
est_init_ekf (int state_len, gsl_vector *ini_state, gsl_matrix *ini_cov)
{
    return est_init (EST_EKF, state_len, ini_state, ini_cov, 0, NULL,
                     0, 0, 0);
}

est_estimator_t *
est_init_ukf (int state_len, gsl_vector *ini_state, gsl_matrix *ini_cov,
              int *angle_mask, double alpha, double beta, double kappa)
{
    //check the incoming ukf weight params
    if (!(0 < alpha && alpha <= 1))
        ERROR ("ERROR: alpha must be: 0 < alpha <= 1");
    if (!(0 <= beta))
        ERROR ("ERROR: beta must be: 0 <= beta");
    if (!(0 <= kappa))
        ERROR ("ERROR: kappa must be: 0 <= kappa");

    return est_init (EST_UKF, state_len, ini_state, ini_cov, 0, angle_mask,
                     alpha, beta, kappa);
}

est_estimator_t *
est_init_pf (int state_len, gsl_vector *ini_state, gsl_matrix *ini_cov,
             int num_particles, int *angle_mask)
{
    return est_init (EST_PF, state_len, ini_state, ini_cov, num_particles, angle_mask,
                     0, 0, 0);
}

void
est_init_ds (est_estimator_t *estimator, int max_delayed_states, int delayed_state_len)
{
    estimator->max_delayed_states = max_delayed_states;
    estimator->delayed_state_len = delayed_state_len;
    estimator->delayed_states_cnt = 1;
}

// free est_estimator_t structure
void
est_free (est_estimator_t *state)
{
    // free members of structure
    gsl_vector_free (state->mu);
    gsl_matrix_free (state->Sigma);
    if (NULL != state->angle_mask)
        free (state->angle_mask);

    gslu_matrix_free (state->particles);

    // free main structure
    free (state);
}

// prediction step ---------------------------------------------------
void
est_predict (est_estimator_t *state, est_pred_t *pred)
{
    // make sure the pred_t structure is setup correctly
    if (check_pred_t (state, pred))
        return;

    switch (state->est_method)
    {
    case EST_EKF:
        ekf_predict (state, pred);
        break;
    case EST_UKF:
        ukf_predict (state, pred);
        break;
    case EST_PF:
        pf_predict (state, pred);
        break;
    }
}

void
est_predict_ds (est_estimator_t *state, est_pred_t *pred, int do_augment)
{
    // make sure the pred_t structure is setup correctly
    if (check_pred_t(state, pred))
        return;

    switch (state->est_method)
    {
    case EST_EKF:
        ekf_predict_ds (state, pred, do_augment);
        break;
    case EST_UKF:
        ERROR ("est_predict_ds() not implemented for UKF");
        break;
    case EST_PF:
        ERROR ("est_predict_ds() not implemented for PF");
        break;
    }
}

void
est_marginalize_ds (est_estimator_t *state, int ds_ind)
{
    switch (state->est_method)
    {
    case EST_EKF:
        ekf_marginalize_d (state, ds_ind);
        break;
    case EST_UKF:
        ERROR ("est_predict_ds() not implemented for UKF");
        break;
    case EST_PF:
        ERROR ("est_predict_ds() not implemented for PF");
        break;
    }
}


// correction step -------------------------------------------------
void
est_correct (est_estimator_t *state, est_meas_t *meas)
{
    // make sure the meas_t structure is setup correctly
    if (check_meas_t(state, meas))
        return;

    switch (state->est_method)
    {
    case EST_EKF:
        ekf_correct (state, meas);
        break;
    case EST_UKF:
        ukf_correct (state, meas);
        break;
    case EST_PF:
        pf_correct (state, meas);
        break;
    }
}

void
est_correct_ds (est_estimator_t *state, est_meas_t *meas, int ds_ind)
{
    // make sure the meas_t structure is setup correctly
    if (check_meas_t (state, meas))
        return;

    switch (state->est_method)
    {
    case EST_EKF:
        ekf_correct_ds (state, meas, ds_ind);
        break;
    case EST_UKF:
        ERROR ("est_correct_ds() not implemented for UKF");
        break;
    case EST_PF:
        ERROR ("est_correct_ds() not implemented for PF");
        break;
    }
}

//==============================================================================
// HELPER FUNCTIONS
//==============================================================================
int
check_pred_t (est_estimator_t *state, est_pred_t *pred_t)
{
    int error = 0;

    if (EST_EKF == state->est_method && NULL == pred_t->pmf_ekf)
        ERROR ("ERROR: Cannot use EKF when pred_t->pmf_ekf == NULL \n");
    else if (EST_UKF == state->est_method && NULL == pred_t->pmf_ukf)
        ERROR ("ERROR: Cannot use UKF when pred_t->pmf_ukf == NULL \n");
    else if (EST_PF == state->est_method && NULL == pred_t->pmf_pf)
        ERROR ("ERROR: Cannot use PF when pred_t->pmf_pf == NULL \n");

    if (error)
        return -1;
    else
        return 0;
}

int
check_meas_t (est_estimator_t *state, est_meas_t *meas_t)
{
    int error = 0;

    if (EST_EKF == state->est_method && NULL == meas_t->omf_ekf)
        ERROR ("ERROR: Cannot use EKF when meas_t->omf_ekf == NULL \n");
    else if (EST_UKF == state->est_method && NULL == meas_t->omf_ukf)
        ERROR ("ERROR: Cannot use UKF when meas_t->omf_ukf == NULL \n");
    else if (EST_PF == state->est_method && NULL == meas_t->omf_pf)
        ERROR ("ERROR: Cannot use PF when meas_t->omf_pf == NULL \n");

    if ((EST_UKF == state->est_method || EST_PF == state->est_method)
            && NULL == meas_t->angle_mask)
        ERROR ("ERROR: Cannot use UKF or PF when meas_t->angle_mask == NULL \n");

    if (NULL == meas_t->z)
        ERROR ("ERROR: meas_t->z == NULL \n");
    if (NULL == meas_t->R)
        ERROR ("ERROR: meas_t->R == NULL \n");

    if (error)
        return -1;
    else
        return 0;
}
