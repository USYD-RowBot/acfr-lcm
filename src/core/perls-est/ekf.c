#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_cdf.h>

#include "perls-math/gsl_util.h"

#include "error.h"
#include "ekf.h"

void
ekf_predict (est_estimator_t *state, est_pred_t *pred)
{
    // allocate variables for returns
    gsl_vector *x_bar = gsl_vector_calloc (state->state_len);  // updated state vector
    gsl_matrix *F = gsl_matrix_calloc (state->state_len, state->state_len);  // process jacobian with respect to state
    gsl_matrix *Q = gsl_matrix_calloc (state->state_len, state->state_len);  // process noise

    // if Q is set in pred_t use it
    if (pred->Q != NULL)
        gsl_matrix_memcpy (Q, pred->Q);

    // call user process model function
    (*pred->pmf_ekf) (state, pred->u, pred->dt, x_bar, F, Q, pred->user);

    // perform prediction
    gsl_vector_memcpy (state->mu, x_bar);
    gsl_matrix *cov_tmp = gsl_matrix_calloc (state->state_len, state->state_len);

    //cov_pred = F*Sigma*F' + Q
    gslu_blas_mmmT (cov_tmp, F, state->Sigma, F, NULL);
    gsl_matrix_add (cov_tmp, Q);
    gsl_matrix_memcpy (state->Sigma, cov_tmp);

    // clean up
    gsl_vector_free (x_bar);
    gsl_matrix_free (F);
    gsl_matrix_free (Q);
    gsl_matrix_free (cov_tmp);
}

void
ekf_predict_ds (est_estimator_t *state, est_pred_t *pred, int do_augment)
{
    // allocate variables for returns
    // updated state vector
    gsl_vector *x_bar = gsl_vector_calloc (state->delayed_state_len);
    // process jacobian with respect to state
    gsl_matrix *F = gsl_matrix_calloc (state->delayed_state_len, state->delayed_state_len);
    // process noise
    gsl_matrix *Q = gsl_matrix_calloc (state->delayed_state_len, state->delayed_state_len);

    // if Q is set in pred_t use it
    if (pred->Q != NULL)
        gsl_matrix_memcpy (Q, pred->Q);

    // call user process model function
    (*pred->pmf_ekf) (state, pred->u, pred->dt, x_bar, F, Q, pred->user);

// looks like F is correct
//gslu_matrix_printf (F, "F");

    int old_inc_start = 0;
    int state_len_aug = 0;
    int state_len_old_inc = 0;
    if (do_augment)
    {
        // find new state length (if augmenting and have not reached max_delayed_states)
        if (-1 == state->max_delayed_states || state->delayed_states_cnt < state->max_delayed_states)
            state->delayed_states_cnt++;

        old_inc_start = 0;
    }
    else
        old_inc_start = state->delayed_state_len;

    state_len_aug = state->delayed_states_cnt * state->delayed_state_len;
    state_len_old_inc = state_len_aug - state->delayed_state_len; //the length of the old state to include

    // allocate new augmented mu and covariance
    gsl_vector *mu_aug = gsl_vector_calloc (state_len_aug);
    gsl_matrix *Sigma_aug = gsl_matrix_calloc (state_len_aug, state_len_aug);

//gslu_vector_printf (state->mu, "mu_start");
//gslu_matrix_printf (state->Sigma, "Sigma_start");


    // fill mu_aug
    // new augmented pose
    gsl_vector_view mu_aug_v = gsl_vector_subvector (mu_aug, 0 ,state->delayed_state_len);
    gsl_vector_memcpy (&mu_aug_v.vector, x_bar);

    if (state_len_old_inc > 0)
    {
        // old poses
        mu_aug_v = gsl_vector_subvector (mu_aug, state->delayed_state_len, state_len_old_inc);
        gsl_vector_view mu_v = gsl_vector_subvector (state->mu, old_inc_start, state_len_old_inc);
        gsl_vector_memcpy (&mu_aug_v.vector, &mu_v.vector);
    }

//gslu_vector_printf (mu_aug, "mu_end");

    // fill augmented covariance
    gsl_matrix_view Sigma_aug_v;
    gsl_matrix_view Sigma_v;

    //upper left = F*Sigma*F' + Q, the new pose covariance
    Sigma_aug_v = gsl_matrix_submatrix(Sigma_aug,
                                       0, 0,
                                       state->delayed_state_len, state->delayed_state_len);
    Sigma_v = gsl_matrix_submatrix (state->Sigma,
                                    0, 0,
                                    state->delayed_state_len, state->delayed_state_len);
    gslu_blas_mmmT (&Sigma_aug_v.matrix, F, &Sigma_v.matrix, F, NULL);
    gsl_matrix_add (&Sigma_aug_v.matrix, Q);

//gslu_matrix_printf (Sigma_aug, "Sigma 1");

    if (state_len_old_inc > 0)
    {
        //lower right, the old poses covariance
        Sigma_aug_v = gsl_matrix_submatrix(Sigma_aug,
                                           state->delayed_state_len, state->delayed_state_len,
                                           state_len_old_inc, state_len_old_inc);
        Sigma_v = gsl_matrix_submatrix (state->Sigma,
                                        old_inc_start, old_inc_start,
                                        state_len_old_inc, state_len_old_inc);
        gsl_matrix_memcpy (&Sigma_aug_v.matrix, &Sigma_v.matrix);

//gslu_matrix_printf (Sigma_aug, "Sigma 2");

        //upper right cross covariance, new pose with old poses
        Sigma_aug_v = gsl_matrix_submatrix(Sigma_aug,
                                           0, state->delayed_state_len,
                                           state->delayed_state_len, state_len_old_inc);
        Sigma_v = gsl_matrix_submatrix (state->Sigma,
                                        0, old_inc_start,
                                        state->delayed_state_len, state_len_old_inc);
        gslu_blas_mm (&Sigma_aug_v.matrix, F, &Sigma_v.matrix);

//gslu_matrix_printf (Sigma_aug, "Sigma 3");

        //lower left cross covariance, new old poses with new pose
        Sigma_aug_v = gsl_matrix_submatrix(Sigma_aug,
                                           state->delayed_state_len, 0,
                                           state_len_old_inc, state->delayed_state_len);
        Sigma_v = gsl_matrix_submatrix (state->Sigma,
                                        old_inc_start, 0,
                                        state_len_old_inc, state->delayed_state_len);
        gslu_blas_mmT (&Sigma_aug_v.matrix, &Sigma_v.matrix, F);
    }


    //free old mu and sigma and reassign pointer
    gsl_vector_free (state->mu);
    gsl_matrix_free (state->Sigma);
    state->mu = mu_aug;
    state->Sigma = Sigma_aug;

    // save increased state length
    state->state_len = state_len_aug;

//gslu_matrix_printf (state->Sigma, "Sigma_end");

    // clean up
    gsl_vector_free (x_bar);
    gsl_matrix_free (F);
    gsl_matrix_free (Q);
}

void
ekf_marginalize_d (est_estimator_t * state, int ds_ind)
{
    if (ds_ind >= state->delayed_states_cnt)
    {
        ERROR ("ERROR: Marginalize index %d > delayed state cnt %d", ds_ind, state->delayed_states_cnt);
        return;
    }

    // allocate new augmented mu and covariance
    int new_state_len = (state->delayed_states_cnt - 1) * state->delayed_state_len;
    gsl_vector *mu_new = gsl_vector_calloc (new_state_len);
    gsl_matrix *Sigma_new= gsl_matrix_calloc (new_state_len, new_state_len);

    // inds
    int before_len = ds_ind*state->delayed_state_len;
    int after_start_old = (ds_ind+1)*state->delayed_state_len;
    int after_start_new = ds_ind*state->delayed_state_len;
    int after_len = (state->delayed_states_cnt-(ds_ind+1))*state->delayed_state_len;


    // update mu
    gsl_vector_view mu_before = gsl_vector_subvector (state->mu,
                                0,
                                before_len);
    gslu_vector_set_subvector (mu_new, 0, &mu_before.vector);
    if (after_len > 0)
    {
        gsl_vector_view mu_after = gsl_vector_subvector (state->mu,
                                   after_start_old,
                                   after_len);
        gslu_vector_set_subvector (mu_new, after_start_new, &mu_after.vector);
    }

    // update Sigma
    // upper left
    gsl_matrix_view upper_left = gsl_matrix_submatrix (state->Sigma,
                                 0, 0,
                                 before_len, before_len);
    gslu_matrix_set_submatrix (Sigma_new, 0, 0, &upper_left.matrix);

    if (after_len > 0)
    {
        // upper right
        gsl_matrix_view upper_right = gsl_matrix_submatrix (state->Sigma,
                                      0, after_start_old,
                                      before_len, after_len);
        gslu_matrix_set_submatrix (Sigma_new, 0, after_start_new, &upper_right.matrix);


        // lower left
        gsl_matrix_view lower_left = gsl_matrix_submatrix (state->Sigma,
                                     after_start_old, 0,
                                     after_len, before_len);
        gslu_matrix_set_submatrix (Sigma_new, after_start_new, 0, &lower_left.matrix);


        // lower right
        gsl_matrix_view lower_right = gsl_matrix_submatrix (state->Sigma,
                                      after_start_old, after_start_old,
                                      after_len, after_len);
        gslu_matrix_set_submatrix (Sigma_new, after_start_new, after_start_new, &lower_right.matrix);
    }

//printf ("len_before=%d, len_after=%d \n", state->mu->size, mu_new->size);
//gslu_vector_printf (state->mu, "mu before");
//gslu_vector_printf (mu_new, "mu after");
    //free old mu and sigma and reassign pointer
    gsl_vector_free (state->mu);
    gsl_matrix_free (state->Sigma);
    state->mu = mu_new;
    state->Sigma = Sigma_new;

    state->delayed_states_cnt--;
    state->state_len = new_state_len;
}

void
ekf_correct_raw (est_estimator_t *state, est_meas_t *meas,
                 const gsl_vector *nu, const gsl_matrix *H)
{
    // perform correction -----------------------------------------------------
    // TODO: make sure efficient and numericaly stable, enforce symmetry
    // calculate kalman gain
    // K = Sigma*H'*inv(H*Sigma*H' + R)';
    gsl_matrix *K = gsl_matrix_calloc (state->state_len, meas->z->size);
    gsl_matrix *S = gsl_matrix_calloc (meas->z->size, meas->z->size);
    gsl_matrix *Sinv = gsl_matrix_calloc (meas->z->size, meas->z->size);
    gslu_blas_mmmT (S, H, state->Sigma, H, NULL);
    gsl_matrix_add (S, meas->R);
    gslu_matrix_inv (Sinv, S);
    gslu_blas_mmTmT (K, state->Sigma, H, Sinv, NULL);

    // calculate nis
    // nis = nu'*inv(S)*nu;
    double nis = gslu_blas_vTmv (nu, Sinv ,nu);


    // don't use mahal test if use_innov_mahal_test == 0
    // or passed mahal test
    if ( !meas->use_innov_mahal_test ||
            nis < gsl_cdf_chisq_Pinv (meas->innov_mahal_test_thresh, nu->size))
    {

        // update covariance (Joseph form)
        // Sigma = (I - K*H)*Sigma*(I - K*H)' + K*R*K';
        gsl_matrix * tmp4 = gsl_matrix_calloc (state->state_len, state->state_len);
        gsl_matrix * tmp5 = gsl_matrix_calloc (state->state_len, state->state_len);
        gsl_matrix_set_identity (tmp5);
        gslu_blas_mm (tmp4, K, H); // K*H
        gsl_matrix_sub (tmp5, tmp4); // (I - K*H)
        gslu_blas_mmmT (tmp4, tmp5, state->Sigma, tmp5, NULL); // (I - K*H)*Sigma*(I - K*H)'
        gslu_blas_mmmT (state->Sigma, K, meas->R, K, NULL); // K*R*K'
        gsl_matrix_add (state->Sigma, tmp4); // (I - K*H)*Sigma*(I - K*H)' + K*R*K'

        // update mean
        // mu = mu + K*nu;
        gsl_vector * tmp3 = gsl_vector_calloc (state->state_len);
        gslu_blas_mv (tmp3, K, nu);
        gsl_vector_add (state->mu, tmp3);

        // clean up
        gsl_vector_free (tmp3);
        gsl_matrix_free (tmp4);
        gsl_matrix_free (tmp5);

        state->last_mahal_innov_passed = 1;
    }
    else
    {
        //printf ("Innovation mahalanobis distance threshold exceded for %s \n", meas->id_str);
        //printf("MAHAL_THRESH = %f, nis = %f, thresh = %f, %s\n",
        //    meas->innov_mahal_test_thresh,
        //    nis,
        //    gsl_cdf_chisq_Pinv (meas->innov_mahal_test_thresh, nu->size),
        //    meas->id_str);
        state->last_mahal_innov_passed = 0;
    }
    // -------------------------------------------------------------------------

    // update debug information in the estimator struct
    gslu_vector_free (state->last_z);
    gslu_vector_free (state->last_nu);
    state->last_z = gsl_vector_calloc (meas->z->size);
    state->last_nu = gsl_vector_calloc (meas->z->size);
    gsl_vector_memcpy (state->last_z, meas->z);
    gsl_vector_memcpy (state->last_nu, nu);
    memcpy (state->last_id_str, meas->id_str, sizeof(char)*EST_MEAS_ID_STR_MAX_LEN);
    state->last_nis = nis;
    state->last_utime = meas->utime;

    // clean up
    gsl_matrix_free (K);
    gsl_matrix_free (S);
    gsl_matrix_free (Sinv);
}

void
ekf_correct (est_estimator_t *state, est_meas_t *meas)
{
    // allocate variables for returns
    gsl_vector * nu = gsl_vector_calloc (meas->z->size);  // output: state update vector
    gsl_matrix * H = gsl_matrix_calloc (meas->z->size, state->state_len);  // output: jacobian with respect to state
    // TODO, currently assumes R is allocated outside, could check and allocate it here if not

    // call user process model function
    (*meas->omf_ekf) (state, meas->z, meas->index_map, meas->R, nu, H, meas->user);

    // **run actual correction**
    ekf_correct_raw (state, meas, nu, H);

    // clean up
    gsl_vector_free (nu);
    gsl_matrix_free (H);
}

void
ekf_correct_ds (est_estimator_t *state, est_meas_t *meas, int ds_ind)
{
    if (ds_ind >= state->delayed_states_cnt)
    {
        ERROR ("ERROR: Delayed state index %d > delayed state cnt %d", ds_ind, state->delayed_states_cnt);
        return;
    }

    int ds_start = state->delayed_state_len * ds_ind;

    // allocate variables for returns of delayed state omf
    gsl_vector * nu = gsl_vector_calloc (meas->z->size);  // output: state update vector
    gsl_matrix * H = gsl_matrix_calloc (meas->z->size, state->state_len);
    gsl_matrix_view H_ds = gsl_matrix_submatrix (H, 0, ds_start, meas->z->size, state->delayed_state_len); // capture only delayed state part of jacobian

    // spoof state to only view delayed state
    gsl_vector * mu_ds = gslu_vector_get_subvector_alloc (state->mu, ds_start, state->delayed_state_len);
    gsl_matrix * Sigma_ds = gsl_matrix_calloc (state->delayed_state_len, state->delayed_state_len);
    gsl_matrix_view Sigma_ds_v = gsl_matrix_submatrix (state->Sigma, ds_start, ds_start,
                                 state->delayed_state_len, state->delayed_state_len);
    gsl_matrix_memcpy (Sigma_ds, &Sigma_ds_v.matrix);

    est_estimator_t *state_ds = est_init_ekf (state->delayed_state_len, mu_ds, Sigma_ds);

    // call user process model function on spoofed state
    (*meas->omf_ekf) (state_ds, meas->z, meas->index_map, meas->R, nu, &H_ds.matrix, meas->user);

    // **run actual correction**
    ekf_correct_raw (state, meas, nu, H);

    // cleanup
    gsl_vector_free (nu);
    gsl_matrix_free (H);
    gsl_vector_free (mu_ds);
    gsl_matrix_free (Sigma_ds);
    est_free (state_ds);
}
