#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_cdf.h>

#include "perls-math/gsl_util.h"

#include "ukf.h"

void
est_ukf_calc_weights (est_ukf_weights_t *w, int n);

double
est_ukf_mean (const gsl_vector *row, const est_ukf_weights_t *w, const int is_ang);

void
est_ukf_cov (gsl_matrix *Sigma, const gsl_matrix *spp, const gsl_vector *mu,
             const int *is_ang, const est_ukf_weights_t *w);
void
est_ukf_cross_cov (gsl_matrix *Sigma_xz,
                   const gsl_matrix *sp, const gsl_vector *mu_x, const int *is_ang_x,
                   const gsl_matrix *spp, const gsl_vector *mu_z, const int *is_ang_z,
                   const est_ukf_weights_t *w);


void
ukf_predict (est_estimator_t *state, est_pred_t *pred)
{
    gsl_matrix *Q = gsl_matrix_calloc (state->state_len, state->state_len);  // process noise

    // if Q is set in pred_t use it
    if (pred->Q != NULL)
        gsl_matrix_memcpy (Q, pred->Q);

    // get the control noise if it is avaliable
    int u_len = 0;
    gsl_matrix *Sigma_u = NULL;
    if (NULL != pred->u)
    {
        u_len = pred->u->size;

        // if we have a control vector we may need to sample sigma points for its noise
        // two interfaces for user to provide noise callback or matrix in pred_t
        if (NULL != pred->Sigma_u || NULL != pred->unf)
            Sigma_u = gsl_matrix_calloc (u_len, u_len);
        //else no noise associated with control provided, treat it as "perfect"

        // if Sigma_u is set in pred_t use it
        if (NULL != pred->Sigma_u)
            gsl_matrix_memcpy (Sigma_u, pred->Sigma_u);

        // if there is a callback to update Sigma_u at each prediciton call it
        if (NULL != pred->unf)
            (*pred->unf)(state, pred->u, pred->dt, Sigma_u, pred->user);
    }


    // FIND SIGMA POINTS
    // first build the augmented state and covariance
    int x_len = state->state_len;
    int aug_len = 0;
    if (NULL != Sigma_u) //if we have a control noise covariance
        aug_len = x_len + u_len;
    else
        aug_len = x_len;

    // calculate weights
    est_ukf_weights_t *ukf_weights = NULL;
    if (NULL != pred->ukf_weights) //if one is passed in through pred_t use it
        ukf_weights = pred->ukf_weights;
    else // use default
        ukf_weights = &(state->ukf_weights);

    est_ukf_calc_weights (ukf_weights, aug_len);

    gsl_vector *mu_aug = gsl_vector_calloc (aug_len);
    gsl_matrix *Sigma_aug = gsl_matrix_calloc (aug_len, aug_len);

    gslu_vector_set_subvector (mu_aug, 0, state->mu);
    gslu_matrix_set_submatrix (Sigma_aug, 0, 0, state->Sigma);

    // if we have a control noise
    if (NULL != Sigma_u)
    {
        gslu_vector_set_subvector (mu_aug, x_len, pred->u);
        gslu_matrix_set_submatrix (Sigma_aug, x_len, x_len, Sigma_u);
    }

    // cholesky decomp
    gsl_matrix *L = gsl_matrix_calloc (aug_len, aug_len);
    gsl_matrix_memcpy (L, Sigma_aug);
    gslu_linalg_cholesky_decomp_lower (L);

    // weight the sqrt root factor by gamma
    gsl_matrix_scale (L, ukf_weights->gamma);

    int num_sig_pts = 2*aug_len+1;
    gsl_matrix *sig_pts = gsl_matrix_calloc (aug_len, num_sig_pts);
    //sig_pts = [mu_aug | mu_aug+L | mu_aug-L];
    for (int j=0 ; j<num_sig_pts ; j++)
        gslu_matrix_set_column (sig_pts, j, mu_aug);

    gsl_matrix_view pL = gsl_matrix_submatrix (sig_pts, 0, 1, L->size1, L->size2);
    gsl_matrix_add (&pL.matrix, L);
    gsl_matrix_view mL = gsl_matrix_submatrix (sig_pts, 0, 1+L->size2, L->size1, L->size2);
    gsl_matrix_sub (&mL.matrix, L);

    // NOT NEEDED
    // //wrap angular quantities in sigma points
    //for (int i=0 ; i<x_len ; i++) {
    //    if (state->angle_mask[i]) {
    //        for (int j=0 ; j<sig_pts->size2 ; j++) {
    //            gsl_matrix_set (sig_pts, i, j, gslu_math_minimized_angle);
    //        }
    //    }
    //}

    // call user process model function for each sigma point
    gsl_matrix *sig_pts_pred = gsl_matrix_calloc (state->state_len, num_sig_pts);
    for (int j=0 ; j<num_sig_pts ; j++)
    {

        // input views
        gsl_vector_view x_sp = gsl_matrix_subcolumn (sig_pts, j, 0, x_len);
        gsl_vector_view u_sp;
        if (NULL != Sigma_u) //have noisy control vector
            u_sp = gsl_matrix_subcolumn (sig_pts, j, x_len, u_len);
        else
            u_sp = gsl_vector_subvector (pred->u, 0, u_len);

        //output views
        gsl_vector_view x_bar = gsl_matrix_column (sig_pts_pred, j);

        (*pred->pmf_ukf) (state, &x_sp.vector, &u_sp.vector, pred->dt,
                          &x_bar.vector, Q, pred->user);
    }

    // find mean of sigma points, update state->mu
    for (int i=0 ; i < state->state_len ; i++)
    {
        gsl_vector_view row = gsl_matrix_row (sig_pts_pred, i);
        gsl_vector_set (state->mu, i,
                        est_ukf_mean (&row.vector, ukf_weights, state->angle_mask[i]));
    }

    // find covariance of sigma points, update state->Sigma
    est_ukf_cov (state->Sigma, sig_pts_pred, state->mu,
                 state->angle_mask, ukf_weights);

    // add in Q
    gsl_matrix_add (state->Sigma, Q);

//gslu_vector_printf (state->mu , "predict state->mu");
//gslu_matrix_printf (state->Sigma, "predict state->Sigma");

    // clean up
    gslu_matrix_free (Sigma_u);
    gsl_vector_free (mu_aug);
    gsl_matrix_free (Sigma_aug);
    gsl_matrix_free (Q);
    gsl_matrix_free (L);
    gsl_matrix_free (sig_pts);
    gsl_matrix_free (sig_pts_pred);
}

void
ukf_correct (est_estimator_t *state, est_meas_t *meas)
{

//gslu_vector_printf (state->mu , "state->mu start");
//gslu_matrix_printf (state->Sigma, "state->Sigma start");

    // FIND SIGMA POINTS
    // first build the augmented state and covariance
    int z_len = meas->z->size;
    int x_len = state->state_len;
    int aug_len = 0;
    aug_len = x_len + z_len;

    // calculate weights
    est_ukf_weights_t *ukf_weights = NULL;
    if (NULL != meas->ukf_weights) //if one is passed in through meas_t use it
        ukf_weights = meas->ukf_weights;
    else // use default
        ukf_weights = &(state->ukf_weights);

    est_ukf_calc_weights (ukf_weights, aug_len);

    gsl_vector *mu_aug = gsl_vector_calloc (aug_len);
    gsl_matrix *Sigma_aug = gsl_matrix_calloc (aug_len, aug_len);

    gslu_vector_set_subvector (mu_aug, 0, state->mu);
    gslu_matrix_set_submatrix (Sigma_aug, 0, 0, state->Sigma);

    //gslu_vector_set_subvector (mu_aug, x_len, 0); //assumes zero mean noise
    gslu_matrix_set_submatrix (Sigma_aug, x_len, x_len, meas->R);


//gslu_vector_printf (meas->z, "z");
//gslu_vector_printf (mu_aug, "mu_aug");
//gslu_matrix_printf (Sigma_aug, "Sigma_aug");

    // cholesky decomp
    gsl_matrix *L = gsl_matrix_calloc (aug_len, aug_len);
    gsl_matrix_memcpy (L, Sigma_aug);
    gslu_linalg_cholesky_decomp_lower (L);
    // weight the sqrt root factor by gamma
    gsl_matrix_scale (L, ukf_weights->gamma);
//gslu_matrix_printf (L, "L");

    int num_sig_pts = 2*aug_len+1;
    gsl_matrix *sig_pts = gsl_matrix_calloc (aug_len, num_sig_pts);
    //sig_pts = [mu_aug | mu_aug+L | mu_aug-L];
    for (int j=0 ; j<num_sig_pts ; j++)
        gslu_matrix_set_column (sig_pts, j, mu_aug);

    gsl_matrix_view pL = gsl_matrix_submatrix (sig_pts, 0, 1, L->size1, L->size2);
    gsl_matrix_add (&pL.matrix, L);
    gsl_matrix_view mL = gsl_matrix_submatrix (sig_pts, 0, 1+L->size2, L->size1, L->size2);
    gsl_matrix_sub (&mL.matrix, L);

    // NOT NEEDED
    //// wrap angular quantities in sigma points
    //for (int i=0 ; i<x_len ; i++) {
    //    if (state->angle_mask[i]) {
    //        for (int j=0 ; j<sig_pts->size2 ; j++) {
    //            gsl_matrix_set (sig_pts, i, j, gslu_math_minimized_angle);
    //        }
    //    }
    //}
    //for (int i=x_len ; i<aug_len ; i++) {
    //    if (meas->angle_mask[i-x_len]) {
    //        for (int j=0 ; j<sig_pts->size2 ; j++) {
    //            gsl_matrix_set (sig_pts, i, j, gslu_math_minimized_angle);
    //        }
    //    }
    //}

    // call user observation model function for each sigma point
    gsl_matrix *sig_pts_pred = gsl_matrix_calloc (z_len, num_sig_pts);
    for (int j=0 ; j<num_sig_pts ; j++)
    {
        // input views
        gsl_vector_view x_sp = gsl_matrix_subcolumn (sig_pts, j, 0, x_len);
        gsl_vector_view zn_sp = gsl_matrix_subcolumn (sig_pts, j, x_len, z_len);

        //output views
        gsl_vector_view z_bar = gsl_matrix_column (sig_pts_pred, j);

        (*meas->omf_ukf) (state, &x_sp.vector, meas->index_map, meas->R,
                          &z_bar.vector, meas->user);

        //add in noise sigma point
        gsl_vector_add (&z_bar.vector, &zn_sp.vector);
    }

    // find mean of sigma points to find zhat
    gsl_vector *z_bar = gsl_vector_calloc (z_len);
    for (int i=0 ; i < z_len ; i++)
    {
        gsl_vector_view row = gsl_matrix_row (sig_pts_pred, i);
        gsl_vector_set (z_bar, i,
                        est_ukf_mean (&row.vector, ukf_weights, meas->angle_mask[i]));
    }
//gslu_vector_printf (z_bar, "z_bar");
//gslu_matrix_printf (sig_pts_pred, "sig_pts_pred");
//printf ("w_m_0=%f w_m_i=%f w_c_0=%f w_c_i=%f\n",ukf_weights->w_m_0, ukf_weights->w_m_i , ukf_weights->w_c_0, ukf_weights->w_c_i);
    // find measurement covariance
    gsl_matrix *S = gsl_matrix_calloc (z_len,z_len);
    est_ukf_cov (S, sig_pts_pred, z_bar,
                 meas->angle_mask, ukf_weights);

//gslu_matrix_printf (S, "S");
//gslu_matrix_printf (sig_pts, "sig_pts");


    // find cross covariance
    gsl_matrix *Sigma_xz = gsl_matrix_calloc (x_len,z_len);
    est_ukf_cross_cov (Sigma_xz, sig_pts, state->mu, state->angle_mask,
                       sig_pts_pred, z_bar, meas->angle_mask,
                       ukf_weights);

//gslu_matrix_printf (Sigma_xz, "Sigma_xz");

    // perform correction -----------------------------------------------------
    // TODO: make sure efficient and numericaly stable, enforce symmetry
    // calculate kalman gain
    //K = Sigma_xz*inv(S);
    gsl_matrix *K = gsl_matrix_calloc (state->state_len, meas->z->size);
    gsl_matrix *Sinv = gsl_matrix_calloc (meas->z->size, meas->z->size);
    gslu_matrix_inv (Sinv, S);
    gslu_blas_mm (K, Sigma_xz, Sinv);

    // calculate innovation
    gsl_vector *nu = gsl_vector_calloc (meas->z->size);
    gsl_vector_memcpy (nu, meas->z);
    gsl_vector_sub (nu, z_bar);
    // wrap innovation if angular
    for (int i=0 ; i < z_len ; i++)
    {
        if (meas->angle_mask[i])
            gsl_vector_set (nu, i, gslu_math_minimized_angle (gsl_vector_get (nu, i)));
    }

    // calculate nis
    // nis = nu'*inv(S)*nu;
    double nis = gslu_blas_vTmv (nu, Sinv ,nu);

    // don't use mahal test if use_innov_mahal_test == 0
    // or passed mahal test
    if ( !meas->use_innov_mahal_test ||
            nis < gsl_cdf_chisq_Pinv (meas->innov_mahal_test_thresh, nu->size))
    {

        // update covariance (Joseph form)
        // Sigma = Sigma - K*S*K';
        gsl_matrix *tmp4 = gsl_matrix_calloc (state->state_len, state->state_len);
        gslu_blas_mmmT (tmp4, K, S, K, NULL); //
        gsl_matrix_sub (state->Sigma, tmp4); // - K*S*K'

        // update mean
        // mu = mu + K*nu;
        gsl_vector *tmp3 = gsl_vector_calloc (state->state_len);
        gslu_blas_mv (tmp3, K, nu);
        gsl_vector_add (state->mu, tmp3);

        // clean up
        gsl_vector_free (tmp3);
        gsl_matrix_free (tmp4);

        state->last_mahal_innov_passed = 1;

    }
    else
    {
        //printf ("Innovation mahalanobis distance threshold exceded for %s \n", meas->id_str);
        //printf("MAHAL_THRESH = %f, nis = %f, thresh = %f, %s \n",
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

//gslu_matrix_printf (K, "K");
//gslu_vector_printf (nu , "nu");
//gslu_vector_printf (state->mu , "correct state->mu");
//gslu_matrix_printf (state->Sigma, "correct state->Sigma");

    // clean up
    gsl_vector_free (mu_aug);
    gsl_matrix_free (Sigma_aug);
    gsl_matrix_free (L);
    gsl_matrix_free (sig_pts);
    gsl_matrix_free (sig_pts_pred);
    gsl_vector_free (z_bar);
    gsl_matrix_free (S);
    gsl_matrix_free (Sigma_xz);
    gsl_vector_free (nu);
    gsl_matrix_free (K);
    gsl_matrix_free (Sinv);
}

// helper functions -----------------------------------------------------------

// find the mean of a sigma point row, taking into consideration if it is an angle
// if it is an angle we take the mean by projecting points onto the unit circle
// averaging and then converting back to an angle
double
est_ukf_mean (const gsl_vector *row, const est_ukf_weights_t *w, const int is_ang)
{
    double mean = 0;
    if (is_ang)
    {
        double x_mean = 0;
        double y_mean = 0;

        // center point is first in list
        x_mean = w->w_m_0 * cos (gsl_vector_get (row, 0));
        y_mean = w->w_m_0 * sin (gsl_vector_get (row, 0));
        // noncentral points in remainder of list
        for (int j=1 ; j < row->size ; j++ )
        {
            x_mean += w->w_m_i * cos (gsl_vector_get (row, j));
            y_mean += w->w_m_i * sin (gsl_vector_get (row, j));
        }
        mean = atan2(y_mean, x_mean);
    }
    else
    {
        // center point is first in list
        mean = w->w_m_0 * gsl_vector_get (row, 0);
        // noncentral points in remainder of list
        for (int j=1 ; j < row->size ; j++ )
            mean += w->w_m_i * gsl_vector_get (row, j);
    }

    return mean;
}


// find the covariance of the sigma points
void
est_ukf_cov (gsl_matrix *Sigma, const gsl_matrix *spp, const gsl_vector *mu,
             const int *is_ang, const est_ukf_weights_t *w)
{
    int len;
    len = spp->size1;
    gsl_matrix_set_zero (Sigma);
    gsl_matrix *S_tmp = gsl_matrix_calloc (Sigma->size1, Sigma->size2);
    gsl_vector *diff = gsl_vector_calloc (len);

    // first column for center point
    for (int i=0 ; i < len ; i++)
    {
        gsl_vector_set (diff, i, gsl_matrix_get(spp, i, 0) - gsl_vector_get(mu, i));
        // if angular wrap the difference
        if (is_ang[i])
            gsl_vector_set (diff, i, gslu_math_minimized_angle (gsl_vector_get (diff, i)));
    }
    // Sigma = w_c_0*(diff)*(diff)';
    gsl_matrix_set_zero (S_tmp);
    gslu_blas_vvT (S_tmp, diff, diff);
    gsl_matrix_scale (S_tmp, w->w_c_0);
    gsl_matrix_add (Sigma, S_tmp);

    // remaining columns for noncentral points
    for (int j=1 ; j < spp->size2 ; j++)
    {
        for (int i=0 ; i < len ; i++)
        {
            gsl_vector_set (diff, i, gsl_matrix_get (spp, i, j) - gsl_vector_get (mu, i));
            // if angular wrap the difference
            if (is_ang[i])
                gsl_vector_set (diff, i, gslu_math_minimized_angle (gsl_vector_get (diff, i)));
        }

        // Sigma = Sigma + w_c_i*(diff)*(diff)';
        gsl_matrix_set_zero (S_tmp);
        gslu_blas_vvT (S_tmp, diff, diff);
        gsl_matrix_scale (S_tmp, w->w_c_i);
        gsl_matrix_add (Sigma, S_tmp);
    }

    // clean up
    gsl_matrix_free (S_tmp);
    gsl_vector_free (diff);

}

// find the cross covariance of the sigma points
void
est_ukf_cross_cov (gsl_matrix *Sigma_xz,
                   const gsl_matrix *sp, const gsl_vector *mu_x, const int *is_ang_x,
                   const gsl_matrix *spp, const gsl_vector *mu_z, const int *is_ang_z,
                   const est_ukf_weights_t *w)
{
    int x_len, z_len;
    x_len = mu_x->size;
    z_len = mu_z->size;

    gsl_matrix_set_zero (Sigma_xz);
    gsl_matrix *S_tmp = gsl_matrix_calloc (x_len, z_len);
    gsl_vector *diff_x = gsl_vector_calloc (x_len);
    gsl_vector *diff_z = gsl_vector_calloc (z_len);

    // first column for center point
    for (int i=0 ; i < x_len ; i++)
    {
        gsl_vector_set (diff_x, i, gsl_matrix_get(sp, i, 0) - gsl_vector_get(mu_x, i));
        // if angular wrap the difference
        if (is_ang_x[i])
            gsl_vector_set (diff_x, i, gslu_math_minimized_angle (gsl_vector_get (diff_x, i)));
    }
    for (int i=0 ; i < z_len ; i++)
    {
        gsl_vector_set (diff_z, i, gsl_matrix_get(spp, i, 0) - gsl_vector_get(mu_z, i));
        // if angular wrap the difference
        if (is_ang_z[i])
            gsl_vector_set (diff_z, i, gslu_math_minimized_angle (gsl_vector_get (diff_z, i)));
    }

    // Sigma = w_c_0*(diff)*(diff)';
    gsl_matrix_set_zero (S_tmp);
    gslu_blas_vvT (S_tmp, diff_x, diff_z);
    gsl_matrix_scale (S_tmp, w->w_c_0);
    gsl_matrix_add (Sigma_xz, S_tmp);

    // remaining columns for noncentral points
    for (int j=1 ; j < spp->size2 ; j++)
    {
        for (int i=0 ; i < x_len ; i++)
        {
            gsl_vector_set (diff_x, i, gsl_matrix_get(sp, i, j) - gsl_vector_get(mu_x, i));
            // if angular wrap the difference
            if (is_ang_x[i])
                gsl_vector_set (diff_x, i, gslu_math_minimized_angle (gsl_vector_get (diff_x, i)));
        }

        for (int i=0 ; i < z_len ; i++)
        {
            gsl_vector_set (diff_z, i, gsl_matrix_get(spp, i, j) - gsl_vector_get(mu_z, i));
            // if angular wrap the difference
            if (is_ang_z[i])
                gsl_vector_set (diff_z, i, gslu_math_minimized_angle (gsl_vector_get (diff_z, i)));
        }

        // Sigma = Sigma + w_c_i*(diff)*(diff)';
        gsl_matrix_set_zero (S_tmp);
        gslu_blas_vvT (S_tmp, diff_x, diff_z);
        gsl_matrix_scale (S_tmp, w->w_c_i);
        gsl_matrix_add (Sigma_xz, S_tmp);

//gslu_vector_printf (diff_x, "diff_x insite");
//gslu_vector_printf (diff_z, "diff_z insite");
//gslu_matrix_printf (S_tmp, "S_tmp insite");
//gslu_matrix_printf (Sigma_xz, "Sigma_xz insite");
    }

    // clean up
    gsl_matrix_free (S_tmp);
    gsl_vector_free (diff_x);
    gsl_vector_free (diff_z);
}

// calculate the ukf weights given alpha, beta and kappa
void
est_ukf_calc_weights (est_ukf_weights_t *w, int n)
{
    w->n = n;
    w->lambda = (w->alpha)*(w->alpha)*(n + (w->kappa)) - n;
    w->gamma = sqrt(n + (w->lambda));

    w->w_m_0 = (w->lambda) / (n + (w->lambda));
    w->w_m_i = 1 / (2 * (n + (w->lambda)));
    w->w_c_0 = w->w_m_0 + (1 - (w->alpha)*(w->alpha) + (w->beta));
    w->w_c_i = w->w_m_i;
}
