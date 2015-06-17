#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/gsl_util.h"

#include "pf.h"

void
est_pf_resample (gsl_matrix *particles, gsl_vector *weights);
void
est_pf_mu_sigma (gsl_vector *mu, gsl_matrix *Sigma, const gsl_matrix *particles, const int *is_ang);
void
est_pf_vector_normalize (gsl_vector *a);

void
pf_predict (est_estimator_t *state, est_pred_t *pred)
{
    // TODO: in the future we might want to provide a batch interface to the user
    // to update all the particles at once, curently we do one at a time so interface
    // is more similar to the EKF/UKF versions

    // call the process model for each particle
    gsl_vector *x_bar = gsl_vector_calloc (state->state_len);

    for (int i=0 ; i<state->num_particles ; i++)
    {
        // call the prediction callback (output x_bar)
        gsl_vector_set_zero (x_bar);
        gsl_vector_view x_p = gsl_matrix_column (state->particles, i);
        (*pred->pmf_pf) (state, &x_p.vector, pred->u, pred->dt, pred->Q, x_bar, pred->user);
        //overwrite old particle
        gsl_vector_memcpy (&x_p.vector, x_bar);
    }

    // clean up
    gsl_vector_free (x_bar);

    // update the sample based mean and covariance
    est_pf_mu_sigma (state->mu, state->Sigma, state->particles, state->angle_mask);
}

void
pf_correct (est_estimator_t *state, est_meas_t *meas)
{
    // TODO: in the future we might want to provide a batch interface to the user
    // to update all the particles at once, curently we do one at a time so interface
    // is more similar to the EKF/UKF versions

    // call the prediction callback (output weight)
    gsl_vector *weights = gsl_vector_calloc (state->num_particles);

    for (int i=0 ; i<state->num_particles ; i++)
    {
        double weight = 0;
        gsl_vector_view x_p = gsl_matrix_column (state->particles, i);
        weight = (*meas->omf_pf) (state, &x_p.vector, meas->z, meas->index_map,
                                  meas->R, meas->user);
        gsl_vector_set (weights, i, weight);
    }

    // normalize weights
    est_pf_vector_normalize (weights);

    // update maximum liklihood particle index
    state->i_x_ml = gsl_vector_max_index (weights);

    // resample based on weights
    est_pf_resample (state->particles, weights);

    // update the sample based mean and covariance
    est_pf_mu_sigma (state->mu, state->Sigma, state->particles, state->angle_mask);

    // clean up
    gsl_vector_free (weights);
}

void
est_pf_mu_sigma (gsl_vector *mu, gsl_matrix *Sigma,
                 const gsl_matrix *particles, const int *is_ang)
{
    int state_len = particles->size1;
    int np = particles->size2;

    // find the mean
    for (int i=0 ; i<state_len ; i++)
    {
        double mean = 0;
        gsl_vector_const_view row = gsl_matrix_const_row (particles, i);

        if (is_ang[i])
        {
            double x_sum = 0;
            double y_sum = 0;
            for (int j=0 ; j < np ; j++ )
            {
                x_sum += cos (gsl_vector_get (&row.vector, j));
                y_sum += sin (gsl_vector_get (&row.vector, j));
            }
            mean = atan2 (y_sum/np, x_sum/np);
        }
        else
        {
            for (int j=0 ; j < np ; j++ )
                mean += gsl_vector_get (&row.vector, j);

            mean = mean/np;
        }
        gsl_vector_set (mu, i, mean);
    }

    // find the covariance
    gsl_matrix_set_zero (Sigma);
    gsl_vector *diff = gsl_vector_calloc (state_len);
    gsl_matrix *S_tmp = gsl_matrix_calloc (Sigma->size1, Sigma->size2);
    for (int j=1 ; j<np ; j++)
    {
        for (int i=0 ; i < state_len; i++)
        {
            gsl_vector_set (diff, i, gsl_matrix_get(particles, i, j) - gsl_vector_get(mu, i));
            // if angular wrap the difference
            if (is_ang[i])
                gsl_vector_set (diff, i, gslu_math_minimized_angle (gsl_vector_get (diff, i)));
        }

        gsl_matrix_set_zero (S_tmp);
        gslu_blas_vvT (S_tmp, diff, diff);
        gsl_matrix_add (Sigma, S_tmp);
    }
    gsl_matrix_scale (Sigma, 1.0/(np - 1.0));

    // clean up
    gsl_vector_free (diff);
    gsl_matrix_free (S_tmp);
}


void
est_pf_resample (gsl_matrix *particles, gsl_vector *weights)
{
    gsl_matrix *new_particles = gsl_matrix_calloc (particles->size1, particles->size2);

    int np = particles->size2;

    // find the cdf of the weights
    gslu_vector_cumsum (weights);

    // initial offset
    gsl_rng *rng = gslu_rand_rng_alloc ();
    double offset = (1.0/(double) np) * gslu_rand_uniform_pos (rng) ;
    gsl_rng_free (rng);

    // resample loop
    int m = 0;
    for (int k=0 ; k<np ; k++)
    {
        while ( m < (np-1) && offset > gsl_vector_get(weights, m) )
            m++;

        gsl_vector_view to = gsl_matrix_column (new_particles, k);
        gsl_vector_view from = gsl_matrix_column (particles, m);
        gsl_vector_memcpy (&to.vector, &from.vector);

        offset = offset + (1.0/(double) np);
    }

    gsl_matrix_memcpy (particles, new_particles);

    gsl_matrix_free (new_particles);
}

void
est_pf_vector_normalize (gsl_vector *a)
{
    double sum = gslu_vector_sum (a);
    gsl_vector_scale (a, 1.0/sum);
}
