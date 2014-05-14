#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/ssc.h"

#include "est_omf.h"

// -----------------------------------------------------------------------------
// Observation model function
// -----------------------------------------------------------------------------
void
est_omf_gpsd_xy (const est_estimator_t *state,  const gsl_vector *z,
                 const void *index_map, const gsl_matrix *R, 
                 gsl_vector *nu, gsl_matrix *H, void *user)
{    
    est_omf_gpsd_xy_user_t *usr = user;
    
    //pull indicies
    const perllcm_est_navigator_index_t *index = index_map;
    
    // vehicle pose in local-level frame from current state est
    double x_lv[6] = {  gsl_vector_get (state->mu, index->x),
                        gsl_vector_get (state->mu, index->y),
                        gsl_vector_get (state->mu, index->z),
                        gsl_vector_get (state->mu, index->r),
                        gsl_vector_get (state->mu, index->p),
                        gsl_vector_get (state->mu, index->h)};

    // xform from local-level to gps reference frame (some translation)

    // predicted microstrain attitude meas wrt sensor frame
    //[x_ls, Jls_plus] = head2tail (x_lv, x_vs);
    double J_plus[6*12];
    double x_ls[6];
    ssc_head2tail (x_ls, J_plus, x_lv, usr->X_vs);
    
    //set measurement jacobian
    // H(1:2 , 1:6) = J_plus(1:2 , 1:6)
    gsl_matrix_view J_plus_v = gsl_matrix_view_array (J_plus, 6, 12);
    gsl_matrix_view H_p = gsl_matrix_submatrix (H, 0, index->x, 2, 6);
    gsl_matrix_view J_plus_tl = gsl_matrix_submatrix (&J_plus_v.matrix, 0, 0, 2, 6);
    gsl_matrix_memcpy (&H_p.matrix, &J_plus_tl.matrix);

    //set innovation
    gsl_vector_set (nu, 0, (gsl_vector_get (z, 0) - x_ls[0]));
    gsl_vector_set (nu, 1, (gsl_vector_get (z, 1) - x_ls[1]));
    
//printf("x_mu = %f, x_ls[0] = %f, x_z = %f\n", x_lv[0], x_ls[0], gsl_vector_get (z, 0));    
//gslu_vector_printf(nu, "nu");
//gslu_matrix_printf(H, "H");
//gslu_matrix_printf(R, "R");

}
