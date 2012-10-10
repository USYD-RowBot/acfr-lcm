#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"

#include "est_omf.h"

// -----------------------------------------------------------------------------
// Observation model function
// -----------------------------------------------------------------------------
void
est_omf_dstar_ssp1_z (const est_estimator_t *state,  const gsl_vector *z,
                      const void *index_map, const gsl_matrix *R, 
                      gsl_vector *nu, gsl_matrix *H, void *user)
{
    est_omf_dstar_ssp1_z_user_t *usr = user;
    
    //pull indicies
    const perllcm_est_navigator_index_t *index = index_map;

    // sensor-vehicle orientation matrices
    //O_sv = rotxyz (x_vs(4:6));
    double O_sv[9];
    double rph_vs[3] = {usr->X_vs[3], usr->X_vs[4], usr->X_vs[5]};
    so3_rotxyz (O_sv, rph_vs);
    
    //%predicted measurement
    //z_predict = O_sv(3,:) * xyz;
    gsl_vector_view O_sv_br = gsl_vector_view_array (&O_sv[6], 3);
    double xyz[3] = {gsl_vector_get (state->mu, index->x),
                     gsl_vector_get (state->mu, index->y),
                     gsl_vector_get (state->mu, index->z)};
    gsl_vector_view xyz_v = gsl_vector_view_array (xyz, 3);
    double z_predict = gslu_vector_dot (&O_sv_br.vector, &xyz_v.vector);
    
    // calculate innovation
    gsl_vector_set (nu, 0, gsl_vector_get (z, 0) - z_predict);
  
    // measurement model Jacobian
    // Hv = [d(z_s)/dXv]
    // Hv(1,xyz_i) =  O_sv(3,:);
    gsl_matrix_view H_p = gsl_matrix_submatrix (H, 0, index->x, 1, 3);
    gsl_matrix_view O_sv_3 = gsl_matrix_view_vector (&O_sv_br.vector, 1, 3);
    gsl_matrix_memcpy (&H_p.matrix, &O_sv_3.matrix);

//printf ("z_predict = %f \n", z_predict);    
//gslu_vector_printf(z, "z");
//gslu_vector_printf(nu, "nu");
//gslu_matrix_printf(H, "H");
//gslu_matrix_printf(R, "R");
}

