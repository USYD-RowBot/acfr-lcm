#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"

#include "est_omf.h"

// -----------------------------------------------------------------------------
// Observation model function
// -----------------------------------------------------------------------------
void
est_omf_ms_gx1_abc (const est_estimator_t *state,  const gsl_vector *z,
                    const void *index_map, const gsl_matrix *R, 
                    gsl_vector *nu, gsl_matrix *H, void *user)
{
    est_omf_ms_gx1_abc_user_t *usr = user;

    //pull indicies
    const perllcm_est_navigator_index_t *index = index_map;

    //R_sv = rotxyz (x_vs(4:6));
    double R_sv[9];
    double rph_vs[3] = {usr->X_vs[3], usr->X_vs[4], usr->X_vs[5]};
    so3_rotxyz (R_sv, rph_vs);
    
    double abc[3] = {gsl_vector_get (state->mu, index->a),
                     gsl_vector_get (state->mu, index->b),
                     gsl_vector_get (state->mu, index->c)};
    GSLU_VECTOR_VIEW (z_pred, 3, {0});
    gsl_vector_view abc_v = gsl_vector_view_array (abc, 3);
    gsl_matrix_view R_vs_v = gsl_matrix_view_array (R_sv, 3, 3);
    gsl_matrix_transpose (&R_vs_v.matrix);
    gslu_blas_mv (&z_pred.vector, &R_vs_v.matrix, &abc_v.vector);
    
    // set innovation
    gsl_vector_set (nu, 0, gsl_vector_get (z, 0) - gsl_vector_get (&z_pred.vector, 0));
    gsl_vector_set (nu, 1, gsl_vector_get (z, 1) - gsl_vector_get (&z_pred.vector, 1));
    gsl_vector_set (nu, 2, gsl_vector_get (z, 2) - gsl_vector_get (&z_pred.vector, 2));
    
    // set jacobian
    // Hv(1:3,abc_i) =  R_sv;
    gsl_matrix_view H_abc_v = gsl_matrix_submatrix (H, 0, index->a, 3, 3);
    gsl_matrix_memcpy (&H_abc_v.matrix, &R_vs_v.matrix);
    
//gslu_vector_printf(nu, "nu");
//gslu_matrix_printf(H, "H");
}





// -----------------------------------------------------------------------------
// Observation model function
// -----------------------------------------------------------------------------
void
est_omf_ms_gx1_ab (const est_estimator_t *state,  const gsl_vector *z,
                   const void *index_map, const gsl_matrix *R, 
                   gsl_vector *nu, gsl_matrix *H, void *user)
{
    est_omf_ms_gx1_ab_user_t *usr = user;

    //pull indicies
    const perllcm_est_navigator_index_t *index = index_map;

    //R_sv = rotxyz (x_vs(4:6));
    double R_sv[9];
    double rph_vs[3] = {usr->X_vs[3], usr->X_vs[4], usr->X_vs[5]};
    so3_rotxyz (R_sv, rph_vs);
    
    double abc[3] = {gsl_vector_get (state->mu, index->a),
                     gsl_vector_get (state->mu, index->b),
                     (-1 == index->c) ? 0 : gsl_vector_get (state->mu, index->c)};
                     
    GSLU_VECTOR_VIEW (z_pred, 3, {0});
    gsl_vector_view abc_v = gsl_vector_view_array (abc, 3);
    gsl_matrix_view R_vs_v = gsl_matrix_view_array (R_sv, 3, 3);
    gsl_matrix_transpose (&R_vs_v.matrix);
    gslu_blas_mv (&z_pred.vector, &R_vs_v.matrix, &abc_v.vector);
    
    // set innovation
    gsl_vector_set (nu, 0, gsl_vector_get (z, 0) - gsl_vector_get (&z_pred.vector, 0));
    gsl_vector_set (nu, 1, gsl_vector_get (z, 1) - gsl_vector_get (&z_pred.vector, 1));
    
    // set jacobian
    // Hv(1:3,abc_i) =  R_sv;
    gsl_matrix_view H_abc_v = gsl_matrix_submatrix (H, 0, index->a, 2, 2);
    gsl_matrix_view R_vs_v_v = gsl_matrix_submatrix (&R_vs_v.matrix, 0, 0, 2, 2);
    gsl_matrix_memcpy (&H_abc_v.matrix, &R_vs_v_v.matrix);
    
//gslu_vector_printf(nu, "nu");
//gslu_matrix_printf(&R_vs_v.matrix, "R_vs");
//gslu_matrix_printf(H, "H");
}
