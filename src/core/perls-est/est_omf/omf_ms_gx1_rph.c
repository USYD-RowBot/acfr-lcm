#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/gsl_util.h"
#include "perls-math/ssc.h"

#include "est_omf.h"

// -----------------------------------------------------------------------------
// Observation model function
// -----------------------------------------------------------------------------
void
est_omf_ms_gx1_rph (const est_estimator_t *state,  const gsl_vector *z,
                    const void *index_map, const gsl_matrix *R, 
                    gsl_vector *nu, gsl_matrix *H, void *user)
{    
    est_omf_ms_gx1_rph_user_t *usr = user;
    
    //pull indicies
    const perllcm_est_navigator_index_t *index = index_map;
 
    double r_z = gsl_vector_get (z, 0);
    double p_z = gsl_vector_get (z, 1);
    double h_z = gsl_vector_get (z, 2); //heading
      
    // vehicle pose in local-level frame from current state est
    double x_lv[6] = {  (-1 == index->x) ? 0 : gsl_vector_get (state->mu, index->x),
                        (-1 == index->y) ? 0 : gsl_vector_get (state->mu, index->y),
                        (-1 == index->z) ? 0 : gsl_vector_get (state->mu, index->z),
                        gsl_vector_get (state->mu, index->r),
                        gsl_vector_get (state->mu, index->p),
                        gsl_vector_get (state->mu, index->h)};
    
    // predicted microstrain attitude meas wrt sensor frame
    //[x_ls, Jls_plus] = head2tail (x_lv, x_vs);
    double Jls_plus[6*12];
    double x_ls[6];
    ssc_head2tail (x_ls, Jls_plus, x_lv, usr->X_vs);
    
    //[x_rs, Jrs_plus] = head2tail (x_rl, x_ls);
    double Jrs_plus[6*12];
    double x_rs[6];
    double x_rl[6] = {0};
    ssc_inverse (x_rl, NULL, usr->X_lr);
    ssc_head2tail (x_rs, Jrs_plus, x_rl, x_ls);        
    // complete Jacobian d(x_rs)/d(x_lv) = d(x_rs)/d(x_ls) * d(x_ls)/d(x_lv)
    //J_plus = Jrs_plus(:,7:12) * Jls_plus(:,1:6);
    gsl_matrix *J_plus = gsl_matrix_calloc (6, 6);
    gsl_matrix_view Jls_v = gsl_matrix_view_array (Jls_plus, 6, 12);
    gsl_matrix_view Jrs_v = gsl_matrix_view_array (Jrs_plus, 6, 12);
    // pull out views of left block and right block
    gsl_matrix_view Jls_v_lb = gsl_matrix_submatrix (&Jls_v.matrix, 0, 0, 6, 6);
    gsl_matrix_view Jrs_v_rb = gsl_matrix_submatrix (&Jrs_v.matrix, 0, 6, 6, 6);
    gslu_blas_mm (J_plus, &Jrs_v_rb.matrix, &Jls_v_lb.matrix);
    
    // predicted measurement = z_predict = x_rs(4:6);
    // calculate innovation
    //gsl_vector_set(nu, 0, bot_mod2pi(r_z - x_rs[3]));
    //gsl_vector_set(nu, 1, bot_mod2pi(p_z - x_rs[4]));
    //gsl_vector_set(nu, 2, bot_mod2pi(h_z - x_rs[5]));
    gsl_vector_set (nu, 0, gslu_math_minimized_angle (r_z - x_rs[3]));
    gsl_vector_set (nu, 1, gslu_math_minimized_angle (p_z - x_rs[4]));
    gsl_vector_set (nu, 2, gslu_math_minimized_angle (h_z - x_rs[5]));
    
    // measurement model Jacobian
    // H = [d(r_s)/dXv
    //      d(p_s)/dXv
    //      d(h_s)/dXv]
    // H(:,xp_i) = J_plus(4:6,:);
    // wrt xyz is zero, only fillin wrt rph (might not be true if x)
    gsl_matrix_view H_p = gsl_matrix_submatrix (H, 0, index->r, 3, 3);
    gsl_matrix_view J_plus_l = gsl_matrix_submatrix (J_plus, 3, 3, 3, 3);
    gsl_matrix_memcpy (&H_p.matrix, &J_plus_l.matrix);

//gslu_vector_printf(z, "z");
//gslu_array_printf (&x_rs[3], 3, 1, "z_pred");
//gslu_vector_printf(nu, "nu");
//gslu_vector_printf(state->mu, "state->mu");
//gslu_matrix_printf(state->Sigma, "state->Sigma");
//gslu_matrix_printf(H, "H");
//gslu_matrix_printf (J_plus, "J_plus");
//gslu_matrix_printf(R, "R");
    
    gsl_matrix_free (J_plus);
}



// -----------------------------------------------------------------------------
// Observation model function
// -----------------------------------------------------------------------------
void
est_omf_ms_gx1_rp (const est_estimator_t *state,  const gsl_vector *z,
                   const void *index_map, const gsl_matrix *R, 
                   gsl_vector *nu, gsl_matrix *H, void *user)
{    
    est_omf_ms_gx1_rp_user_t *usr = user;
    
    //pull indicies
    const perllcm_est_navigator_index_t *index = index_map;
 
    double r_z = gsl_vector_get (z, 0);
    double p_z = gsl_vector_get (z, 1);
      
    // vehicle pose in local-level frame from current state est
    double x_lv[6] = {  (-1 == index->x) ? 0 : gsl_vector_get (state->mu, index->x),
                        (-1 == index->y) ? 0 : gsl_vector_get (state->mu, index->y),
                        (-1 == index->z) ? 0 : gsl_vector_get (state->mu, index->z),
                        gsl_vector_get (state->mu, index->r),
                        gsl_vector_get (state->mu, index->p),
                        (-1 == index->h) ? 0 : gsl_vector_get (state->mu, index->h)};
    
    // predicted microstrain attitude meas wrt sensor frame
    //[x_ls, Jls_plus] = head2tail (x_lv, x_vs);
    double Jls_plus[6*12];
    double x_ls[6];
    ssc_head2tail (x_ls, Jls_plus, x_lv, usr->X_vs);
    
    //[x_rs, Jrs_plus] = head2tail (x_rl, x_ls);
    double Jrs_plus[6*12];
    double x_rs[6];
    double x_rl[6] = {0};
    ssc_inverse (x_rl, NULL, usr->X_lr);
    ssc_head2tail (x_rs, Jrs_plus, x_rl, x_ls);    
    // complete Jacobian d(x_rs)/d(x_lv) = d(x_rs)/d(x_ls) * d(x_ls)/d(x_lv)
    //J_plus = Jrs_plus(:,7:12) * Jls_plus(:,1:6);
    gsl_matrix *J_plus = gsl_matrix_calloc (6, 6);
    gsl_matrix_view Jls_v = gsl_matrix_view_array (Jls_plus, 6, 12);
    gsl_matrix_view Jrs_v = gsl_matrix_view_array (Jrs_plus, 6, 12);
    // pull out views of left block and right block
    gsl_matrix_view Jls_v_lb = gsl_matrix_submatrix (&Jls_v.matrix, 0, 0, 6, 6);
    gsl_matrix_view Jrs_v_rb = gsl_matrix_submatrix (&Jrs_v.matrix, 0, 6, 6, 6);
    gslu_blas_mm (J_plus, &Jrs_v_rb.matrix, &Jls_v_lb.matrix);
    
    // predicted measurement = z_predict = x_rs(4:6);
    // calculate innovation
    gsl_vector_set (nu, 0, gslu_math_minimized_angle (r_z - x_rs[3]));
    gsl_vector_set (nu, 1, gslu_math_minimized_angle (p_z - x_rs[4]));
    
    // measurement model Jacobian
    // H = [d(r_s)/dXv
    //      d(p_s)/dXv
    //      d(h_s)/dXv]
    // H(:,xp_i) = J_plus(4:6,:);
    // wrt xyz is zero, only fillin wrt rp
    gsl_matrix_view H_p = gsl_matrix_submatrix (H, 0, index->r, 2, 2);
    gsl_matrix_view J_plus_l = gsl_matrix_submatrix (J_plus, 3, 3, 2, 2);
    gsl_matrix_memcpy (&H_p.matrix, &J_plus_l.matrix);

//gslu_vector_printf(z, "z");
//gslu_array_printf (&x_rs[3], 3, 1, "z_pred");
//gslu_vector_printf(nu, "nu");
//gslu_vector_printf(state->mu, "state->mu");
//gslu_matrix_printf(state->Sigma, "state->Sigma");
//gslu_matrix_printf(H, "H");
//gslu_matrix_printf (J_plus, "J_plus");
//gslu_matrix_printf(R, "R");
    
    gsl_matrix_free (J_plus);
}
