#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"

#include "est_omf.h"

// -----------------------------------------------------------------------------
// Observation model function
// -----------------------------------------------------------------------------
void
est_omf_rdi_pd4_uvw (const est_estimator_t *state,  const gsl_vector *z,
                     const void *index_map, const gsl_matrix *R, 
                     gsl_vector *nu, gsl_matrix *H, void *user)
{    
    est_omf_rdi_pd4_uvw_user_t *usr = user;
    
    //pull indicies
    const perllcm_est_navigator_index_t *index = index_map;
    
    
    // sensor-vehicle orientation matrices
    //O_sv = rotxyz (x_vs(4:6));
    double O_sv[9];
    double rph_vs[3] = {usr->X_vs[3], usr->X_vs[4], usr->X_vs[5]};
    so3_rotxyz (O_sv, rph_vs);
    //t_vs = x_vs(1:3);
    double t_vs[3] = {usr->X_vs[0], usr->X_vs[1], usr->X_vs[2]};
    
    //predicted measurement
    //z_predict = O_sv * [uvw - skewsym(t_vs)*abc];
    //skewsym = [ 0       -t_vs(3)  t_vs(2); ...
    //            t_vs(3)  0       -t_vs(1); ...
    //           -t_vs(2)  t_vs(1)  0   ];
    double abc[3] = {gsl_vector_get (state->mu, index->a),
                     gsl_vector_get (state->mu, index->b),
                     gsl_vector_get (state->mu, index->c)};
    double uvw[3] = {gsl_vector_get (state->mu, index->u),
                     gsl_vector_get (state->mu, index->v),
                     gsl_vector_get (state->mu, index->w)};
    double skewsym[9] = { 0,       -t_vs[2],  t_vs[1],
                          t_vs[2],  0,       -t_vs[0],
                         -t_vs[1],  t_vs[0],  0        };
    gsl_vector *z_pred = gsl_vector_alloc (3);
    gsl_vector_view abc_v = gsl_vector_view_array (abc, 3);
    gsl_vector_view uvw_v = gsl_vector_view_array (uvw, 3);
    gsl_matrix_view O_sv_v = gsl_matrix_view_array (O_sv, 3, 3);
    gsl_matrix_view skewsym_v = gsl_matrix_view_array (skewsym, 3, 3);
    //skewsym(t_vs)*abc;
    gslu_blas_mv (z_pred, &skewsym_v.matrix, &abc_v.vector);
    //[uvw - skewsym(t_vs)*abc]
    gsl_vector_sub (&uvw_v.vector, z_pred);
    //z_predict = O_sv * [uvw - skewsym(t_vs)*abc];
    gslu_blas_mv (z_pred, &O_sv_v.matrix, &uvw_v.vector);
    
    // calculate innovation
    gsl_vector_set (nu, 0, gsl_vector_get(z,0) - gsl_vector_get(z_pred,0));
    gsl_vector_set (nu, 1, gsl_vector_get(z,1) - gsl_vector_get(z_pred,1));
    gsl_vector_set (nu, 2, gsl_vector_get(z,2) - gsl_vector_get(z_pred,2));
    
    //measurement model Jacobian
    //Hv = [d(u_s)/dXv
    //      d(v_s)/dXv
    //      d(w_s)/dXv]
    //Hv(1:3,uvw_i) =  O_sv;
    gsl_matrix_view H_uvw_v = gsl_matrix_submatrix (H, 0, index->u, 3, 3);
    gsl_matrix_memcpy (&H_uvw_v.matrix, &O_sv_v.matrix);
    //Hv(1:3,abc_i) = -O_sv * skewsym(t_vs);
    gsl_matrix_view H_abc_v = gsl_matrix_submatrix (H, 0, index->a, 3, 3);
    gsl_matrix *TMP = gsl_matrix_alloc (3, 3);
    gsl_matrix_scale (&O_sv_v.matrix, -1);
    gslu_blas_mm (TMP, &O_sv_v.matrix, &skewsym_v.matrix);
    gsl_matrix_memcpy (&H_abc_v.matrix, TMP);

//gslu_matrix_printf(O_sv, "O_sv");    
//gslu_vector_printf(z, "z");
//gslu_vector_printf(z_pred, "z_pred");
//gslu_vector_printf(nu, "nu");
//gslu_matrix_printf(H, "H");    
    
    gsl_matrix_free (TMP);
    gsl_vector_free (z_pred);
}
