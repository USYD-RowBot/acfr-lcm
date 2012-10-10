#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "est_omf.h"

// -----------------------------------------------------------------------------
// Observation model function
// -----------------------------------------------------------------------------
void
est_omf_osi_motor_count_u (const est_estimator_t *state,  const gsl_vector *z,
                           const void *index_map, const gsl_matrix *R, 
                           gsl_vector *nu, gsl_matrix *H, void *user)
{    
    //user structure
    //est_omf_osi_motor_count_u_user_t *usr = user;
    
    //pull indicies  
    const perllcm_est_navigator_index_t *index = index_map;
     
    // direct measurement of state
    // set innovation 
    gsl_vector_set (nu, 0, gsl_vector_get (z,0) - gsl_vector_get (state->mu, index->u));
    // set jacobian
    gsl_matrix_set (H, 0, index->u, 1);

//gslu_matrix_printf(R, "R");
//gslu_matrix_printf(H, "H");
//gslu_vector_printf(nu, "nu");

}