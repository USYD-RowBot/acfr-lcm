#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"

#include "est_pmf.h"

#define EULER_NUM_INT_DT 0.0001  // EULER NUMERICAL INTEGRATION DELTA TIME

#define I_U_WL 0  // control vector index left wheel angular velocity
#define I_U_WR 1  // control vector index right wheel angular velocity
#define I_U_DH 2  // control vector index delta heading

static const int state_len = 8;
static const int delayed_state_len = 8;

// -----------------------------------------------------------------------------
// process model function
// -----------------------------------------------------------------------------
void
est_pmf_differential_drive (const est_estimator_t *state, const gsl_vector *u_ctl, const double dt,
                            gsl_vector *x_bar, gsl_matrix *F, gsl_matrix *Q, void *user)
{
    est_pmf_differential_drive_user_t *usr = user;
    
    // read the index_t 
    perllcm_est_navigator_index_t index_t;
    est_pmf_differential_drive_get_index_t (&index_t);
    
    // get values from state vector
    // local-level translation
    double x = gsl_vector_get (state->mu, index_t.x); 
    double y = gsl_vector_get (state->mu, index_t.y);
    // local-level Euler angles (rph)
    double r = gsl_vector_get (state->mu, index_t.r); 
    double p = gsl_vector_get (state->mu, index_t.p);
    double h = gsl_vector_get (state->mu, index_t.h);
    // body frame angular rates
    double a = gsl_vector_get (state->mu, index_t.a); 
    double b = gsl_vector_get (state->mu, index_t.b);
    double c = gsl_vector_get (state->mu, index_t.c);
    // get values from control
    double wl = gsl_vector_get (u_ctl, I_U_WL); 
    double wr = gsl_vector_get (u_ctl, I_U_WR);
    double dh = gsl_vector_get (u_ctl, I_U_DH);
    
    //Euler angular rates in local-level frame
    GSLU_VECTOR_VIEW (rph, 3, {r, p, h});
    GSLU_VECTOR_VIEW (abc, 3, {a, b, c});
    GSLU_VECTOR_VIEW (rph_dot, 3, {0});
    GSLU_MATRIX_VIEW (Jabc_rph, 3, 6, {0});
    so3_body2euler_gsl (&abc.vector, &rph.vector, &rph_dot.vector, &Jabc_rph.matrix);
    
    // velocity of the center of the vehicle
    double wheel_size_fudge_factor = usr->wheel_size_fudge_factor;
    double vc = wheel_size_fudge_factor * (wl + wr) / 2.0;

    // set predicted state output
    // local-level translation
    gsl_vector_set (x_bar, index_t.x, x + vc*cos(h)*dt);
    gsl_vector_set (x_bar, index_t.y, y + vc*sin(h)*dt);
    // local-level Euler angles (rph)
    double r_dot = gsl_vector_get (&rph_dot.vector, 0);
    double p_dot = gsl_vector_get (&rph_dot.vector, 1);
    gsl_vector_set (x_bar, index_t.r, r + r_dot*dt);
    gsl_vector_set (x_bar, index_t.p, p + p_dot*dt);
    gsl_vector_set (x_bar, index_t.h, h + dh);
    // body frame angular rates (constant)
    gsl_vector_set (x_bar, index_t.a, a);
    gsl_vector_set (x_bar, index_t.b, b);
    gsl_vector_set (x_bar, index_t.c, c);
    
    // build process jacobian (F)
    gsl_matrix_set_identity (F);
    // x_bar wrt h
    gsl_matrix_set (F, index_t.x, index_t.h, -vc*sin(h)*dt);
    // y_bar wrt h
    gsl_matrix_set (F, index_t.y, index_t.h,  vc*cos(h)*dt);
    
    // pre scale the body2euler jacobian by dt
    gsl_matrix_scale (&Jabc_rph.matrix, dt);
    // rp_bar wrt rph
    gsl_matrix_view F_rp_rph =  gsl_matrix_submatrix (F, index_t.r, index_t.r, 2, 3);
    gsl_matrix_view J_rph = gsl_matrix_submatrix (&Jabc_rph.matrix, 0, 3, 2, 3);
    // important added NOT overwritten need identity 2x3 
    gsl_matrix_add (&F_rp_rph.matrix, &J_rph.matrix); 
    // rp_bar wrt abc
    gsl_matrix_view J_abc = gsl_matrix_submatrix (&Jabc_rph.matrix, 0, 0, 2, 3);
    gslu_matrix_set_submatrix (F, index_t.r, index_t.a, &J_abc.matrix);
    
    // build additive noise (Q)
    GSLU_MATRIX_VIEW (Qu_full, 8, 8, {0});
    GSLU_MATRIX_VIEW (Ju, 8, 3, {0});
    // set non-zero elements of jacobian
    double x_wrt_wv = wheel_size_fudge_factor * cos(h) * dt / 2.0;
    gsl_matrix_set (&Ju.matrix, index_t.x, I_U_WL, x_wrt_wv);
    gsl_matrix_set (&Ju.matrix, index_t.x, I_U_WR, x_wrt_wv);
    double y_wrt_wv = wheel_size_fudge_factor * sin(h) * dt / 2.0;
    gsl_matrix_set (&Ju.matrix, index_t.y, I_U_WL, y_wrt_wv);
    gsl_matrix_set (&Ju.matrix, index_t.y, I_U_WR, y_wrt_wv);
    gsl_matrix_set (&Ju.matrix, index_t.h, I_U_DH, 1);

    gslu_blas_mmmT (&Qu_full.matrix, &Ju.matrix, usr->Qu, &Ju.matrix, NULL);
        
    gsl_matrix_add (Q, usr->Qadd);
    gsl_matrix_add (Q, &Qu_full.matrix);
    

//gslu_matrix_printf (&Ju.matrix, "Qu");
//gslu_matrix_printf (&Ju.matrix, "Ju");
//gslu_matrix_printf (usr->Qadd, "usr->Qadd");
//gslu_matrix_printf (&Qu_full.matrix, "Qu_full");
//gslu_matrix_printf (Q, "Q");

// hack, always propagate forward same state
// gsl_vector_memcpy (x_bar, state->mu);
// gsl_matrix_set_identity (F);    
    
    
//gslu_vector_printf (u_ctl, "u_ctl");
//gslu_vector_printf (x_bar, "x_bar");
//gslu_vector_printf (state->mu, "state->mu");
// once there are attitude observation models check and make sure F is as expected
//gslu_matrix_printf (&Jabc_rph.matrix, "Jabc_rph");    
//gslu_matrix_printf (F, "F");
    
}

// -----------------------------------------------------------------------------
// process index structure function
// -----------------------------------------------------------------------------
void
est_pmf_differential_drive_get_index_t (perllcm_est_navigator_index_t *index)
{    
    //init all unused
    init_index_t (index);
    
    index->proc_state_len = state_len; //length of state process model acts on
    
    //length of control vector
    index->u_len = 3; //no control for this process model          
    
    // specify indicies for the used variables
    index->x = 0;     // Translation (x y)
    index->y = 1;
    index->r = 2;     // Euler angle rotation
    index->p = 3;
    index->h = 4;
    index->a = 5;     // body frame angular velocities
    index->b = 6;
    index->c = 7;
}


// -----------------------------------------------------------------------------
// process model user_t functions
// -----------------------------------------------------------------------------
// allocate user_t
est_pmf_differential_drive_user_t *
est_pmf_differential_drive_alloc_user_t (void)
{   
   est_pmf_differential_drive_user_t *user = calloc (1, sizeof (*user));
   
   user->Qu = gsl_matrix_calloc (3, 3);
   user->Qadd = gsl_matrix_calloc (state_len, state_len);

   return user; 
}

// free user_t
void
est_pmf_differential_drive_free_user_t (void *user)
{
    est_pmf_differential_drive_user_t *usr = user;
    
    gslu_matrix_free (usr->Qu);
    gslu_matrix_free (usr->Qadd);
    free (usr); 
}

