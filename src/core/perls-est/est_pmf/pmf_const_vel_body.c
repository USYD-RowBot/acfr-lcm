#include <gsl/gsl_linalg.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"

#include "est_pmf.h"

#define EULER_NUM_INT_DT 0.0001  // EULER NUMERICAL INTEGRATION DELTA TIME

static const int state_len = 12;
static const int delayed_state_len = 12;

// -----------------------------------------------------------------------------
// process model function
// -----------------------------------------------------------------------------
void
est_pmf_const_vel_body (const est_estimator_t *state, const gsl_vector *u_ctl, const double dt,
                        gsl_vector *x_bar, gsl_matrix *F, gsl_matrix *Q, void *user)
{
    est_pmf_const_vel_body_user_t *usr = user;
    
    // read the index_t 
    perllcm_est_navigator_index_t index_t;
    est_pmf_const_vel_body_get_index_t (&index_t);

    // get values from state vector
    //local-level Euler angles (rph)
    double r = gsl_vector_get (state->mu, index_t.r); 
    double p = gsl_vector_get (state->mu, index_t.p);
    double h = gsl_vector_get (state->mu, index_t.h);
    //body frame velocities
    double u = gsl_vector_get (state->mu, index_t.u); 
    double v = gsl_vector_get (state->mu, index_t.v);
    double w = gsl_vector_get (state->mu, index_t.w);
    //body frame angular rates
    double a = gsl_vector_get (state->mu, index_t.a); 
    double b = gsl_vector_get (state->mu, index_t.b);
    double c = gsl_vector_get (state->mu, index_t.c);

    //rotation matrix vehicle to local-level (Rlv = rotxyz(rph);)
    gsl_matrix *Rlv = gsl_matrix_calloc (3, 3);
    gsl_vector *rph = gsl_vector_calloc (3);
    gsl_vector_set (rph, 0, r);
    gsl_vector_set (rph, 1, p);
    gsl_vector_set (rph, 2, h);
    so3_rotxyz_gsl (Rlv, rph);

    //vehicle velocity in local-level frame (xyz_dot = Rlv*uvw)
    gsl_vector *xyz_dot = gsl_vector_calloc (3);
    gsl_vector *uvw = gsl_vector_calloc (3);
    gsl_vector_set (uvw, 0, u);
    gsl_vector_set (uvw, 1, v);
    gsl_vector_set (uvw, 2, w);
    gslu_blas_mv (xyz_dot, Rlv, uvw ); // xyz_dot = Rlv*uvw

    //Euler angular rates in local-level frame
    gsl_vector *rph_dot = gsl_vector_calloc (3);
    gsl_vector *abc = gsl_vector_calloc (3);
    gsl_vector_set (abc, 0, a);
    gsl_vector_set (abc, 1, b);
    gsl_vector_set (abc, 2, c);
    gsl_matrix *Jabc_rph = gsl_matrix_calloc (3, 6);
    //[rph_dot,Jabc_rph] = body2euler(abc,rph);
    so3_body2euler_gsl (abc, rph, rph_dot, Jabc_rph);

    // constant velocity process model
    gsl_vector *x_dot = gsl_vector_calloc (state_len);
    //Xdot(xyz_i) = xyz_dot;
    gsl_vector_set (x_dot, index_t.x, gsl_vector_get (xyz_dot, 0)); 
    gsl_vector_set (x_dot, index_t.y, gsl_vector_get (xyz_dot, 1));
    gsl_vector_set (x_dot, index_t.z, gsl_vector_get (xyz_dot, 2));
    //Xdot(rph_i) = rph_dot;
    gsl_vector_set (x_dot, index_t.r, gsl_vector_get (rph_dot, 0)); 
    gsl_vector_set (x_dot, index_t.p, gsl_vector_get (rph_dot, 1));
    gsl_vector_set (x_dot, index_t.h, gsl_vector_get (rph_dot, 2));
    //Xdot(uvw_i) = [0,0,0]';
    gsl_vector_set (x_dot, index_t.u, 0);
    gsl_vector_set (x_dot, index_t.v, 0);
    gsl_vector_set (x_dot, index_t.w, 0);
    //Xdot(abc_i) = [0,0,0]';
    gsl_vector_set (x_dot, index_t.a, 0);
    gsl_vector_set (x_dot, index_t.b, 0);
    gsl_vector_set (x_dot, index_t.c, 0);

    // calculate the Jacobian
    gsl_matrix *ROTX = gsl_matrix_calloc (3, 3);
    gsl_matrix *ROTY = gsl_matrix_calloc (3, 3);
    gsl_matrix *ROTZ = gsl_matrix_calloc (3, 3);
    gsl_matrix *DROTX = gsl_matrix_calloc (3, 3);
    gsl_matrix *DROTY = gsl_matrix_calloc (3, 3);
    gsl_matrix *DROTZ = gsl_matrix_calloc (3, 3);
    so3_rotx_gsl (ROTX, r);
    so3_roty_gsl (ROTY, p);
    so3_rotz_gsl (ROTZ, h);
    so3_drotx_gsl (DROTX, r);
    so3_droty_gsl (DROTY, p);
    so3_drotz_gsl (DROTZ, h);
    
    gsl_matrix_set_zero (F); //F is 12x12
    gsl_matrix *Fv = gsl_matrix_calloc (state_len, state_len);

    // fill in non-zero entries of Fv for xyz_dot
    // Fv(xyz_i,uvw_i)    = Rlv;  %deriv xyz_dot w.c.t. uvw
    gsl_matrix_view  F_Rlv = gsl_matrix_submatrix (Fv, index_t.x, index_t.u, 3, 3);
    gsl_matrix_memcpy (&F_Rlv.matrix, Rlv);

    gsl_matrix *tmp_rot = gsl_matrix_calloc (3, 3);
    gsl_vector *tmp_vec = gsl_vector_calloc (3);
    
    // Fv(xyz_i,rph_i(1)) = ROTZ'*ROTY'*DROTX'*uvw;  % deriv xyz_dot w.c.t. c
    gslu_blas_mTmTmT (tmp_rot, ROTZ, ROTY, DROTX, NULL);
    gslu_blas_mv (tmp_vec, tmp_rot, uvw);
    gsl_vector_view  F_wrt_r = gsl_matrix_subcolumn (Fv, index_t.r, index_t.x, 3);
    gsl_vector_memcpy (&F_wrt_r.vector, tmp_vec);
    
    // Fv(xyz_i,rph_i(2)) = ROTZ'*DROTY'*ROTX'*uvw;  % deriv xyz_dot w.c.t. a
    gslu_blas_mTmTmT (tmp_rot, ROTZ, DROTY, ROTX, NULL);
    gslu_blas_mv (tmp_vec, tmp_rot, uvw);
    gsl_vector_view  F_wrt_p = gsl_matrix_subcolumn (Fv, index_t.p, index_t.x, 3);
    gsl_vector_memcpy (&F_wrt_p.vector, tmp_vec);
    
    //  Fv(xyz_i,rph_i(3)) = DROTZ'*ROTY'*ROTX'*uvw;  % deriv xyz_dot w.c.t. h
    gslu_blas_mTmTmT (tmp_rot, DROTZ, ROTY, ROTX, NULL);
    gslu_blas_mv (tmp_vec, tmp_rot, uvw);
    gsl_vector_view  F_wrt_h = gsl_matrix_subcolumn (Fv, index_t.h, index_t.x, 3);
    gsl_vector_memcpy (&F_wrt_h.vector, tmp_vec);

    //  fill in non-zero entries for rph_dot
    //  Fv(rph_i,rph_i) = Jabc_rph(:,4:6);            % deriv rph_dot w.c.t. rph
    gsl_matrix_view  F_wrt_rph = gsl_matrix_submatrix (Fv, index_t.r, index_t.r, 3, 3);
    gsl_matrix_view  Jrph = gsl_matrix_submatrix (Jabc_rph, 0, 3, 3, 3);
    gsl_matrix_memcpy (&F_wrt_rph.matrix, &Jrph.matrix);
    //  Fv(rph_i,abc_i) = Jabc_rph(:,1:3);            % deriv rph_dot w.c.t. abc
    gsl_matrix_view  F_wrt_abc = gsl_matrix_submatrix (Fv, index_t.r, index_t.a, 3, 3);
    gsl_matrix_view  Jabc = gsl_matrix_submatrix (Jabc_rph, 0, 0, 3, 3);
    gsl_matrix_memcpy (&F_wrt_abc.matrix, &Jabc.matrix);

    // converter pwcontinous to discrete
    // have x_dot and Fv (pwcontinous)
    // need x_bar and F (discrete)
    // use van Loan method to compute state transition matrix and associated
    // process noise covariance, see Brown & Hwang reference
    gsl_matrix *A = gsl_matrix_calloc (2*state_len, 2*state_len);
    // build A = [-Fv,  Qv; ...
    //             0,   Fv'];
    // upper left block -Fv
    gsl_matrix *tmp_1212 = gsl_matrix_calloc (state_len, state_len);
    gsl_matrix_memcpy (tmp_1212, Fv);
    gsl_matrix_scale (tmp_1212, -1);
    gsl_matrix_view  A_up_l = gsl_matrix_submatrix (A, 0, 0, state_len, state_len);
    gsl_matrix_memcpy (&A_up_l.matrix, tmp_1212);
    // lower right block Fv'
    gsl_matrix_memcpy (tmp_1212, Fv);
    gsl_matrix_transpose(tmp_1212);
    gsl_matrix_view  A_lo_r = gsl_matrix_submatrix (A, state_len, state_len, state_len, state_len);
    gsl_matrix_memcpy (&A_lo_r.matrix, tmp_1212);
    // upper right block Qv
    gsl_matrix_view  A_up_r = gsl_matrix_submatrix (A, 0, state_len, state_len, state_len);
    gsl_matrix_memcpy (&A_up_r.matrix, usr->Qv);

    // calculate the matrix exponential
    //B = expm(A*dt);
    gsl_matrix *B = gsl_matrix_calloc (2*state_len, 2*state_len);
    gsl_matrix_scale (A, dt);
    gsl_linalg_exponential_ss (A, B, .01);

    //F = B(state_len+1:end,state_len+1:end)'; % discrete state transition matrix
    gsl_matrix_view B_lr_v = gsl_matrix_submatrix (B, state_len, state_len, state_len, state_len);
    gsl_matrix_memcpy (F, &B_lr_v.matrix);
    gsl_matrix_transpose (F);
    
    //Q = F * B(1:state_len,state_len+1:end); % discrete process covariance
    gsl_matrix_view B_ur_v = gsl_matrix_submatrix (B, 0, state_len, state_len, state_len);
    gslu_blas_mm (Q, F, &B_ur_v.matrix);

    //% compute DT input
    //uk = x_dot - Fv*mu_v;
    gsl_vector *uk = gsl_vector_calloc (state_len);
    gslu_blas_mv (uk, Fv, state->mu);
    gsl_vector_scale (uk, -1);
    gsl_vector_add (uk, x_dot);


    // find bk with numerical integration (euler)
    gsl_matrix *Bk = gsl_matrix_calloc (state_len, state_len);
    gsl_matrix *NUMINT = gsl_matrix_calloc (state_len, state_len);
    gsl_matrix *nFv = gsl_matrix_calloc (state_len, state_len);
    gsl_matrix *exp_nFv = gsl_matrix_calloc (state_len, state_len);


    // simpson rule numerical integration to find Bk
    // number of pts for simpson rule numerical int, must be even
    #define SIMPSON_NUM_PTS 6
    double t = 0; //t = 0 ... dt
    for (int i = 0; i <= SIMPSON_NUM_PTS ; i++) {
        // tmp = tmp + expm(-Fv*t);
        gsl_matrix_memcpy (nFv, Fv);
        gsl_matrix_scale (nFv, -1);
        gsl_matrix_scale (nFv, t);
        gsl_linalg_exponential_ss (nFv, exp_nFv, .01);
        
        // scale depending on where we are in the sequence
        // no scaling on first value
        if (i > 0 && i < SIMPSON_NUM_PTS) {
            if (0 == (i % 2)) //even scale by 2
                gsl_matrix_scale (exp_nFv, 2);    
            else // if odd scale by 4
                gsl_matrix_scale (exp_nFv, 4);
        }
        gsl_matrix_add (NUMINT, exp_nFv);
      
        // t = t + (xn - x0)/n;  
        t = t + dt/SIMPSON_NUM_PTS;
    }
    // scale by (xn-x0)/(3*n)
    gsl_matrix_scale (NUMINT, dt/(3*SIMPSON_NUM_PTS));


//////// WORKS BUT IN ORDER TO BE ACCURATE ENOUGH (SMALL DT) VERY SLOW
////////  // actually might just have a bug
////////  
////////    int num_steps = floor(dt/EULER_NUM_INT_DT);
//////////printf("NUMSTEPS = %d \n", num_steps);
////////    double t = 0;
////////    for (int i = 0; i <= num_steps ; i++) {
////////        // tmp = tmp + expm(-Fv*n)*dt_dt;
////////        gsl_matrix_memcpy (nFv, Fv);
////////        gsl_matrix_scale (nFv, -1);
////////        gsl_matrix_scale (nFv, t);
////////        gsl_linalg_exponential_ss(nFv, exp_nFv, .01);
////////        gsl_matrix_scale (exp_nFv, EULER_NUM_INT_DT);
////////        gsl_matrix_add (NUMINT, exp_nFv);
////////        t = t + EULER_NUM_INT_DT;
////////    }
////////    // integrate the last little bit if it is more than 1/4 a EULER_NUM_INT_DT
////////    if (t < dt - EULER_NUM_INT_DT/4) {
////////        gsl_matrix_memcpy (nFv, Fv);
////////        gsl_matrix_scale (nFv, -1);
////////        gsl_matrix_scale (nFv, dt);
////////        gsl_linalg_exponential_ss(nFv, exp_nFv, .01);
////////        gsl_matrix_scale (exp_nFv, dt-t);
////////        gsl_matrix_add (NUMINT, exp_nFv);
////////    }


    //Bk = expm(Fv*dt) * tmp;
    gsl_matrix_memcpy (nFv, Fv);
    gsl_matrix_scale (nFv, dt);
    gsl_linalg_exponential_ss (nFv, exp_nFv, .01);
    gslu_blas_mm (Bk, exp_nFv, NUMINT);
    
    //x_bar = F*mu + Bk*uk;
    //x_bar = F*mu;
    gslu_blas_mv (x_bar, F, state->mu);
    gsl_vector *Bk_uk = gslu_blas_mv_alloc (Bk, uk);
    gsl_vector_add (x_bar, Bk_uk);


//printf("DT = %l\n", dt);
//gslu_matrix_printf(Fv, "Fv");
//gslu_matrix_printf(F, "F");
//gslu_matrix_printf(Q, "Q");
//gslu_matrix_printf(usr->Qv, "Q");
//gslu_matrix_printf(Bk, "Bk");
//gslu_matrix_printf(exp_nFv, "exp_nFv");
//gslu_matrix_printf(NUMINT, "NUMINT");
//gslu_vector_printf(x_bar, "x_bar");


    gsl_matrix_free (Rlv);
    gsl_vector_free (rph);
    gsl_vector_free (xyz_dot);
    gsl_vector_free (uvw);
    gsl_vector_free (rph_dot);
    gsl_vector_free (abc);
    gsl_matrix_free (Jabc_rph);
    gsl_vector_free (x_dot);
    gsl_matrix_free (ROTX);
    gsl_matrix_free (ROTY);
    gsl_matrix_free (ROTZ);
    gsl_matrix_free (DROTX);
    gsl_matrix_free (DROTY);
    gsl_matrix_free (DROTZ);
    gsl_matrix_free (Fv);
    gsl_matrix_free (tmp_rot);
    gsl_vector_free (tmp_vec);
    gsl_matrix_free (A);
    gsl_matrix_free (tmp_1212);
    gsl_matrix_free (B);
    gsl_vector_free (uk);
    gsl_matrix_free (Bk);
    gsl_matrix_free (nFv);
    gsl_matrix_free (exp_nFv);
    gsl_matrix_free (NUMINT);
    gsl_vector_free (Bk_uk);
}

// -----------------------------------------------------------------------------
// process index structure function
// -----------------------------------------------------------------------------
void
est_pmf_const_vel_body_get_index_t (perllcm_est_navigator_index_t *index)
{    
    //init all unused
    init_index_t (index);
    
    index->proc_state_len = state_len; //length of state process model acts on

    //length of control vector
    index->u_len = 0; //no control for this process model          
    
    // specify indicies for the used variables
    index->x = 0;  // Translation
    index->y = 1;
    index->z = 2;
    index->r = 3;  // Euler angle rotation
    index->p = 4;
    index->h = 5;
    index->u = 6;  // body frame velocities
    index->v = 7;
    index->w = 8;
    index->a = 9;  // body frame angular velocities
    index->b = 10;
    index->c = 11;
}


// -----------------------------------------------------------------------------
// process model user_t functions
// -----------------------------------------------------------------------------
// allocate user_t
est_pmf_const_vel_body_user_t *
est_pmf_const_vel_body_alloc_user_t (void)
{   
    est_pmf_const_vel_body_user_t *user = calloc (1, sizeof (*user));
   
   user->Qv = gsl_matrix_calloc (state_len, state_len);  
   return user; 
}

// free user_t
void
est_pmf_const_vel_body_free_user_t (void *user)
{
    est_pmf_const_vel_body_user_t *usr = user;
    
    gsl_matrix_free (usr->Qv);
    free (usr);
}

