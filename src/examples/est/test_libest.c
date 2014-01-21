#include <stdio.h>
#include <gsl/gsl_blas.h>
#include <bot_core/bot_core.h>

#include "perls-est/est.h"
#include "perls-math/math.h"

typedef struct _pred_u_data_t pred_u_data_t;
struct _pred_u_data_t
{
    double alphas[4];
};

typedef struct _meas_u_data_t meas_u_data_t;
struct _meas_u_data_t
{
    int marker_id;
};

// ============================================================================
// OMFs
// ============================================================================

void
om_ekf_bearing (const est_estimator_t * state,  const gsl_vector * z,
                const void *index_map, const gsl_matrix * R,
                gsl_vector * nu, gsl_matrix * H, void *user){
    
    meas_u_data_t * meas_u_data = user;
  
    //known marker position
    int marker_x[6] = {21, 242, 463, 463, 242, 21};
    int marker_y[6] = {0, 0, 0, 292, 292, 292};

    // given known marker id calcuated predicted observation
    double dx = marker_x[meas_u_data->marker_id-1] - gsl_vector_get(state->mu, 0);
    double dy = marker_y[meas_u_data->marker_id-1] - gsl_vector_get(state->mu, 1);
    double zhat = gslu_math_minimized_angle( atan2(dy, dx) - gsl_vector_get(state->mu, 2));

    //set the innovation
    double ztmp = gsl_vector_get(z,0);
    gsl_vector_set(nu, 0, gslu_math_minimized_angle(ztmp - zhat));

    //jacobian observation model wrt state
    double sq_ratio = dy*dy/(dx*dx);
    gsl_matrix_set(H, 0, 0, dy/(dx*dx*(1+sq_ratio)));
    gsl_matrix_set(H, 0, 1, -1/(dx*(1+sq_ratio)));
    gsl_matrix_set(H, 0, 2, -1);
    
}

void
om_ukf_bearing (const est_estimator_t * state, const gsl_vector * state_sp,
                const void *index_map, const gsl_matrix *R, gsl_vector *z_pred,
                void *user) {
    
    meas_u_data_t * meas_u_data = user;
  
    //known marker position
    int marker_x[6] = {21, 242, 463, 463, 242, 21};
    int marker_y[6] = {0, 0, 0, 292, 292, 292};

    // given known marker id calcuated predicted observation
    double dx = marker_x[meas_u_data->marker_id-1] - gsl_vector_get(state_sp, 0);
    double dy = marker_y[meas_u_data->marker_id-1] - gsl_vector_get(state_sp, 1);
    double zhat = gslu_math_minimized_angle(atan2(dy, dx) - gsl_vector_get(state_sp, 2));
    
    gsl_vector_set(z_pred, 0, zhat);
    
}

double 
om_pf_bearing (const est_estimator_t * state, const gsl_vector * x_p,
               const gsl_vector *z, const void *index_map, const gsl_matrix *R,
               void *user ) {
    
    meas_u_data_t * meas_u_data = user;
    
    //known marker position
    int marker_x[6] = {21, 242, 463, 463, 242, 21};
    int marker_y[6] = {0, 0, 0, 292, 292, 292};

    // given known marker id calcuated predicted observation
    double dx = marker_x[meas_u_data->marker_id-1] - gsl_vector_get(x_p, 0);
    double dy = marker_y[meas_u_data->marker_id-1] - gsl_vector_get(x_p, 1);
    double zhat = gslu_math_minimized_angle( atan2(dy, dx) - gsl_vector_get(x_p, 2));

    //set the innovation
    double ztmp = gsl_vector_get(z,0);
    double nu = gslu_math_minimized_angle(ztmp - zhat);
    double sigma = gsl_matrix_get (R, 0, 0);
    double weight =  exp ( -1/(2*sigma) * nu*nu );
    
    return weight;
}


// ============================================================================
// PMFs
// ============================================================================
void
pm_ekf (const est_estimator_t * state, const gsl_vector * u, const double dt,
        gsl_vector * mu_bar, gsl_matrix * F, gsl_matrix * Q, void *user) {
      
    // typecast user as known type
    pred_u_data_t * pred_u_data = user;
 
    // if you want to you can use index map here
    double r1 = gsl_vector_get(u, 0); 
    double t = gsl_vector_get(u, 1); 
    double r2 = gsl_vector_get(u, 2);
    double x = gsl_vector_get(state->mu, 0);
    double y = gsl_vector_get(state->mu, 1);
    double theta = gsl_vector_get(state->mu, 2);
    
    //calculate noise covariance based on control input
    gsl_matrix * Rtmp = gsl_matrix_calloc(3,3);
    gsl_matrix_set(Rtmp, 0, 0, pred_u_data->alphas[0]*r1*r1+pred_u_data->alphas[1]*t*t);
    gsl_matrix_set(Rtmp, 1, 1, pred_u_data->alphas[2]*t*t+pred_u_data->alphas[3]*(r1*r1+r2*r2));
    gsl_matrix_set(Rtmp, 2, 2, pred_u_data->alphas[0]*r2*r2+pred_u_data->alphas[1]*t*t);
    // jacobian with respect to control action
    //J =   [-t*sin(theta + r1) cos(theta + r1) 0
    //        t*cos(theta + r1) sin(theta + r1) 0
    //        1                 0               1];
    gsl_matrix * Jtmp = gsl_matrix_calloc(3,3);
    gsl_matrix_set(Jtmp, 0, 0, -t*sin(theta + r1));
    gsl_matrix_set(Jtmp, 0, 1, cos(theta + r1));
    gsl_matrix_set(Jtmp, 1, 0, t*cos(theta + r1));
    gsl_matrix_set(Jtmp, 1, 1, sin(theta + r1));
    gsl_matrix_set(Jtmp, 2, 0, 1);
    gsl_matrix_set(Jtmp, 2, 2, 1);
    // Q = Jtmp*Q*Jtmp'
    gslu_blas_mmmT (Q, Jtmp, Rtmp ,Jtmp, NULL);
    gsl_matrix_free(Jtmp);
    gsl_matrix_free(Rtmp); 
    
    // calculate jacobian of state transition function wrt state
    //F =[1 0 -t*sin(theta + r1)
    //    0 1  t*cos(theta + r1)
    //    0 0  1 ];
    gsl_matrix_set_identity(F);
    gsl_matrix_set(F, 0, 2, -t*sin(theta + r1));
    gsl_matrix_set(F, 1, 2, t*cos(theta + r1));
    
    // predict the mean
    //x_bar = [x+t*cos(theta+r1)
    //        y+t*sin(theta+r1)
    //        minimizedAngle(theta+r1+r2)];
    gsl_vector_set(mu_bar, 0, x+t*cos(theta+r1));
    gsl_vector_set(mu_bar, 1, y+t*sin(theta+r1));
    gsl_vector_set(mu_bar, 2, gslu_math_minimized_angle(theta+r1+r2)); 
        
    //gslu_vector_printf(state->mu,"mu");
    //gslu_matrix_printf(state->Sigma,"Sigma");
    
    //calculate difference from matlab result to verify
}

void
pm_ukf (const est_estimator_t * state, const gsl_vector * state_sp,
        const gsl_vector * u_sp, const double dt, gsl_vector * x_bar,
        gsl_matrix * Q, void *user) {
 
    // if you want to you can use index map here
    double r1 = gsl_vector_get(u_sp, 0); 
    double t = gsl_vector_get(u_sp, 1); 
    double r2 = gsl_vector_get(u_sp, 2);
    double x = gsl_vector_get(state_sp, 0);
    double y = gsl_vector_get(state_sp, 1);
    double theta = gsl_vector_get(state_sp, 2);
    
    // predict the mean
    //x_bar = [x+t*cos(theta+r1)
    //        y+t*sin(theta+r1)
    //        minimizedAngle(theta+r1+r2)];
    gsl_vector_set(x_bar, 0, x+t*cos(theta+r1));
    gsl_vector_set(x_bar, 1, y+t*sin(theta+r1));
    gsl_vector_set(x_bar, 2, gslu_math_minimized_angle(theta+r1+r2)); 
        
    //gslu_vector_printf(state->mu,"mu");
    //gslu_matrix_printf(state->Sigma,"Sigma");
}

// process model noise function
void
unf (const est_estimator_t * state, const gsl_vector * u, const double dt, 
     gsl_matrix * Sigma_u, void *user ) {
    
    // typecast user as known type
    pred_u_data_t * pred_u_data = user;
 
    // if you want to you can use index map here
    double r1 = gsl_vector_get(u, 0); 
    double t = gsl_vector_get(u, 1); 
    double r2 = gsl_vector_get(u, 2);
    
    //calculate control noise covariance based on control input
    gsl_matrix_set_zero (Sigma_u);
    gsl_matrix_set(Sigma_u, 0, 0, pred_u_data->alphas[0]*r1*r1+pred_u_data->alphas[1]*t*t);
    gsl_matrix_set(Sigma_u, 1, 1, pred_u_data->alphas[2]*t*t+pred_u_data->alphas[3]*(r1*r1+r2*r2));
    gsl_matrix_set(Sigma_u, 2, 2, pred_u_data->alphas[0]*r2*r2+pred_u_data->alphas[1]*t*t);
    
}

void
pm_pf (const est_estimator_t * state, const gsl_vector * x_p, const gsl_vector * u,
       const double dt, const gsl_matrix * Q, gsl_vector * x_bar, void *user ) {
    
    
    // typecast user as known type
    pred_u_data_t * pred_u_data = user;
 
    // if you want to you can use index map here
    double r1 = gsl_vector_get(u, 0); 
    double t = gsl_vector_get(u, 1); 
    double r2 = gsl_vector_get(u, 2);
    double x = gsl_vector_get(x_p, 0);
    double y = gsl_vector_get(x_p, 1);
    double theta = gsl_vector_get(x_p, 2);
 
    //calculate noise covariance based on control input
    gsl_matrix * Sigma_u = gsl_matrix_calloc(3,3);
    gsl_matrix_set(Sigma_u, 0, 0, pred_u_data->alphas[0]*r1*r1+pred_u_data->alphas[1]*t*t);
    gsl_matrix_set(Sigma_u, 1, 1, pred_u_data->alphas[2]*t*t+pred_u_data->alphas[3]*(r1*r1+r2*r2));
    gsl_matrix_set(Sigma_u, 2, 2, pred_u_data->alphas[0]*r2*r2+pred_u_data->alphas[1]*t*t);
 
    // sample noisy control
    GSLU_VECTOR_VIEW (u_noisy, 3, {0});
    gsl_rng *rng = gslu_rand_rng_alloc ();
    gslu_rand_gaussian_vector (rng, &u_noisy.vector, u, Sigma_u, NULL);
    gsl_rng_free (rng);
    
    r1 = gsl_vector_get(&u_noisy.vector, 0); 
    t = gsl_vector_get(&u_noisy.vector, 1); 
    r2 = gsl_vector_get(&u_noisy.vector, 2);   
    
    // predict the mean
    gsl_vector_set(x_bar, 0, x+t*cos(theta+r1));
    gsl_vector_set(x_bar, 1, y+t*sin(theta+r1));
    gsl_vector_set(x_bar, 2, gslu_math_minimized_angle(theta+r1+r2)); 

}


// Main loop
int
main(int argc, char *argv[]){

    // test lib est using data from robocup bearing based simulation and compare
    // with MATLAB result
    
    // load data ---------------------------------------------------------------
    // load data file
    int data_len = 200;
    int data_fields = 13; // realObservation, noisefreeMotion, noisefreeObservation, realRobot,     noisefreeRobot
                          // [angle, id]      [r1, t, r2,]     [angle, id]           [x, y, theta]  [x, y, theta]   
    gsl_matrix * data = gsl_matrix_alloc (data_len, data_fields);
    FILE * f;
    f = fopen ("data.txt", "rb");
    if (f == NULL){ printf("Failed to open data.txt\n"); }
    gsl_matrix_fscanf (f, data);
    fclose (f);
    // load matlab results EKF
    int result_fields = 3;
    gsl_matrix * results_matlab_ekf = gsl_matrix_alloc (data_len, result_fields);
    f = fopen ("result_ekf.txt", "rb");
    if (f == NULL){ printf("Failed to open result_ekf.txt\n"); }
    gsl_matrix_fscanf (f, results_matlab_ekf);
    fclose (f);
    gsl_matrix * results_cov_matlab_ekf = gsl_matrix_alloc (result_fields, result_fields);
    f = fopen ("result_cov_ekf.txt", "rb");
    if (f == NULL){ printf("Failed to open result_cov_ekf.txt\n"); }
    gsl_matrix_fscanf (f, results_cov_matlab_ekf);
    fclose (f);
    // load matlab results for UKF
    gsl_matrix * results_matlab_ukf = gsl_matrix_alloc (data_len, result_fields);
    f = fopen ("result_ukf.txt", "rb");
    if (f == NULL){ printf("Failed to open result_ukf.txt\n"); }
    gsl_matrix_fscanf (f, results_matlab_ukf);
    fclose (f);
    gsl_matrix * results_cov_matlab_ukf = gsl_matrix_alloc (result_fields, result_fields);
    f = fopen ("result_cov_ukf.txt", "rb");
    if (f == NULL){ printf("Failed to open result_cov_ukf.txt\n"); }
    gsl_matrix_fscanf (f, results_cov_matlab_ukf);
    fclose (f);
    // load matlab results PF
    gsl_matrix * results_matlab_pf = gsl_matrix_alloc (data_len, result_fields);
    f = fopen ("result_pf.txt", "rb");
    if (f == NULL){ printf("Failed to open result_pf.txt\n"); }
    gsl_matrix_fscanf (f, results_matlab_pf);
    fclose (f);
    gsl_matrix * results_cov_matlab_pf = gsl_matrix_alloc (result_fields, result_fields);
    f = fopen ("result_cov_pf.txt", "rb");
    if (f == NULL){ printf("Failed to open result_cov_pf.txt\n"); }
    gsl_matrix_fscanf (f, results_cov_matlab_pf);
    fclose (f);
    
    printf("Test Data Loaded ----------------------------------------------\n");
    
    // init filters
    int state_len = 3;
    gsl_vector * ini_state = gsl_vector_calloc (state_len);
    gsl_matrix * ini_cov = gsl_matrix_calloc (state_len,state_len);
    gsl_vector_set (ini_state, 0, 180);
    gsl_vector_set (ini_state, 1, 50);
    gsl_vector_set (ini_state, 2, 0);
    gsl_matrix_set_identity (ini_cov);
    gsl_matrix_scale (ini_cov, 0.0001);
    
    // init EKF
    est_estimator_t *  estimator_ekf = est_init_ekf (state_len, ini_state, ini_cov);
    // init UKF
    int is_angle[3] = {0, 0, 1};
    est_estimator_t *  estimator_ukf = est_init_ukf (state_len, ini_state, ini_cov,
                                                     is_angle, 1, 2, 0);
    // init PF
    // TEST DATA GENERATED WITH 500,000 particles, wont match that closely unless we use a ton of particles
    est_estimator_t * estimator_pf = est_init_pf (state_len, ini_state, ini_cov,    
                                                    10000, is_angle);
    printf("Filter Inited -------------------------------------------------\n");
    
    // setup prediction model
    est_pred_t pred = {0};
    // process model function
    pred.pmf_ekf = (est_pmf_ekf_t) &pm_ekf;
    pred.pmf_ukf = (est_pmf_ukf_t) &pm_ukf;
    pred.pmf_pf = (est_pmf_pf_t) &pm_pf;
    pred.unf = (est_unf_t) &unf;
    pred.u = gsl_vector_calloc(3); // set .u during loop before each call
    pred.Q = gsl_matrix_calloc(state_len,state_len);
    pred_u_data_t * pred_u_data = calloc(1, sizeof(pred_u_data_t));
    pred.user = pred_u_data;
    pred_u_data->alphas[0] = 0.0025;
    pred_u_data->alphas[1] = 0.000001;
    pred_u_data->alphas[2] = 0.0025;
    pred_u_data->alphas[3] = 0.0001;
    
    // setup obversation models
    est_meas_t bearing_meas = {0};
    bearing_meas.omf_ekf = (est_omf_ekf_t) &om_ekf_bearing;
    bearing_meas.omf_ukf = (est_omf_ukf_t) &om_ukf_bearing;
    bearing_meas.omf_pf = (est_omf_pf_t) &om_pf_bearing;
    bearing_meas.R = gsl_matrix_calloc(1,1);
    gsl_matrix_set(bearing_meas.R, 0, 0, 0.121846967914683427);
    bearing_meas.index_map = NULL;
    bearing_meas.z = gsl_vector_calloc(1);
    int is_angle_om[1] = {1};
    bearing_meas.angle_mask = is_angle_om;
    meas_u_data_t * meas_u_data = calloc(1, sizeof(meas_u_data_t)); 
    bearing_meas.user = meas_u_data;
    // note .z and .user->marker_id will be set durring the loop before each call
  
    
    printf("Setup meas_t and pred_t ---------------------------------------\n");
    printf("Starting main loop --------------------------------------------\n");
    
    // MAIN LOOP ---------------------------------------------------------------
    for (int n = 0 ; n < data_len ; n++){
    //for (int n = 0 ; n < 10 ; n++){
        
        //printf("step %d --------------------------------------------\n", n);
        printf("%d..", n);
        fflush(stdout);
        
        // Prediciton Step
        // data(n,2:4); has r1, trans, r2
        gsl_vector_set(pred.u, 0, gsl_matrix_get(data, n, 2));
        gsl_vector_set(pred.u, 1, gsl_matrix_get(data, n, 3));
        gsl_vector_set(pred.u, 2, gsl_matrix_get(data, n, 4));
         
        est_predict (estimator_ekf, &pred);
        est_predict (estimator_ukf, &pred);
        est_predict (estimator_pf, &pred);
      
        // Update Step
        // data(n,0:1); has bearing, marker id
        gsl_vector_set(bearing_meas.z, 0, gsl_matrix_get(data, n, 0));
        meas_u_data->marker_id = (int)gsl_matrix_get(data, n, 1); 
        est_correct (estimator_ekf, &bearing_meas);
        est_correct (estimator_ukf, &bearing_meas);
        est_correct (estimator_pf, &bearing_meas);
        
        //printf("Step %d \n",n);
        //gslu_vector_printf(estimator_pf->mu, "Mean");
        //// compare with estimate from matlab
        //gsl_vector_view result_row = gsl_matrix_row (results_matlab_pf, n);
        //gsl_vector * diff = gsl_vector_alloc(3);
        //gsl_vector_memcpy (diff, estimator_pf->mu);
        //gsl_vector_sub(diff, &result_row.vector);
        //gslu_vector_printf(diff, "Diff from MATLAB");
        
        //gslu_vector_printf(&result_row.vector, "Mean_True");
        //gslu_matrix_printf(estimator_ukf->Sigma, "Covariance");
        
    } // END MAIN LOOP
    printf("\n");
    printf("Done main loop ------------------------------------------------\n");
    
    // print results
    printf("EKF Results ---------------------------------------------------\n");
    //gslu_matrix_printf(estimator_ekf->Sigma, "Covariance");
    //gslu_matrix_printf(results_cov_matlab_ekf, "Covariance Matlab");
    gsl_vector_view result_row = gsl_matrix_row (results_matlab_ekf, data_len-1);
    gsl_vector * diff = gsl_vector_alloc(3);
    gsl_vector_memcpy (diff, estimator_ekf->mu);
    gsl_vector_sub(diff, &result_row.vector);
    gslu_vector_printf(diff, "State Diff MATLAB");
    gsl_matrix_sub(results_cov_matlab_ekf, estimator_ekf->Sigma);
    gslu_matrix_printf(results_cov_matlab_ekf, "Covariance Diff");
    
    printf("UKF Results ---------------------------------------------------\n");
    result_row = gsl_matrix_row (results_matlab_ukf, data_len-1);
    gsl_vector_memcpy (diff, estimator_ukf->mu);
    gsl_vector_sub(diff, &result_row.vector);
    gslu_vector_printf(diff, "State Diff MATLAB");
    gsl_matrix_sub(results_cov_matlab_ukf, estimator_ukf->Sigma);
    gslu_matrix_printf(results_cov_matlab_ukf, "Covariance Diff");
    
    printf("PF Results ----------------------------------------------------\n");
    result_row = gsl_matrix_row (results_matlab_pf, data_len-1);
    gsl_vector_memcpy (diff, estimator_pf->mu);
    gsl_vector_sub(diff, &result_row.vector);
    gslu_vector_printf(diff, "State Diff MATLAB");
    gsl_matrix_sub(results_cov_matlab_pf, estimator_pf->Sigma);
    gslu_matrix_printf(results_cov_matlab_pf, "Covariance Diff");
    
    // clean up ----------------------------------------------------------------
    if(bearing_meas.user != NULL){ free(bearing_meas.user); }
    if(bearing_meas.z != NULL){ gsl_vector_free(bearing_meas.z); }
    if(bearing_meas.R != NULL){ gsl_matrix_free(bearing_meas.R); }
    if(pred.user != NULL){ free(pred.user); } 
    if(pred.u != NULL){ gsl_vector_free(pred.u); }
    if(pred.Q != NULL){ gsl_matrix_free(pred.Q); }
    
    est_free(estimator_ekf);
    est_free(estimator_ukf);
    
    gsl_matrix_free(data);
    gsl_matrix_free(results_matlab_ekf);
    gsl_matrix_free(results_cov_matlab_ekf);
    gsl_matrix_free(results_matlab_ukf);
    gsl_matrix_free(results_cov_matlab_ukf);
    gsl_matrix_free(results_matlab_pf);
    gsl_matrix_free(results_cov_matlab_pf);
    gsl_vector_free(ini_state);
    gsl_matrix_free(ini_cov);
    
    printf("Clean up finished, done ---------------------------------------\n");
}
