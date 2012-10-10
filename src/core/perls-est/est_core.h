#ifndef __PERLS_EST_CORE_H__
#define __PERLS_EST_CORE_H__

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


#define EST_MEAS_ID_STR_MAX_LEN 128


/**
 * @defgroup PerlsEstCore Core Estimation Types
 * @brief Core types and methods for estimation library
 * @ingroup PerlsEst
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif


enum _est_method_t
{
    EST_EKF = 1,
    EST_UKF = 2,
    EST_PF  = 3,
};
typedef enum _est_method_t est_method_t;

/**
 * @brief weight structure for ukf.
 *
 * @details
 * 
 * given \f$\alpha\f$, \f$\beta\f$ and \f$\kappa\f$ weights are calculated as:
 * \f[
 * n = length(mu_{aug})
 * \f] \f[
 * \lambda = \alpha^2(n+\kappa) - n
 * \f] \f[
 * \gamma = \sqrt{n + \lambda}
 * \f] \f[
 * w_m^0 = \frac{\lambda}{(n+\lambda)}
 * \f] \f[
 * w_m^i = \frac{1}{(2(n+\lambda))}
 * \f]\f[
 * w_c^0 = w_m^0 + (1-\alpha^2+\beta)
 * \f]\f[
 * w_c^i = w_m^i
 * \f]
 * 
 */
typedef struct _est_ukf_weights_t est_ukf_weights_t;
struct _est_ukf_weights_t
{
    // set by user
    double alpha;   //!< input: sigma point scaling (0 < alpha <= 1)
    double beta;    //!< input: higher order moment matching (0 <= beta)
    double kappa;   //!< input: scalar tuning param (0 <= kappa)
    
    // calculated
    int n;          //!< calculated: length of sigma pt
    double lambda;  //!< calculated: tmp weight factor
    double gamma;   //!< calculated: scale factor for chol decomp
    double w_m_0;   //!< calculated: mean center weight
    double w_m_i;   //!< calculated: mean offcenter weight
    double w_c_0;   //!< calculated: coviariance center weight
    double w_c_i;   //!< calculated: coviariance offcenter weight
};

/**
 * @brief Generic estimator structure for Extended Kalman Filter (EKF),
 * Unscented Kalman Filter (EKF) and Particle Filter (PF).
 *
 * @details TODO detailed description of usage
 *
 * @see est_meas_t
 * @see est_correct()
 * @see est_pred_t
 * @see est_predict()
 * 
 */
typedef struct _est_estimator_t est_estimator_t;
struct _est_estimator_t
{
    est_method_t est_method; //!< estimation method to use 'ekf', 'ukf', 'pf'
    int state_len;  //!< the length of the state vector
        
    gsl_vector *mu;
    gsl_matrix *Sigma;
    
    int num_particles; //!< number of particles used by PF
    gsl_matrix *particles; //!< particles (state x number of particles)
    int i_x_ml; //!< index of maximum liklihood state particle (highest probability particle)
    
    int *angle_mask; //!< bool vector to flag angular quanties in state vector, (avoids warpping errors for averages and innovations in EKF / UKF )
    
    est_ukf_weights_t ukf_weights; //!< ukf weights for filter
    
    //int64_t utime_pred; // time of last prediction
    //int64_t utime_meas; // time of last measurement
    
    // Delayed state settings
    int delayed_state_len;  //!< the length of a single delayed state
    int max_delayed_states; //!< maximum number of delayed states beyond which old will be marginalized out. -1 for no max
    int delayed_states_cnt; //!< current number of delayes states
    
    // Debugging values, access after calling est_predict
    int64_t last_utime;                             //!< utime of last measurment
    char last_id_str[EST_MEAS_ID_STR_MAX_LEN];      //!< string used to identify measurement
    gsl_vector *last_nu;                            //!< last innovation
    gsl_vector *last_z;                             //!< last innovation
    double last_nis;                                //!< last nis = nu'*inv(H*Sigma*H' + R)*nu;
    int last_mahal_innov_passed;                    //!< 1 if passed mahalnobis innovation test
};


// =============================================================================
// OBSERVATION MODEL TYPEDEFS
// =============================================================================

/**
 * @brief Extended Kalman filter (EKF) observation model callback function
 *
 * @details User must supplly callback function of this type for each observation
 * model if using the EKF.
 *
 * User function takes in the current state, and the observation and must
 * return the innovation vector (nu) and an observation jacobian (H).
 *
 * Additional information used in the observation model may be accessed through
 * the void user pointer.
 *
 * @param *state current state
 * @param *z measurement vector
 * @param *index_map maps function state indicies to state vector indicies, user defined type
 * @param *R input: measurement noise covariance 
 * @param *nu output: innovation vector
 * @param *H output: observation jacobian
 * @param *user point to struture of user defined data
 *
 * @see est_meas_t
 * @see est_correct()
 * 
 */
typedef void (*est_omf_ekf_t)
    (
        const est_estimator_t *state,   //!< input: current state
        const gsl_vector *z,     //!< input: measurement vector
        const void *index_map,   //!< input: maps function state indicies to state vector indicies, user defined type
        const gsl_matrix *R,     //!< input: measurement noise covariance 
        
        gsl_vector *nu,          //!< output: innovation vector
        gsl_matrix *H,           //!< output: observation jacobian
        
        void *user               //!< point to struture of user defined data
    );


/**
 * @brief Unscented Kalman filter (UKF) observation model callback function
 *
 * @details User must supplly callback function of this type for each observation
 * model if using the UKF.
 *
 * User function takes in the sigma point for the current state and must
 * return the predicted measurment (z_pred).
 *
 * Additional information used in the observation model may be accessed through
 * the void user pointer.
 *
 * @param *state  input: current estimator state
 * @param *x_sp   input: state sigma point
 * @param *index_map          input: maps function state indicies to state vector indicies, user defined type
 * @param *R            input: measurement noise covariance 
 * @param *z_pred          output: predicted measurement for this sigma point
 * @param *user               point to struture of user defined data

 * @see est_meas_t
 * @see est_correct()
 * 
 */
typedef void (*est_omf_ukf_t)
    (
        const est_estimator_t *state,  //!< input: current estimator state
        const gsl_vector *x_sp,        //!< input: state sigma point
        const void *index_map,         //!< input: maps function state indicies to state vector indicies, user defined type
        const gsl_matrix *R,           //!< input: measurement noise covariance 
        
        gsl_vector *z_pred,            //!< output: predicted measurement for this sigma point
        
        void *user                     //!< point to struture of user defined data
    );
        
/**
 * @brief particle filter (PF) observation model callback function
 *
 * @details User must supplly callback function of this type for each observation
 * model if using PF.
 *
 * User function takes in the current state particle, and the observation and must
 * return the importance weight for this particle.
 *
 * Additional information used in the observation model may be accessed through
 * the void user pointer.
 *
 * @return output: importance weight for this particle
 *
 * @param *state     input: current state
 * @param *x_p    input: state particle
 * @param *z       input: measurement vector
 * @param *index_map     input: maps function state indicies to state vector indicies, user defined type
 * @param *R      input: measurement noise covariance 
 * @param *user point to struture of user defined data
 * 
 * @see est_meas_t
 * @see est_correct()
 * 
 */
typedef double (*est_omf_pf_t)
    (
        const est_estimator_t *state,   //!< input: current state
        const gsl_vector *x_p,   //!< input: state particle
        const gsl_vector *z,     //!< input: measurement vector
        const void *index_map,   //!< input: maps function state indicies to state vector indicies, user defined type
        const gsl_matrix *R,     //!< input: measurement noise covariance 
        
        void *user               //!< point to struture of user defined data
    );

/**
 * @brief Measurement structure filled then passed to est_correct()
 * 
 * @details Fill this measurement structure and pass it to est_correct() to
 * perfom a correction step.
 *
 * If observation models for each filter type are not avaliable or not used
 * the unused om callback pointers can be set to NULL.
 *
 * @see est_correct()
 * @see est_omf_ekf_t
 * @see est_omf_ukf_t
 * @see est_omf_pf_t
 */
typedef struct _est_meas_t est_meas_t;
struct _est_meas_t
{
    // observation model callback functions
    est_omf_ekf_t omf_ekf;  //!< Pointer to EKF observation model callback
    est_omf_ukf_t omf_ukf;  //!< Pointer to UKF observation model callback
    est_omf_pf_t omf_pf;    //!< Pointer to PF observation model callback
    
    gsl_vector *z;    //!< measurement vector
    gsl_matrix *R;    //!< measurement noise covariance
    
    void *index_map;  //!< input: maps function state indicies to state vector indicies, user defined type
    void *user;       //!< pointer to struture defined by user to supply additional information the the observation model function
    int64_t utime;    //!< time measurement was received
    
    int use_innov_mahal_test; //!< set to 0 to not use
    double innov_mahal_test_thresh; //!< threshold (in percentage ie 0.99) to test innovation before accepting measurement
    
    est_ukf_weights_t *ukf_weights; //!< ukf weights for this measurement (overide default if used)
    
    int *angle_mask; //!< bool vector to flag angular quanties in state vector, avoids warpping errors for averages and innovations in UKF and PF
    
    // for debugging    
    char id_str[EST_MEAS_ID_STR_MAX_LEN];  //!< string used to identify measurement
};


// =============================================================================
// PREDICTION MODEL TYPEDEFS
// =============================================================================

/**
 * @brief Extended Kalman filter (EKF) process model callback function
 *
 * @details User must supplly callback function of this type for the process
 * model if the estimator is set to use the EKF.
 *
 * User function takes in the current state, and the control vector and must
 * return the predicted state (x_bar) and an process jacobian (F).
 *
 * The process noise, Q, will be treated as additive, and maybe updated during
 * the callback if nesscary. Often this will happen if process noise is dependent
 * on the control action performed.
 *
 * Additional information used in the process model may be accessed through the void
 * user pointer.
 *

 * @param *state   input: current state
 * @param *u      input: control vector
 * @param dt          input: delta time to predict over (in seconds)
 * @param *mu_bar       output: predicted state mean
 * @param *F            output: jacobian with respect to state
        
 * @param *Q            output: process noise covariance (update if needed)
        
 * @param *user                 point to struture of user defined data

 
 *
 * @see est_pred_t
 * @see est_correct()
 * 
 */
typedef void (*est_pmf_ekf_t)
    (
        const est_estimator_t *state,  //!< input: current state
        const gsl_vector *u,      //!< input: control vector
        const double dt,          //!< input: delta time to predict over (in seconds)
        
        gsl_vector *mu_bar,       //!< output: predicted state mean
        gsl_matrix *F,            //!< output: jacobian with respect to state
        
        gsl_matrix *Q,            //!< output: process noise covariance (update if needed)
        
        void *user                //!< point to struture of user defined data
    );
        
/**
 * @brief Unscented Kalman filter (UKF) process model callback function
 *
 * @details User must supplly callback function of this type for the process
 * model if the estimator is set to use the UKF.
 *
 * User function takes in a sigma point for the state and control and must
 * return the predicted state (x_bar).
 *
 * Additional information used in the process model may be accessed through the
 * void user pointer.
 *
 * @param *state    input: current estimator state
 * @param *x_sp     input: state sigma point
 * @param *u_sp     input: control vector sigma point
 * @param dt        input: delta time to predict over (in seconds)
        
 * @param *x_bar    output: predicted state for this sigma point
        
 * @param *Q        output: process noise covariance (update if needed)
        
 * @param *user     point to struture of user defined data

 * @see est_pred_t
 * @see est_correct()
 * 
 */
typedef void (*est_pmf_ukf_t)
    (
        const est_estimator_t *state,  //!< input: current estimator state
        const gsl_vector *x_sp,   //!< input: state sigma point
        const gsl_vector *u_sp,   //!< input: control vector sigma point
        const double dt,          //!< input: delta time to predict over (in seconds)
        
        gsl_vector *x_bar,        //!< output: predicted state for this sigma point
        
        gsl_matrix *Q,            //!< output: process noise covariance (update if needed)
        
        void *user                //!< point to struture of user defined data
    );        

/**
 * @brief particle filter (PF) process model callback function
 *
 * @details User must supplly callback function of this type for the process
 * model if the estimator is set to use the PF.
 *
 * User function takes in a state particle, and the control vector and must sample
 * from the process noise and return the predicted state sample (x_bar).
 *
 * Additional information used in the process model may be accessed through the void
 * user pointer.
 *
 * @param *state  input: current estimator state
 * @param *x_p   input: state particle
 * @param *u     input: control vector
 * @param dt           input: delta time to predict over (in seconds)
 * @param *Q     input: process noise covariance
 * @param *x_bar       output: predicted state for this particle
 * @param *ser                point to struture of user defined data
 * @see est_pred_t
 * @see est_correct()
 * 
 */
typedef void (*est_pmf_pf_t)
    (
        const est_estimator_t *state,  //!< input: current estimator state
        const gsl_vector *x_p,    //!< input: state particle
        const gsl_vector *u,      //!< input: control vector
        const double dt,          //!< input: delta time to predict over (in seconds)
        const gsl_matrix *Q,      //!< input: process noise covariance
        
        gsl_vector *x_bar,        //!< output: predicted state for this particle
        
        void *user                //!< point to struture of user defined data
    );
        
/**
 * @brief control (u) noise callback function
 *
 * @details Required for UKF filters if control vector is used.
 *
 * Used to find the control noise based on the state, control vector or
 * additional user supplided data before processing the UKF prediction step. 
 *
 * @param *state  input: current estimator state
 * @param *u     input: control vector
 * @param dt          input: delta time to predict over (in seconds)
 * @param *Sigma_u     output: control noise covariance (prefilled with est_pred_t->Sigma_u if avaliable)
 * @param *ser                point to struture of user defined data
 *
 * @see est_correct()
 * 
 */
typedef void (*est_unf_t)
    (
        const est_estimator_t *state,  //!< input: current estimator state
        const gsl_vector *u,      //!< input: control vector
        const double dt,          //!< input: delta time to predict over (in seconds)
        
        gsl_matrix *Sigma_u,      //!< output: control noise covariance (prefilled with est_pred_t->Sigma_u if avaliable)
        
        void *user                //!< point to struture of user defined data
    );        

/**
 * @brief Prediction structure filled then passed to est_predict()
 * 
 * @details Fill this prediction structure and pass it to est_predict() to
 * perfom a prediction step.
 *
 * If prediction models for each filter type are not avaliable or not used
 * the unused pm callback pointers can be set to NULL.
 *
 * Setting pnf is not required if the process noise is constant, simply set it
 * to NULL and set the desired Q in this meas_t structure.
 *
 * @see est_predict()
 * @see est_pmf_ekf_t
 * @see est_pmf_ukf_t
 * @see est_pmf_pf_t
 * @see est_pnf_t
 */
typedef struct _est_pred_t est_pred_t;
struct _est_pred_t
{
    // process model function
    est_pmf_ekf_t pmf_ekf;  //!< Pointer to EKF process model callback
    est_pmf_ukf_t pmf_ukf;  //!< Pointer to UKF process model callback
    est_pmf_pf_t pmf_pf;    //!< Pointer to PF process model callback
           
    void *user;             //!<  pointer to struture defined by user to supply additional information the the observation model function
    
    gsl_vector *u;          //!<  input (optional): control vector
    double dt;              //!<  input (optional): delta time to predict over in seconds
    gsl_matrix *Q;          //!<  input (optional): process noise covariance
    gsl_matrix *Sigma_u;    //!<  input (optional): control noise covariance 
    est_unf_t unf;          //!<  input (optional): Pointer to control noise callback
    est_ukf_weights_t *ukf_weights; //!< input (optional): ukf weights for this measurement (overide default if used)

};


// =============================================================================
// CORE FUNCTIONS
// =============================================================================

/**
 * @brief Initialize the estimator_t structure for an EKF filter
 *
 * @return an allocated pointer to the new estimator structure
 *
 * @see est_estimator_t
 */
est_estimator_t *
est_init_ekf (int state_len,               //!< length of the state vector
              gsl_vector *ini_state,       //!< initial state vector
              gsl_matrix *ini_cov          //!< initial covariance
             );

/**
 * @brief Initialize the estimator_t structure for an UKF filter
 *
 * @return an allocated pointer to the new estimator structure
 *
 * @see est_estimator_t
 */
est_estimator_t *
est_init_ukf (int state_len,            //!< length of the state vector
              gsl_vector *ini_state,    //!< initial state vector
              gsl_matrix *ini_cov,      //!< initial covariance
              int *angle_mask,          //!< binary mask if need to treat as circular angular quantity
              double alpha,             //!< sigma point scaling (0 < alpha <= 1)
              double beta,              //!< higher order moment matching (0 <= beta)
              double kappa              //!< scalar tuning param (0 <= kappa)
             );

/**
 * @brief Initialize the estimator_t structure for an PF filter
 *
 * @details Initialize the estimator_t structure for an PF filter
 *
 * @return an allocated pointer to the new estimator structure
 *
 * @see est_estimator_t
 */
est_estimator_t *
est_init_pf (int state_len,            //!< length of the state vector
             gsl_vector *ini_state,    //!< initial state vector
             gsl_matrix *ini_cov,      //!< initial covariance
             int num_particles,        //!< number of particles
             int *angle_mask           //!< binary mask if need to treat as circular angular quantity
             );


/**
 * @brief Initialize delayed state configuration
 *
 * @details Currently only works for EKF based filters
 *
 * @see est_estimator_t
 */
void
est_init_ds (est_estimator_t *estimator,  //!< previsouly initialized estimator strucutre
             int delayed_state_len,       //!< length of the delayed state
             int max_delayed_states       //!< maximum number of delayed states beyond which old will be marginalized out. -1 for no max 
             );

/**
 * @brief Initialize the estimator_t structure for an EKF filter
 *
 * @details Initialize the estimator_t structure for an EKF filter
 *
 * @see est_estimator_t
 */
void
est_free (est_estimator_t *state);

/**
 * @brief Preform a prediction step
 *
 * @details Given a pred_t structure and the current estimator state this function
 * preforms a prediction step
 *
 * FOR EKF: calls callback in est_pred_t->pmf_ekf then using output
 * \mu = \bar{\mu}
 * \Sigma = F \Sigma F^{\top} + Q
 *
 * @see est_estimator_t
 * @see est_pred_t
 */
void
est_predict (est_estimator_t *state,     //!< lib est state
             est_pred_t *pred            //!< prediction structure 
             );

/**
 * @brief Preform a prediction step with delayed state
 *
 * @details Given a pred_t structure and the current estimator state this function
 * preforms a prediction step and for a delayed state filter (no augmentation)
 *
 * FOR EKF: calls callback in est_pred_t->pmf_ekf then using output
 * \mu = \bar{\mu}
 * \Sigma = F \Sigma F^{\top} + Q
 *
 * @see est_estimator_t
 * @see est_pred_t
 */
void
est_predict_ds (est_estimator_t *state, //!< lib est state
                est_pred_t *pred,       //!< prediction structure
                int do_augment          //!< augment new pose during this predict?
                );


/**
 * @brief Preform a correction step
 *
 * @details Given a meas_t structure and the current estimator state this function
 * preforms a measurement step
 *
 * @see est_estimator_t
 * @see est_meas_t
 */
void
est_correct (est_estimator_t *state,     //!< lib est state
             est_meas_t *meas            //!< measurement structure
             );

/**
 * @brief Perform a correction step on a specific delayed state.
 *
 * @details Given a meas_t structure, the current esetimator state, and a delayed state
 * index, this function performs a correction by sending the delayed state ds_ind to 
 * the observation model contained in meas_t struct.
 *
 * @see est_estimator_t
 * @see est_meas_t
 */
void
est_correct_ds (est_estimator_t *state,     //!< lib est state
                    est_meas_t *meas,           //!< measurement structure
                    int ds_ind                  //!< index of delayed state to apply meas to
                    );
/**
 * @brief Marginalize out a delayed state
 *
 * @details For delayed state filters this will marginalize out a delayed state at the given index
 *
 * @see est_estimator_t
 */
void
est_marginalize_ds (est_estimator_t *state,     //!< lib est state
                    int ds_ind                  //!< index of delayed state to marginalize 
                    );


/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif //__PERLS_EST_CORE_H__
