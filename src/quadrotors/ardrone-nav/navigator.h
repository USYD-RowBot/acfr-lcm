#ifndef __PERLS_ARDRONE_NAVIGATOR_H__
#define __PERLS_ARDRONE_NAVIGATOR_H__

#include <map>
#include <algorithm>
#include <list>
#include <deque>
#include "math.h"
#include "perls-math/ssc.h"
#include "perls-est/est.h"
#include "perls-math/gsl_util_blas.h"
#include "perls-math/gsl_util_math.h"
#include "perls-math/gsl_util_matrix.h"
#include "perls-math/gsl_util_vector.h"

#include "perls-lcmtypes/senlcm_mocap_t.h"
#include "perls-lcmtypes/perllcm_position_t.h"
#include "perls-lcmtypes/perllcm_ardrone_state_t.h"
#include "perls-lcmtypes/perllcm_ardrone_drive_t.h"
#include "perls-lcmtypes/perllcm_tag_detection_collection_t.h"
#include "perls-lcmtypes/bot_core_image_sync_t.h"

enum obs_type {
    BASE_OBS = 0,
    APRIL_OBS,
    MOCAP_OBS,
    STATE_OBS,
    SYNC_OBS,
    TIME_OBS,
    CONTROL_OBS,
};

class observation {
public:
    virtual obs_type
    get_type (void) { return BASE_OBS; }
};

class april_obs : public observation {
public:
    perllcm_tag_detection_collection_t *obs;

    april_obs (perllcm_tag_detection_collection_t *o) : obs(o) {}
    ~april_obs (void) { perllcm_tag_detection_collection_t_destroy (obs); }

    obs_type
    get_type (void) { return APRIL_OBS; }
};

class mocap_obs : public observation {
public:
    senlcm_mocap_t *obs;

    mocap_obs (senlcm_mocap_t *o) : obs(o) {}
    ~mocap_obs (void) { senlcm_mocap_t_destroy (obs); }

    obs_type
    get_type (void) { return MOCAP_OBS; }
};

class state_obs : public observation {
public:
    perllcm_ardrone_state_t *obs;

    state_obs (perllcm_ardrone_state_t *o) : obs(o) {}
    ~state_obs (void) { perllcm_ardrone_state_t_destroy (obs); }

    obs_type
    get_type (void) { return STATE_OBS; }
};

class sync_obs : public observation {
public:
    bot_core_image_sync_t *obs;

    sync_obs (bot_core_image_sync_t *o) : obs(o) {}
    ~sync_obs (void) { bot_core_image_sync_t_destroy (obs); }

    obs_type
    get_type (void) { return SYNC_OBS; }
};

class time_obs : public observation {
public:
    int64_t obs;

    time_obs (int64_t o) : obs(o) {}
    ~time_obs (void) {}

    obs_type
    get_type (void) { return TIME_OBS; }
};

class control_obs : public observation {
public:
    perllcm_ardrone_drive_t *obs;

    control_obs (perllcm_ardrone_drive_t *o) : obs(o) {}
    ~control_obs (void) { perllcm_ardrone_drive_t_destroy (obs); }

    obs_type
    get_type (void) { return CONTROL_OBS; }
};

struct snapshot {
    int64_t prev_utime;
    bool flying;
    bool mocap_viewed;
    bool state_viewed;
    perllcm_ardrone_drive_t *control_input;

    snapshot (int64_t u, bool f, bool m, bool s, perllcm_ardrone_drive_t *c)
        : prev_utime (u), flying (f), mocap_viewed (m), state_viewed (s) {
        if (c) this->control_input = perllcm_ardrone_drive_t_copy (c);
        else control_input = NULL;
    }
    ~snapshot (void) {
        if (control_input) perllcm_ardrone_drive_t_destroy (control_input);
    }
};


class navigator {
public:
    /* default constructor */
    navigator (void);
    navigator (double *, bool);

    /* destructor */
    ~navigator (void);

    /* general init called by constructors */
    void init (double *, bool);

    /* for process model function */
    void set_control_input (perllcm_ardrone_drive_t *);

    /* observation functions */
    void add_time_observation (int64_t utime);
    void add_april_observation (perllcm_tag_detection_collection_t *);
    void add_mocap_observation (senlcm_mocap_t *);
    void add_state_observation (perllcm_ardrone_state_t *);
    void add_sync_observation (bot_core_image_sync_t *);

    /* quadrotor state vector interface */
    bool is_quadrotor_viewed () {return quad_viewed;}
    perllcm_position_t * get_quadrotor_state ();

    /* target state vector interface */
    bool is_target_viewed () {return target_viewed;}
    perllcm_position_t * get_target_state ();

    gsl_vector *get_quad_to_cam () {return x_quad_to_cam;}
    gsl_vector *get_tag_to_target () {return x_active_tag_to_target;}

    bool is_flying () {return flying;}

private:
    int64_t prev_utime;

    bool flying;
    bool quad_viewed;
    bool target_viewed;
    bool mocap_viewed;
    bool state_viewed;

    bool track_target;

    /* for replaying observations after initializing target */
    std::list<observation*> replay_observations;
    std::list<snapshot*> replay_internal_snapshot;
    std::deque<int64_t> delayed_state_utimes;

    perllcm_ardrone_drive_t *control_input;

    /* estimator variables */
    est_estimator_t *estimator_ekf;
    est_pred_t *estimator_pred;
    est_meas_t *meas_quad_mocap;
    est_meas_t *meas_quad_state;
    est_meas_t *meas_april_tag;

    /* useful gsl vectors/matrices */
    gsl_vector *x_quad_to_cam;
    std::map<int, gsl_vector*> x_tag_to_target;
    gsl_vector *x_active_tag_to_target;
    gsl_vector *initial_mocap_obs;
    gsl_vector *initial_mocap_mu;
    gsl_matrix *initial_mocap_Sigma;
    gsl_vector *initial_state_obs;
    gsl_vector *initial_state_mu;
    gsl_matrix *initial_state_Sigma;

    /* gsl noise terms */
    gsl_matrix *obs_noise_mocap;
    gsl_matrix *obs_noise_state;
    gsl_matrix *obs_noise_april;

    /* initializes filter for quadrotor only */
    void initialize_filter (gsl_vector *);

    /* prediction propagation step */
    void predict (int64_t utime, bool drop_delayed_state);
    
    /* adds target from state vector */
    void add_target (gsl_vector *, int ds_ind);

    /* removes target from state vector */
    void remove_target ();

    /* runs state estimator through replay data */
    void replay (int64_t ds_utime);

    void publish_debug (void);
};




// Process model function
void
pm_ekf (const est_estimator_t *state, const gsl_vector *u, const double dt,
        gsl_vector *mu_bar, gsl_matrix *F, gsl_matrix *Q, void *user);

// Observation model functions
void
om_ekf_quad_state (const est_estimator_t *state, const gsl_vector *z,
        const void *index_map, const gsl_matrix *R,
        gsl_vector *nu, gsl_matrix *H, void *user);

void
om_ekf_quad_mocap (const est_estimator_t *state, const gsl_vector *z,
        const void *index_map, const gsl_matrix *R,
        gsl_vector *nu, gsl_matrix *H, void *user);

void
om_ekf_april_tag (const est_estimator_t *state, const gsl_vector *z,
        const void *index_map, const gsl_matrix *R,
        gsl_vector *nu, gsl_matrix *H, void *user);

#endif // __PERLS_ARDRONE_NAVIGATOR_H__
