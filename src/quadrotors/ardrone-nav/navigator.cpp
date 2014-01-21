#include "navigator.h"
#include <bot_lcmgl_client/lcmgl.h>

// for debug messages
#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_lcmplot_t.h"

//----------------------------------------------------------------------------------
// Default constructor
//----------------------------------------------------------------------------------
navigator::navigator (void)
{
    init (NULL, false);
}

navigator::navigator (double *x_vs, bool track_target)
{
    init (x_vs, track_target);
}

//----------------------------------------------------------------------------------
// Destructor 
//----------------------------------------------------------------------------------
navigator::~navigator (void)
{
    if (this->control_input) perllcm_ardrone_drive_t_destroy (control_input);

    gslu_vector_free (this->x_quad_to_cam);
    gslu_vector_free (this->initial_mocap_obs);
    gslu_vector_free (this->initial_mocap_mu);
    gslu_matrix_free (this->initial_mocap_Sigma);
    gslu_vector_free (this->initial_state_obs);
    gslu_vector_free (this->initial_state_mu);
    gslu_matrix_free (this->initial_state_Sigma);

    gslu_matrix_free (this->obs_noise_mocap);
    gslu_matrix_free (this->obs_noise_state);
    gslu_matrix_free (this->obs_noise_april);

    for (std::map<int, gsl_vector*>::iterator it = this->x_tag_to_target.begin(); it != this->x_tag_to_target.end(); ++it)
        gslu_vector_free ((*it).second);
}

//----------------------------------------------------------------------------------
// Initializes data structures -- called by all constructors
//----------------------------------------------------------------------------------
void
navigator::init (double *quad_to_cam, bool track_target)
{
    this->prev_utime = 0;
    this->quad_viewed = false;
    this->target_viewed = false;
    this->mocap_viewed = false;
    this->state_viewed = false;

    this->track_target = track_target;

    this->control_input = NULL;

    this->estimator_ekf = NULL;

    this->x_quad_to_cam = NULL;
    if (quad_to_cam) {
        this->x_quad_to_cam = gsl_vector_calloc (6);
        gsl_vector_set (this->x_quad_to_cam, 0, quad_to_cam[0]);
        gsl_vector_set (this->x_quad_to_cam, 1, quad_to_cam[1]);
        gsl_vector_set (this->x_quad_to_cam, 2, quad_to_cam[2]);
        gsl_vector_set (this->x_quad_to_cam, 3, quad_to_cam[3]);
        gsl_vector_set (this->x_quad_to_cam, 4, quad_to_cam[4]);
        gsl_vector_set (this->x_quad_to_cam, 5, quad_to_cam[5]);
    }
    this->initial_mocap_obs   = NULL;
    this->initial_mocap_mu    = NULL;
    this->initial_mocap_Sigma = NULL;
    this->initial_state_obs   = NULL;
    this->initial_state_mu    = NULL;
    this->initial_state_Sigma = NULL;

    /* initialize tag transforms */
    this->x_tag_to_target[6]   = gsl_vector_calloc (6);
    this->x_tag_to_target[535] = gsl_vector_calloc (6);
    this->x_tag_to_target[442] = gsl_vector_calloc (6);
    this->x_tag_to_target[441] = gsl_vector_calloc (6);
    this->x_tag_to_target[443] = gsl_vector_calloc (6);
    //gsl_vector_set (this->x_tag_to_target[442], 0, -0.264);

    gsl_vector_set (this->x_tag_to_target[442], 0, -0.462);

    gsl_vector_set (this->x_tag_to_target[441], 0, -0.462);
    gsl_vector_set (this->x_tag_to_target[441], 1, 0.204);

    gsl_vector_set (this->x_tag_to_target[443], 0, -0.462);
    gsl_vector_set (this->x_tag_to_target[443], 1, -0.202);
    x_active_tag_to_target     = NULL;


    /* initialize observation noise */
    this->obs_noise_mocap = gsl_matrix_calloc (6,6);
    gsl_matrix_set_identity (this->obs_noise_mocap);
    gsl_matrix_scale (this->obs_noise_mocap, 0.0009);

    this->obs_noise_state = gsl_matrix_calloc (6,6);
    gsl_matrix_set (this->obs_noise_state, 0, 0, 0.12);
    gsl_matrix_set (this->obs_noise_state, 1, 1, 0.0019);
    gsl_matrix_set (this->obs_noise_state, 2, 2, 0.0019);
    gsl_matrix_set (this->obs_noise_state, 3, 3, 0.0019);
    gsl_matrix_set (this->obs_noise_state, 4, 4, 0.02);
    gsl_matrix_set (this->obs_noise_state, 5, 5, 0.02);

    this->obs_noise_april = gsl_matrix_calloc (7,7);
    gsl_matrix_set_identity (this->obs_noise_april);
    gsl_matrix_scale (this->obs_noise_april, 0.36); /* may need more tuning */
    gsl_matrix_set (this->obs_noise_april, 0, 0, 0.09);
    gsl_matrix_set (this->obs_noise_april, 1, 1, 0.09);
    gsl_matrix_set (this->obs_noise_april, 2, 2, 0.09);
    gsl_matrix_set (this->obs_noise_april, 0, 0, 0.19);
    gsl_matrix_set (this->obs_noise_april, 1, 1, 0.19);
    gsl_matrix_set (this->obs_noise_april, 2, 2, 0.19);
    gsl_matrix_set (this->obs_noise_april, 3, 3, 0.4);
    gsl_matrix_set (this->obs_noise_april, 4, 4, 0.4);
    gsl_matrix_set (this->obs_noise_april, 5, 5, 0.4);
    gsl_matrix_set (this->obs_noise_april, 6, 6, 0.19);

    /* initialize prediction structure */
    this->estimator_pred = (est_pred_t*) malloc (sizeof (est_pred_t));
    this->estimator_pred->pmf_ekf = (est_pmf_ekf_t) &pm_ekf;
    int state_length = 12;
    this->estimator_pred->Q = gsl_matrix_calloc (state_length, state_length);
    gsl_matrix_set (this->estimator_pred->Q, 6, 6, 1);
    gsl_matrix_set (this->estimator_pred->Q, 7, 7, 1);
    gsl_matrix_set (this->estimator_pred->Q, 8, 8, 1);
    gsl_matrix_set (this->estimator_pred->Q, 9, 9, 1);
    gsl_matrix_set (this->estimator_pred->Q, 10, 10, 1);
    gsl_matrix_set (this->estimator_pred->Q, 11, 11, 1);

    /* initialize ardrone state observations */
    this->meas_quad_state = (est_meas_t*) malloc (sizeof (est_meas_t));
    this->meas_quad_state->omf_ekf = (est_omf_ekf_t) om_ekf_quad_state;
    this->meas_quad_state->R = gsl_matrix_calloc (6,6);
    gsl_matrix_memcpy (this->meas_quad_state->R, this->obs_noise_state);
    this->meas_quad_state->z = gsl_vector_calloc (6);
    int is_angle_om1[6] = {0, 1, 1, 1, 0, 0};
    this->meas_quad_state->angle_mask = is_angle_om1;
    strcpy (this->meas_quad_state->id_str, "QUAD_STATE_OBS");

    /* initialize ardrone mocap observations */
    this->meas_quad_mocap = (est_meas_t*) malloc (sizeof (est_meas_t));
    this->meas_quad_mocap->omf_ekf = (est_omf_ekf_t) om_ekf_quad_mocap;
    this->meas_quad_mocap->R = gsl_matrix_calloc (6,6);
    gsl_matrix_memcpy (this->meas_quad_mocap->R, this->obs_noise_mocap);
    this->meas_quad_mocap->z = gsl_vector_calloc (6);
    int is_angle_om2[6] = {0, 0, 0, 1, 1, 1};
    this->meas_quad_mocap->angle_mask = is_angle_om2;
    strcpy (this->meas_quad_mocap->id_str, "QUAD_MOCAP_OBS");

    /* initialize april tag observations */
    this->meas_april_tag = (est_meas_t*) malloc (sizeof (est_meas_t));
    this->meas_april_tag->omf_ekf = (est_omf_ekf_t) om_ekf_april_tag;
    this->meas_april_tag->R = gsl_matrix_calloc (7,7);
    gsl_matrix_memcpy (this->meas_april_tag->R, this->obs_noise_april);
    this->meas_april_tag->z = gsl_vector_calloc (7);
    this->meas_april_tag->angle_mask = is_angle_om2;
    strcpy (this->meas_april_tag->id_str, "APRIL_TAG_OBS");
}

//----------------------------------------------------------------------------------
// Kalman Fitler Prediction step
//----------------------------------------------------------------------------------
void
navigator::predict (int64_t utime, bool drop_delayed_state)
{
    /* format prediction */
    double delta_t = (utime - this->prev_utime) / 1e6;

    /* turn crank on kalman filter */
    if (this->prev_utime > 0 && delta_t > 0 && this->estimator_ekf) {

        /* do prediction */
        this->estimator_pred->user = (void *) this;
        this->estimator_pred->dt = delta_t;
        est_predict_ds (this->estimator_ekf, this->estimator_pred, drop_delayed_state ? 1:0);
        if (drop_delayed_state) this->delayed_state_utimes.push_back (utime);

    }

    /* only advance time if time increases */
    if (utime > this->prev_utime)
        this->prev_utime = utime;

    /* clear out and marginalize out "old" stuff */
    while (this->delayed_state_utimes.size () > 5) {
        /* marginalize in est */
        est_marginalize_ds (this->estimator_ekf, this->delayed_state_utimes.size ());
        
        /* remove from queue */
        this->delayed_state_utimes.pop_front ();
    }

    /* clear out old replay messages */
    while (!this->replay_internal_snapshot.empty () && (this->prev_utime > (this->replay_internal_snapshot.front()->prev_utime + 5e5))) {
        observation *o = this->replay_observations.front ();
        switch (o->get_type ()) {
            case APRIL_OBS:
                delete (april_obs*) o;
                break;
            case MOCAP_OBS:
                delete (mocap_obs*) o;
                break;
            case STATE_OBS:
                delete (state_obs*) o;
                break;
            case SYNC_OBS:
                delete (sync_obs*) o;
                break;
            case TIME_OBS:
                delete (time_obs*) o;
                break;
            case CONTROL_OBS:
                delete (control_obs*) o;
                break;
            case BASE_OBS:
                delete o;
                break;
        }
        replay_observations.pop_front ();

        delete replay_internal_snapshot.front ();
        replay_internal_snapshot.pop_front ();
    }

    /* check to marginalize out target, by looking at *relative* uncertainty */
    if (target_viewed) {
        gsl_vector *x_vT = gsl_vector_calloc (6);
        gsl_matrix *J = gsl_matrix_calloc (6,12);
        gsl_matrix *Sig_vT = gsl_matrix_calloc (6,6);
        gsl_matrix *Sig_positional = gsl_matrix_calloc (12,12);
        gsl_vector_view x_lv = gsl_vector_subvector (this->estimator_ekf->mu, 0, 6);
        gsl_vector_view x_lT = gsl_vector_subvector (this->estimator_ekf->mu, 12, 6);

        gsl_matrix_view tmp = gsl_matrix_submatrix (this->estimator_ekf->Sigma, 0, 0, 6, 6);
        gslu_matrix_set_submatrix (Sig_positional, 0, 0, &tmp.matrix);
        tmp = gsl_matrix_submatrix (this->estimator_ekf->Sigma, 0, 12, 6, 6);
        gslu_matrix_set_submatrix (Sig_positional, 0, 6, &tmp.matrix);
        tmp = gsl_matrix_submatrix (this->estimator_ekf->Sigma, 12, 0, 6, 6);
        gslu_matrix_set_submatrix (Sig_positional, 6, 0, &tmp.matrix);
        tmp = gsl_matrix_submatrix (this->estimator_ekf->Sigma, 12, 12, 6, 6);
        gslu_matrix_set_submatrix (Sig_positional, 6, 6, &tmp.matrix);

        ssc_tail2tail_gsl (x_vT, J, &x_lv.vector, &x_lT.vector);
        gslu_blas_mmmT (Sig_vT, J, Sig_positional, J, NULL);

        double det = gsl_matrix_get (Sig_vT, 0, 0) * gsl_matrix_get (Sig_vT, 1, 1)
            - gsl_matrix_get (Sig_vT, 0, 1) * gsl_matrix_get (Sig_vT, 1, 0);
        det = pow (det, 0.25);
        printf ("~~~~~~~~~~~~~~~~~target_det: %4.4f\n", det);

        //if (det > 0.5) this->remove_target ();

        gsl_vector_free (x_vT);
        gsl_matrix_free (J);
        gsl_matrix_free (Sig_vT);
        gsl_matrix_free (Sig_positional);
    }




    //if ( target_viewed && (gsl_matrix_get (this->estimator_ekf->Sigma, 12, 12) + gsl_matrix_get (this->estimator_ekf->Sigma, 13, 13)) > 4)
        //remove_target ();
}

//----------------------------------------------------------------------------------
// Set control input to be used in prediction
//----------------------------------------------------------------------------------
void
navigator::set_control_input (perllcm_ardrone_drive_t *new_control)
{
    if (!new_control) {
        if (this->control_input) perllcm_ardrone_drive_t_destroy (this->control_input);
        this->control_input = NULL;
        return;
    }

    if (!this->control_input)
        this->control_input = perllcm_ardrone_drive_t_copy (new_control);

    memcpy (control_input, new_control, sizeof( perllcm_ardrone_drive_t));

    /* add observation to queue if target not yet viewed */
    if (!this->target_viewed && this->track_target) {
        this->replay_observations.push_back ((observation*) new control_obs (perllcm_ardrone_drive_t_copy (new_control)));
        this->replay_internal_snapshot.push_back (new snapshot (this->prev_utime, this->flying, this->mocap_viewed, this->state_viewed, this->control_input));
    }
}

//----------------------------------------------------------------------------------
// EKF interface for adding simple predictions
//----------------------------------------------------------------------------------
void
navigator::add_time_observation (int64_t utime)
{
    printf ("nav::add_time\n");
    /* need quad initialized before we can add sync observations */
    if (!this->quad_viewed || !this->track_target) return;

    /* add observation to queue if target not yet viewed */
    if (!this->target_viewed && this->track_target) {
        this->replay_observations.push_back ((observation*) new time_obs (utime));
        this->replay_internal_snapshot.push_back (new snapshot (this->prev_utime, this->flying, this->mocap_viewed, this->state_viewed, this->control_input));
    }

    /* run a state prediction */
    this->predict (utime, false);
}

//----------------------------------------------------------------------------------
// EKF interface for adding sync message observation
//----------------------------------------------------------------------------------
void
navigator::add_sync_observation (bot_core_image_sync_t *obs)
{
    printf ("nav::add_sync\n");
    /* need quad initialized before we can add sync observations */
    if (!this->quad_viewed || !this->track_target) return;

    /* add observation to queue if target not yet viewed */
    if (!this->target_viewed && this->track_target) {
        this->replay_observations.push_back ((observation*) new sync_obs (bot_core_image_sync_t_copy (obs)));
        this->replay_internal_snapshot.push_back (new snapshot (this->prev_utime, this->flying, this->mocap_viewed, this->state_viewed, this->control_input));
    }

    /* predict and drop a delayed state */
    this->predict (obs->utime, true);

}

//----------------------------------------------------------------------------------
// EKF interface for adding april observations
//----------------------------------------------------------------------------------
void
navigator::add_april_observation (perllcm_tag_detection_collection_t *obs)
{
    printf ("nav::add_april\n");
    /* need quad initialized before we can add april observations */
    if (!this->quad_viewed || !this->track_target) return;

    /* run state prediction */
    printf ("predict\n");
    this->predict (obs->utime, false);

    /* find correspong delayed state */
    std::deque<int64_t>::iterator ds_it = std::find (this->delayed_state_utimes.begin(), this->delayed_state_utimes.end(), obs->utime);

    printf ("Q: %ld, List: ", obs->utime);
    for (std::deque<int64_t>::iterator asd = delayed_state_utimes.begin(); asd != delayed_state_utimes.end(); asd++)
        printf ("%ld, ", *asd);
    printf ("\n");

    if (ds_it == this->delayed_state_utimes.end()) {
        printf ("FLAG: Unmatched image!\n");
        return;
    }

    /* if first april obs, need to add it to state vector first */
    printf ("check target viewed\n");
    if (!target_viewed) {
        /* find most accurate target observation */
        double best = 0; int idx = -1;
        for (int i=0; i<obs->ndetections; ++i) {
            double det = obs->detections[i].pose.xyzrph_cov[0] * 
                            obs->detections[i].pose.xyzrph_cov[7] -
                            obs->detections[i].pose.xyzrph_cov[1] * 
                            obs->detections[i].pose.xyzrph_cov[1];
            if (x_tag_to_target.find(obs->detections[i].id) != x_tag_to_target.end() &&
                    (idx == -1 || det < best)) {
                best = det;
                idx = i;
            }
        }

        if (idx == -1) return;

        gsl_vector *v = gsl_vector_calloc (6);
        for (int i=0; i<6; i++)
            gsl_vector_set (v, i, obs->detections[idx].pose.xyzrph[i]);

        this->x_active_tag_to_target = x_tag_to_target[obs->detections[idx].id];

        /* instantiate target into state vector */
        this->add_target (v, this->delayed_state_utimes.end () - ds_it);
        gsl_vector_free (v);
        return;
    }




    // if made it this far, we can do state correction

    /* pass in pointer to current navigator structure */
    this->meas_april_tag->user = (void *) this;

    /* process each observation independently */
    printf ("process indep\n");
    for (int d=0; d<obs->ndetections; ++d) {
        gsl_vector *x_s1t = gsl_vector_calloc (6);
        gsl_vector *x_s2t = gsl_vector_calloc (6);
        gsl_vector *x_lv1 = gsl_vector_calloc (6);
        gsl_vector *x_lv2 = gsl_vector_calloc (6);
        gsl_vector *x_ls1 = gsl_vector_calloc (6);
        gsl_vector *x_ls2 = gsl_vector_calloc (6);
        gsl_vector *x_s2s1 = gsl_vector_calloc (6);

        int delayed_st_start = (this->delayed_state_utimes.end () - ds_it) * 24;

        for (int i=0; i<6; ++i) {
            // local frame to delayed vehicle
            gsl_vector_set (x_lv1, i, gsl_vector_get (this->estimator_ekf->mu, i+delayed_st_start));

            // local frame to latest vehicle
            gsl_vector_set (x_lv2, i, gsl_vector_get (this->estimator_ekf->mu, i));

            // delayed sensor to tag
            gsl_vector_set (x_s1t, i, obs->detections[d].pose.xyzrph[i]);
        }

        // add on camera extrinsics to both states
        ssc_head2tail_gsl (x_ls1, NULL, x_lv1, this->x_quad_to_cam);
        ssc_head2tail_gsl (x_ls2, NULL, x_lv2, this->x_quad_to_cam);

        // pose from current to delayed state
        ssc_tail2tail_gsl (x_s2s1, NULL, x_ls2, x_ls1);
        // pose from current to tag
        ssc_head2tail_gsl (x_s2t, NULL, x_s2s1, x_s1t);

        /* copy observation */
        // actual delayed state methodology
        //for (int i=0; i<6; i++)
            //gsl_vector_set (this->meas_april_tag->z, i, obs->detections[d].pose.xyzrph[i]);

        // push observation into current frame and act on current
        for (int i=0; i<6; i++)
            gsl_vector_set (this->meas_april_tag->z, i, gsl_vector_get (x_s2t, i));


        // make spoofed altitude measurement from tag
        double TAG_HEIGHT = 0.5;
        gsl_vector *x_gv            = gsl_vector_calloc (6);
        gsl_vector *x_gt            = gsl_vector_calloc (6);
        gsl_vector *x_sv            = gsl_vector_calloc (6);
        gsl_vector *x_tv            = gsl_vector_calloc (6);
        gsl_vector_set (x_gt, 2, -TAG_HEIGHT);

        ssc_inverse_gsl (x_sv, NULL, this->x_quad_to_cam);
        ssc_tail2tail_gsl (x_tv, NULL, x_s2t, x_sv);
        ssc_head2tail_gsl (x_gv, NULL, x_gt, x_tv);

        gsl_vector_set (this->meas_april_tag->z, 6, gsl_vector_get (x_gv, 2));

        // copy observation covariance from tag detection message
        //for (int i=0; i<6; i++)
            //for (int j=0; j<6; j++)
                //gsl_matrix_set (this->meas_april_tag->R, i, j, obs->detections[d].pose.xyzrph_cov[i+j*6]);

        /* using predetermined april noise instead.. */
        gsl_matrix_memcpy (this->meas_april_tag->R, this->obs_noise_april);


        /* find and assign tag to target mapping for use in OM */
        std::map<int, gsl_vector*>::iterator it = this->x_tag_to_target.find(obs->detections[d].id);
        if (it == this->x_tag_to_target.end()) {
            printf ("ERROR: Unknown tag type!\n");
            continue;
        }

        this->x_active_tag_to_target = (*it).second;

        /* turn crank on kalman filter, acting on delayed state idx */
        est_correct_ds (this->estimator_ekf, this->meas_april_tag, 0);
        //est_correct_ds (this->estimator_ekf, this->meas_april_tag, this->delayed_state_utimes.end () - ds_it);
        
        gsl_vector_free (x_s1t);
        gsl_vector_free (x_s2t);
        gsl_vector_free (x_lv1);
        gsl_vector_free (x_lv2);
        gsl_vector_free (x_ls1);
        gsl_vector_free (x_ls2);
        gsl_vector_free (x_s2s1);

        gsl_vector_free (x_gv);
        gsl_vector_free (x_gt);
        gsl_vector_free (x_sv);
        gsl_vector_free (x_tv);
    }

    printf ("remove up to: %ld\n", obs->utime);
    for(std::deque<int64_t>::iterator it=delayed_state_utimes.begin(); it != delayed_state_utimes.end(); ++it) printf ("%ld \n", (*it));
    /* marginalize out all older delayed states and this updated one */
    while (!this->delayed_state_utimes.empty() && this->delayed_state_utimes.front () <= obs->utime) {
        est_marginalize_ds (this->estimator_ekf, this->delayed_state_utimes.size ());
        delayed_state_utimes.pop_front ();
    }
    for(std::deque<int64_t>::iterator it=delayed_state_utimes.begin(); it != delayed_state_utimes.end(); ++it) printf ("%ld \n", (*it));


}

//----------------------------------------------------------------------------------
// EKF interface for adding quadrotor mocap observations
//----------------------------------------------------------------------------------
void
navigator::add_mocap_observation (senlcm_mocap_t *obs)
{
    printf ("nav::add_mocap\n");
    /* check if we need to initialize filter */
    if (!this->quad_viewed) {
        gsl_vector *start = gsl_vector_calloc (6);
        for (int i=0; i<6; i++)
            gsl_vector_set (start, i, obs->xyzrph[i]);
        this->initialize_filter (start);
        gsl_vector_free (start);
    }
    
    /* check if this is first mocap observation -> store it for reference point */
    if (!this->mocap_viewed) {
        /* copy over initial observation */
        if (this->initial_mocap_obs == NULL)
            this->initial_mocap_obs = gsl_vector_calloc (6);
        for (int i=0; i<6; i++)
            gsl_vector_set (this->initial_mocap_obs, i, obs->xyzrph[i]);

        /* capture state of first observation */
        if (this->initial_mocap_mu == NULL)
            this->initial_mocap_mu = gsl_vector_calloc (6);
        for (int i=0; i<6; i++)
            gsl_vector_set (this->initial_mocap_mu, i, gsl_vector_get (this->estimator_ekf->mu, i));
        if (this->initial_mocap_Sigma == NULL)
            this->initial_mocap_Sigma = gsl_matrix_calloc (6,6);
        for (int i=0; i<6; i++)
            for (int j=0; j<6; j++)
                gsl_matrix_set (this->initial_mocap_Sigma, i, j, gsl_matrix_get (this->estimator_ekf->Sigma, i, j));

        this->mocap_viewed = true;
        return;
    }

    /* add observation to queue if target not yet viewed */
    if (!this->target_viewed && this->track_target) {
        this->replay_observations.push_back ((observation*) new mocap_obs (senlcm_mocap_t_copy (obs)));
        this->replay_internal_snapshot.push_back (new snapshot (this->prev_utime, this->flying, this->mocap_viewed, this->state_viewed, this->control_input));
    }

// FOR DEBUG ONLY -------------------------------
    if (false) {
        lcm_t *lcm = lcm_create (NULL);
        perllcm_position_t state;
        state.utime = obs->utime;
        state.xyzrph[0] = obs->xyzrph[0];
        state.xyzrph[1] = obs->xyzrph[1];
        state.xyzrph[2] = obs->xyzrph[2];
        state.xyzrph[3] = obs->xyzrph[3];
        state.xyzrph[4] = obs->xyzrph[4];
        state.xyzrph[5] = obs->xyzrph[5];

        perllcm_position_t_publish (lcm, "DEBUG_MOCAP", &state);
        lcm_destroy (lcm);
        return;
    }
// FOR DEBUG ONLY -------------------------------

    /* run state prediction */
    this->predict (obs->utime, false);

    /* transform & format observation */
    gsl_vector *x_mv         = gsl_vector_calloc (6);
    gsl_vector *x_v0v        = gsl_vector_calloc (6);
    gsl_matrix *J_A          = gsl_matrix_calloc (6, 12);
    gsl_matrix *J_B          = gsl_matrix_calloc (6, 12);
    gsl_matrix *double_noise = gsl_matrix_calloc (12, 12);
    gsl_matrix *sig_v0v      = gsl_matrix_calloc (6, 6);


    for (int i=0; i<6; i++)
        gsl_vector_set (x_mv, i, obs->xyzrph[i]);

    ssc_tail2tail_gsl (x_v0v, J_A, this->initial_mocap_obs, x_mv);
    ssc_head2tail_gsl (meas_quad_mocap->z, J_B, this->initial_mocap_mu, x_v0v);

    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            gsl_matrix_set (double_noise, i, j, gsl_matrix_get (this->obs_noise_mocap, i, j));
            gsl_matrix_set (double_noise, i+6, j+6, gsl_matrix_get (this->obs_noise_mocap, i, j));
        }
    }

    gslu_blas_mmmT (sig_v0v, J_A, double_noise, J_A, NULL);

    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            gsl_matrix_set (double_noise, i, j, gsl_matrix_get (this->initial_mocap_Sigma, i, j));
            gsl_matrix_set (double_noise, i+6, j+6, gsl_matrix_get (sig_v0v, i, j));
        }
    }

    gslu_blas_mmmT (this->meas_quad_mocap->R, J_B, double_noise, J_B, NULL);

    gsl_vector_free (x_mv);
    gsl_vector_free (x_v0v);
    gsl_matrix_free (J_A);
    gsl_matrix_free (J_B);
    gsl_matrix_free (double_noise);
    gsl_matrix_free (sig_v0v);

    /* turn crank on kalman filter, acting on most recent delayed state (0) */
    est_correct_ds (this->estimator_ekf, this->meas_quad_mocap, 0);
}

//----------------------------------------------------------------------------------
// EKF interface for adding quadrotor state data observations
//----------------------------------------------------------------------------------
void
navigator::add_state_observation (perllcm_ardrone_state_t *obs)
{
    printf ("nav::add_state\n");
    this->flying = obs->flying;

    /* fix bad altitude observations */
    if (obs->altitude < 0 || obs->altitude > 25 || !obs->flying) obs->altitude = 0;

    /* check if we need to initialize filter */
    if (!this->quad_viewed) {
        gsl_vector *start = gsl_vector_calloc (6);
        gsl_vector_set (start, 2, -obs->altitude);
        gsl_vector_set (start, 3, obs->roll);
        gsl_vector_set (start, 4, obs->pitch);
        this->initialize_filter (start);
        gsl_vector_free (start);
    }
    
    /* check if this is first state observation -> store it for reference point */
    if (!this->state_viewed) {
        /* copy over initial observation */
        if (this->initial_state_obs == NULL)
            this->initial_state_obs = gsl_vector_calloc (6);

        gsl_vector_set (this->initial_state_obs, 0, obs->altitude);
        gsl_vector_set (this->initial_state_obs, 1, obs->roll);
        gsl_vector_set (this->initial_state_obs, 2, obs->pitch);
        gsl_vector_set (this->initial_state_obs, 3, obs->yaw);
        gsl_vector_set (this->initial_state_obs, 4, obs->vx);
        gsl_vector_set (this->initial_state_obs, 5, obs->vy);

        /* capture state of first observation */
        if (this->initial_state_mu == NULL)
            this->initial_state_mu = gsl_vector_calloc (6);
        for (int i=2; i<6; i++) /* only capture z->heading */
            gsl_vector_set (this->initial_state_mu, i, gsl_vector_get (this->estimator_ekf->mu, i));

        if (this->initial_state_Sigma == NULL)
            this->initial_state_Sigma = gsl_matrix_calloc (6,6);
        for (int i=2; i<6; i++)
            for (int j=2; j<6; j++)
                gsl_matrix_set (this->initial_state_Sigma, i, j, gsl_matrix_get (this->estimator_ekf->Sigma, i, j));

        this->state_viewed = true;
        return;
    }

    /* add observation to queue if target not yet viewed */
    if (!this->target_viewed && this->track_target) {
        this->replay_observations.push_back ((observation*) new state_obs (perllcm_ardrone_state_t_copy (obs)));
        this->replay_internal_snapshot.push_back (new snapshot (this->prev_utime, this->flying, this->mocap_viewed, this->state_viewed, this->control_input));
    }

// FOR DEBUG ONLY -------------------------------
    if (false) {
        lcm_t *lcm = lcm_create (NULL);
        perllcm_position_t state;
        state.utime = obs->utime;
        state.xyzrph[2] = -obs->altitude;
        state.xyzrph[3] = obs->roll;
        state.xyzrph[4] = obs->pitch;
        state.xyzrph[5] = obs->yaw;

        perllcm_position_t_publish (lcm, "DEBUG_STATE", &state);
        lcm_destroy (lcm);
    }
// FOR DEBUG ONLY -------------------------------

    /* run state prediction */
    this->predict (obs->utime, false);

    /* format observation */
    gsl_vector *x_sv            = gsl_vector_calloc (6);
    gsl_vector *x_sv0           = gsl_vector_calloc (6);
    gsl_vector *x_lv            = gsl_vector_calloc (6);
    gsl_vector *x_v0v           = gsl_vector_calloc (6);
    gsl_matrix *J_A             = gsl_matrix_calloc (6, 12);
    gsl_matrix *J_B             = gsl_matrix_calloc (6, 12);
    gsl_matrix *double_noise    = gsl_matrix_calloc (12,12);
    gsl_matrix *sig_v0v         = gsl_matrix_calloc (6,6);
    gsl_matrix *noise           = gsl_matrix_calloc (6, 6);

    gsl_vector_set (x_sv0, 2, -gsl_vector_get (this->initial_state_obs, 0));
    for (int i=3; i<6; i++)
        gsl_vector_set (x_sv0, i, gsl_vector_get (this->initial_state_obs, i-2));

    gsl_vector_set (x_sv, 2, -obs->altitude);
    gsl_vector_set (x_sv, 3, obs->roll);
    gsl_vector_set (x_sv, 4, obs->pitch);
    gsl_vector_set (x_sv, 5, obs->yaw);

    ssc_tail2tail_gsl (x_v0v, J_A, x_sv0, x_sv);
    ssc_head2tail_gsl (x_lv, J_B, this->initial_state_mu, x_v0v);

    for (int i=2; i<6; i++) {
        for (int j=2; j<6; j++) {
            gsl_matrix_set (double_noise, i, j, gsl_matrix_get (this->obs_noise_state, i-2, j-2));
            gsl_matrix_set (double_noise, i+6, j+6, gsl_matrix_get (this->obs_noise_state, i-2, j-2));
        }
   }

    gslu_blas_mmmT (sig_v0v, J_A, double_noise, J_A, NULL);

    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++) {
            gsl_matrix_set (double_noise, i, j, gsl_matrix_get (this->initial_state_Sigma, i, j));
            gsl_matrix_set (double_noise, i+6, j+6, gsl_matrix_get (sig_v0v, i, j));
        }
    }

    gslu_blas_mmmT (noise, J_B, double_noise, J_B, NULL);

    for (int i=2; i<6; i++) {
        for (int j=2; j<6; j++) {
            //gsl_matrix_set (this->meas_quad_state->R, i-2, j-2, gsl_matrix_get (noise, i, j));
        }
    }

    //gsl_matrix_set (this->meas_quad_state->R, 0, 0, gsl_matrix_get (this->obs_noise_state, 4, 4));
    //gsl_matrix_set (this->meas_quad_state->R, 1, 1, gsl_matrix_get (this->obs_noise_state, 5, 5));

    gsl_vector_set (this->meas_quad_state->z, 0, gsl_vector_get (x_lv, 2)); // z
    gsl_vector_set (this->meas_quad_state->z, 1, obs->roll); // roll
    gsl_vector_set (this->meas_quad_state->z, 2, obs->pitch); // pitch
    gsl_vector_set (this->meas_quad_state->z, 3, gsl_vector_get (x_lv, 5)); // yaw
    gsl_vector_set (this->meas_quad_state->z, 3, obs->yaw - gsl_vector_get (this->initial_state_obs, 3) + gsl_vector_get (this->initial_state_mu, 5)); // yaw
    gsl_vector_set (this->meas_quad_state->z, 4, obs->vx);
    gsl_vector_set (this->meas_quad_state->z, 5, obs->vy);
    this->meas_quad_state->user = (void*) this;

    bool out_of_range = !(target_viewed && sqrt (
                pow (gsl_vector_get (this->estimator_ekf->mu, 0) - gsl_vector_get (this->estimator_ekf->mu, 12), 2) +
                pow (gsl_vector_get (this->estimator_ekf->mu, 1) - gsl_vector_get (this->estimator_ekf->mu, 13), 2)) < 0.5);

    // increase altimeter noise within range of target
    if (!out_of_range)
        gsl_matrix_set (this->meas_quad_state->R, 0, 0, 25*gsl_matrix_get (this->obs_noise_state,0,0));
   
    /* cleanup */
    gsl_vector_free (x_sv);
    gsl_vector_free (x_sv0);
    gsl_vector_free (x_lv);
    gsl_vector_free (x_v0v);
    gsl_matrix_free (J_A);
    gsl_matrix_free (J_B);
    gsl_matrix_free (double_noise);
    gsl_matrix_free (sig_v0v);
    gsl_matrix_free (noise);

    /* turn crank on kalman filter, acting on most recent delayed state (0) */
    est_correct_ds (this->estimator_ekf, this->meas_quad_state, 0);
}

//----------------------------------------------------------------------------------
// Get Quadrotor state from within estimator's mu/Sigma
//----------------------------------------------------------------------------------
perllcm_position_t *
navigator::get_quadrotor_state ()
{
    if (!quad_viewed) return NULL;

    perllcm_position_t *p = new perllcm_position_t;
    p->utime = prev_utime;

    for (int i=0; i<6; i++)
        p->xyzrph[i] = gsl_vector_get (this->estimator_ekf->mu, i);
    for (int i=0; i<6; i++)
        for (int j=0; j<6; j++)
            p->xyzrph_cov[i*6+j] = gsl_matrix_get (this->estimator_ekf->Sigma, i, j);
    for (int i=0; i<6; i++)
        p->xyzrph_dot[i] = gsl_vector_get (this->estimator_ekf->mu, 6+i);

    return p;
}

//----------------------------------------------------------------------------------
// Get Target state from within estiamtor's mu/Sigma
//----------------------------------------------------------------------------------
perllcm_position_t *
navigator::get_target_state ()
{
    if (!target_viewed) return NULL;

    perllcm_position_t *p = new perllcm_position_t;
    p->utime = prev_utime;

    for (int i=0; i<6; i++)
        p->xyzrph[i] = gsl_vector_get (this->estimator_ekf->mu, i+12);
    for (int i=0; i<6; i++)
        for (int j=0; j<6; j++)
            p->xyzrph_cov[i*6+j] = gsl_matrix_get (this->estimator_ekf->Sigma, i+12, j+12);
    for (int i=0; i<6; i++)
        p->xyzrph_dot[i] = gsl_vector_get (this->estimator_ekf->mu, 18+i);

    return p;
}

//----------------------------------------------------------------------------------
// Initializes the EKF for quadrotor only
//----------------------------------------------------------------------------------
void
navigator::initialize_filter (gsl_vector *init_pose)
{
    int state_length = 12;

    /* initial state is all zero, unless init_pose present */
    gsl_vector *ini_state = gsl_vector_calloc (state_length);

    /* assign initial pose, if NULL, leave initial as all zeros */
    if (init_pose != NULL)
        gslu_vector_set_subvector (ini_state, 0, init_pose);

    /* initial covariance diagonal 0.0001 */
    gsl_matrix *ini_cov = gsl_matrix_calloc (state_length, state_length);
    gsl_matrix_set_identity (ini_cov);
    gsl_matrix_scale (ini_cov, 0.0001);

    /* initialize ekf estimator */
    this->estimator_ekf = est_init_ekf (state_length, ini_state, ini_cov);

    /* initialize delayed state elements */
    est_init_ds (this->estimator_ekf, -1, state_length);

    this->quad_viewed = true;
}

//----------------------------------------------------------------------------------
// Adds the target to the state vector
//----------------------------------------------------------------------------------
void
navigator::add_target (gsl_vector *x_st, int ds_ind)
{
    printf ("adding target.. \n");
    printf ("~~ds: %d, mds: %ld, ds: %d\n", this->estimator_ekf->delayed_states_cnt, delayed_state_utimes.size (), ds_ind);

    /* initialize target state */
    est_estimator_t *old_estimator = this->estimator_ekf;
    int state_length = 24;
    int ds_start = old_estimator->delayed_state_len * ds_ind;

    gsl_vector *ini_state = gsl_vector_calloc (state_length);
    gsl_matrix *ini_cov = gsl_matrix_calloc (state_length, state_length);
    gsl_matrix_set_identity (ini_cov);
    gsl_matrix_scale (ini_cov, 0.0001);

    /* copy over previous state */
    gsl_vector_view ini_state_v = gsl_vector_subvector (old_estimator->mu, ds_start, 12);
    gslu_vector_set_subvector (ini_state, 0, &ini_state_v.vector);
    gsl_matrix_view ini_cov_v = gsl_matrix_submatrix (old_estimator->Sigma, ds_start, ds_start, 12, 12);
    gslu_matrix_set_submatrix (ini_cov, 0, 0, &ini_cov_v.matrix);

    /* instantiate new "landmark"/target observation */
    gsl_matrix *J_A         = gsl_matrix_calloc (6, 12);
    gsl_matrix *J_B         = gsl_matrix_calloc (6, 12);
    gsl_matrix *Sig_x_lt    = gsl_matrix_calloc (6, 6);
    gsl_matrix *Sig_x_lv    = gsl_matrix_calloc (6, 6);
    gsl_vector *x_ls        = gsl_vector_calloc (6);
    gsl_vector *x_ltag      = gsl_vector_calloc (6);
    gsl_vector *x_ltarget   = gsl_vector_calloc (6);
    gsl_vector *x_lv        = gsl_vector_calloc (6);
    gsl_matrix *B           = gsl_matrix_calloc (6, 6);
    gsl_matrix *J_C         = gsl_matrix_calloc (6, 12);
    gsl_matrix *tmp         = gsl_matrix_calloc (6, 6);
    gsl_matrix *J_c1b1a1    = gsl_matrix_calloc (6, 6);
    gsl_matrix *J_c1b2      = gsl_matrix_calloc (6, 6);


    gslu_vector_get_subvector (x_lv, ini_state, 0, 6);
    gsl_matrix_view sig_x_lv_v = gsl_matrix_submatrix (ini_cov, 0, 0, 6, 6);
    gsl_matrix_memcpy (Sig_x_lv, &sig_x_lv_v.matrix);
    
    ssc_head2tail_gsl (x_ls, J_A, x_lv, x_quad_to_cam);
    ssc_head2tail_gsl (x_ltag, J_B, x_ls, x_st);
    ssc_head2tail_gsl (x_ltarget, J_C, x_ltag, get_tag_to_target());

    gsl_matrix_view J_A1 = gsl_matrix_submatrix (J_A, 0, 0, 6, 6);
    gsl_matrix_view J_B1 = gsl_matrix_submatrix (J_B, 0, 0, 6, 6);
    gsl_matrix_view J_B2 = gsl_matrix_submatrix (J_B, 0, 6, 6, 6);
    gsl_matrix_view J_C1 = gsl_matrix_submatrix (J_C, 0, 0, 6, 6);

    /* augment covariance as: 
            [Sig_x_lv,         B'
                    B,   Sig_x_lt]
    */
    gsl_matrix_view tag_R = gsl_matrix_submatrix (this->meas_april_tag->R, 0, 0, 6, 6);

    gslu_blas_mmm (J_c1b1a1, &J_C1.matrix, &J_B1.matrix, &J_A1.matrix, NULL);
    gslu_blas_mm (J_c1b2, &J_C1.matrix, &J_B2.matrix);
    gslu_blas_mm (B, J_c1b1a1, Sig_x_lv);
    gslu_blas_mmT (Sig_x_lt, B, J_c1b1a1);

    gslu_blas_mmmT (tmp, J_c1b2, &tag_R.matrix, J_c1b2, NULL);
    gsl_matrix_add (Sig_x_lt, tmp);

    /* set target terms */
    gslu_vector_set_subvector (ini_state, 12, x_ltarget);
    gslu_matrix_set_submatrix (ini_cov, 12, 12, Sig_x_lt);
    gslu_matrix_set_submatrix (ini_cov, 12, 0, B);
    gsl_matrix_transpose (B);
    gslu_matrix_set_submatrix (ini_cov, 0, 12, B);

    /* cleanup */
    gsl_matrix_free (J_A);
    gsl_matrix_free (J_B);
    gsl_matrix_free (J_C);
    gsl_matrix_free (Sig_x_lt);
    gsl_matrix_free (Sig_x_lv);
    gsl_vector_free (x_ls);
    gsl_vector_free (x_ltag);
    gsl_vector_free (x_ltarget);
    gsl_vector_free (x_lv);
    gsl_matrix_free (B);
    gsl_matrix_free (tmp);
    gsl_matrix_free (J_c1b1a1);
    gsl_matrix_free (J_c1b2);

    /* reset Q --> add new terms for target */
    gsl_matrix_free (this->estimator_pred->Q);
    this->estimator_pred->Q = gsl_matrix_calloc (state_length, state_length);
    gsl_matrix_set (this->estimator_pred->Q, 6, 6, 1); 
    gsl_matrix_set (this->estimator_pred->Q, 7, 7, 1); 
    gsl_matrix_set (this->estimator_pred->Q, 8, 8, 1); 
    gsl_matrix_set (this->estimator_pred->Q, 9, 9, 1); 
    gsl_matrix_set (this->estimator_pred->Q, 10, 10, 1); 
    gsl_matrix_set (this->estimator_pred->Q, 11, 11, 1); 
    gsl_matrix_set (this->estimator_pred->Q, 18, 18, 1); 
    gsl_matrix_set (this->estimator_pred->Q, 19, 19, 1); 
    gsl_matrix_set (this->estimator_pred->Q, 14, 14, 1); // positional noise instead
    gsl_matrix_set (this->estimator_pred->Q, 15, 15, 1); // positional noise instead
    gsl_matrix_set (this->estimator_pred->Q, 16, 16, 1); // positional noise instead
    gsl_matrix_set (this->estimator_pred->Q, 17, 17, 1); // positional noise instead

    /* re-init estimator */
    est_free (old_estimator);
    this->estimator_ekf = est_init_ekf (state_length, ini_state, ini_cov);
    est_init_ds (this->estimator_ekf, -1, state_length);

    this->target_viewed = true;

    /* replay messages since this delayed state */
    this->replay (this->delayed_state_utimes.at (this->delayed_state_utimes.size () - ds_ind));

    printf ("done adding target..\n");
}

//----------------------------------------------------------------------------------
// Replays all observations in replay_observation
//----------------------------------------------------------------------------------
void
navigator::replay (int64_t ds_utime)
{
    printf ("rebuilding delayed states through replay..\n");
    /* find start of replay and delete everything up to start of replay */
    std::list<observation*>::iterator obs_it = this->replay_observations.begin ();
    while (obs_it != this->replay_observations.end ()) {
        if ((*obs_it)->get_type () == SYNC_OBS && ((sync_obs*)(*obs_it))->obs->utime == ds_utime)
            break;

        /* not found yet, delete it */
        switch ((*obs_it)->get_type ()) {
            case APRIL_OBS:
                delete (april_obs*) (*obs_it);
                break;
            case MOCAP_OBS:
                delete (mocap_obs*) (*obs_it);
                break;
            case STATE_OBS:
                delete (state_obs*) (*obs_it);
                break;
            case SYNC_OBS:
                delete (sync_obs*) (*obs_it);
                break;
            case TIME_OBS:
                delete (time_obs*) (*obs_it);
                break;
            case CONTROL_OBS:
                delete (control_obs*) (*obs_it);
                break;
            case BASE_OBS:
                delete (*obs_it);
                break;
        }

        obs_it = replay_observations.erase (obs_it);
        delete replay_internal_snapshot.front ();
        replay_internal_snapshot.pop_front ();
    }

    /* replay! */
    this->prev_utime = replay_internal_snapshot.front()->prev_utime;
    this->flying = replay_internal_snapshot.front()->flying;
    this->mocap_viewed = replay_internal_snapshot.front()->mocap_viewed;
    this->state_viewed = replay_internal_snapshot.front()->state_viewed;
    this->set_control_input (replay_internal_snapshot.front()->control_input);
    delayed_state_utimes.clear ();

    while (!replay_observations.empty ()) {
        switch ((*obs_it)->get_type ()) {
            case APRIL_OBS:
                this->add_april_observation (((april_obs*)(*obs_it))->obs);
                delete (april_obs*) (*obs_it);
                break;
            case MOCAP_OBS:
                this->add_mocap_observation (((mocap_obs*)(*obs_it))->obs);
                delete (mocap_obs*) (*obs_it);
                break;
            case STATE_OBS:
                this->add_state_observation (((state_obs*)(*obs_it))->obs);
                delete (state_obs*) (*obs_it);
                break;
            case SYNC_OBS:
                this->add_sync_observation (((sync_obs*)(*obs_it))->obs);
                delete (sync_obs*) (*obs_it);
                break;
            case TIME_OBS:
                this->add_time_observation (((time_obs*)(*obs_it))->obs);
                delete (time_obs*) (*obs_it);
                break;
            case CONTROL_OBS:
                this->set_control_input (((control_obs*)(*obs_it))->obs);
                delete (control_obs*) (*obs_it);
                break;
            case BASE_OBS:
                delete (*obs_it);
                break;
        }

        obs_it = replay_observations.erase (obs_it);
        delete replay_internal_snapshot.front ();
        replay_internal_snapshot.pop_front ();
    }

}

//----------------------------------------------------------------------------------
// Removes target from the state vector
//----------------------------------------------------------------------------------
void
navigator::remove_target ()
{
    printf ("removing target.. \n");

    /* initialize target state */
    est_estimator_t *old_estimator = this->estimator_ekf;
    int state_length = 12;
    gsl_vector *ini_state = gsl_vector_calloc (state_length);
    gsl_matrix *ini_cov = gsl_matrix_calloc (state_length, state_length);
    gsl_matrix_set_identity (ini_cov);
    gsl_matrix_scale (ini_cov, 0.0001);

    /* copy over previous state */
    for (int i=0; i<12; i++)
        gsl_vector_set (ini_state, i, gsl_vector_get (old_estimator->mu, i));
    for (int i=0; i<12; i++)
        for (int j=0; j<12; j++)
            gsl_matrix_set (ini_cov, i, j, gsl_matrix_get (old_estimator->Sigma, i, j));

    /* reset Q --> add new terms for target */
    gsl_matrix_free (this->estimator_pred->Q);
    this->estimator_pred->Q = gsl_matrix_calloc (state_length, state_length);
    gsl_matrix_set (this->estimator_pred->Q, 6, 6, 1); 
    gsl_matrix_set (this->estimator_pred->Q, 7, 7, 1); 
    gsl_matrix_set (this->estimator_pred->Q, 8, 8, 1); 
    gsl_matrix_set (this->estimator_pred->Q, 9, 9, 1); 
    gsl_matrix_set (this->estimator_pred->Q, 10, 10, 1); 
    gsl_matrix_set (this->estimator_pred->Q, 11, 11, 1); 

    /* re-init estimator */
    est_free (old_estimator);
    this->estimator_ekf = est_init_ekf (state_length, ini_state, ini_cov);

    this->target_viewed = false;
}

// END navigator class


//----------------------------------------------------------------------------------
// Process model function
//----------------------------------------------------------------------------------

/*************************************************************************************
 * This process model is a simple constant velocity model with white Gaussian noise
 * induced on the velocity terms. Additionally, there is additive white Gaussian noise
 * on the Z, roll, pitch, yaw components of the target's model.
 *************************************************************************************/
void
pm_ekf (const est_estimator_t *state, const gsl_vector *u, const double dt,
        gsl_vector *mu_bar, gsl_matrix *F, gsl_matrix *Q, void *user)
{
    /* must fill mu_bar and F */
    navigator *nav = (navigator *) user;

    gsl_matrix_set_identity (F);

    /* only propogate *quadrotor* state while flying */
    if (nav->is_flying()) {
        gsl_matrix_set (F, 0, 6, dt);
        gsl_matrix_set (F, 1, 7, dt);
        gsl_matrix_set (F, 2, 8, dt);
        gsl_matrix_set (F, 3, 9, dt);
        gsl_matrix_set (F, 4, 10, dt);
        gsl_matrix_set (F, 5, 11, dt);
    }

    /* ONLY use constant velocity model for target x/y */
    if (state->delayed_state_len > 12) {
        gsl_matrix_set (F, 12, 18, dt*2); // changed
        gsl_matrix_set (F, 13, 19, dt*2); // changed
    }

    /* propogate state using constant velocity terms set above */
    gsl_vector_view mu_v = gsl_vector_subvector (state->mu, 0, state->delayed_state_len);
    gslu_blas_mv (mu_bar, F, &mu_v.vector);

    /* minimize angles */
    gsl_vector_set (mu_bar, 3, gslu_math_minimized_angle (gsl_vector_get (mu_bar, 3)));
    gsl_vector_set (mu_bar, 4, gslu_math_minimized_angle (gsl_vector_get (mu_bar, 4)));
    gsl_vector_set (mu_bar, 5, gslu_math_minimized_angle (gsl_vector_get (mu_bar, 5)));

    gsl_matrix_scale (Q, dt*.1);

    /* remove process model noise for when not flying */
    if (!nav->is_flying())
        for (int i=6; i<=11; ++i) gsl_matrix_set (Q, i, i, 0);
}

//----------------------------------------------------------------------------------
// Observation model functions
//----------------------------------------------------------------------------------

/*************************************************************************************
 * AR.Drone State Observations
 * This observation model function adds direct observations of on-board altimeter,
 * gyro, and optical flow. The resulting observation consists of altitude, roll,
 * pitch, yaw, and body-frame velocities, vx and vy.
 *************************************************************************************/
void
om_ekf_quad_state (const est_estimator_t *state, const gsl_vector *z,
        const void *index_map, const gsl_matrix *R,
        gsl_vector *nu, gsl_matrix *H, void *user)
{
    /* must fill nu and H */

    navigator *nav = (navigator *) user;

    double h, x_dot, y_dot;
    h = gsl_vector_get (state->mu, 5);
    x_dot = gsl_vector_get (state->mu, 6);
    y_dot = gsl_vector_get (state->mu, 7);

 
    if (nav->is_flying()) {
        // altimeter
        gsl_vector_set (nu, 0, gsl_vector_get (z, 0) - gsl_vector_get (state->mu, 2));
        gsl_matrix_set (H, 0, 2, 1); 
    }

    if (nav->is_flying()) {
        // vx/vy
        gsl_vector_set (nu, 4, gsl_vector_get (z, 4) - (cos(h)*x_dot + sin(h)*y_dot));
        gsl_vector_set (nu, 5, gsl_vector_get (z, 5) - (-sin(h)*x_dot + cos(h)*y_dot));
        gsl_matrix_set (H, 4, 5, -sin(h)*x_dot + cos(h)*y_dot);
        gsl_matrix_set (H, 4, 6, cos(h));
        gsl_matrix_set (H, 4, 7, sin(h));
        gsl_matrix_set (H, 5, 5, -cos(h)*x_dot - sin(h)*y_dot);
        gsl_matrix_set (H, 5, 6, -sin(h));
        gsl_matrix_set (H, 5, 7, cos(h));
    }

    // roll, pitch, yaw
    gsl_vector_set (nu, 1, gslu_math_minimized_angle (
                        gsl_vector_get (z, 1) - gsl_vector_get (state->mu, 3)));
    gsl_vector_set (nu, 2, gslu_math_minimized_angle (
                        gsl_vector_get (z, 2) - gsl_vector_get (state->mu, 4)));
    gsl_vector_set (nu, 3, gslu_math_minimized_angle (
                        gsl_vector_get (z, 3) - gsl_vector_get (state->mu, 5)));

    gsl_matrix_set (H, 1, 3, 1);
    gsl_matrix_set (H, 2, 4, 1);
    gsl_matrix_set (H, 3, 5, 1);
}

/*************************************************************************************
 * AR.Drone MoCap Observations
 * This observation model function directly observes the pose (xyzrph) of the
 * AR.Drone. 
 *************************************************************************************/
void
om_ekf_quad_mocap (const est_estimator_t *state, const gsl_vector *z,
        const void *index_map, const gsl_matrix *R,
        gsl_vector *nu, gsl_matrix *H, void *user)
{
    /* must fill nu and H */

    /* directly observing state variables */
    gsl_matrix_set (H, 0, 0, 1);
    gsl_matrix_set (H, 1, 1, 1);
    gsl_matrix_set (H, 2, 2, 1);
    gsl_matrix_set (H, 3, 3, 1);
    gsl_matrix_set (H, 4, 4, 1);
    gsl_matrix_set (H, 5, 5, 1);

    gsl_vector_set (nu, 0, gsl_vector_get (z, 0) - gsl_vector_get (state->mu, 0));
    gsl_vector_set (nu, 1, gsl_vector_get (z, 1) - gsl_vector_get (state->mu, 1));
    gsl_vector_set (nu, 2, gsl_vector_get (z, 2) - gsl_vector_get (state->mu, 2));
    gsl_vector_set (nu, 3, gslu_math_minimized_angle (gsl_vector_get (z, 3) - gsl_vector_get (state->mu, 3)));
    gsl_vector_set (nu, 4, gslu_math_minimized_angle (gsl_vector_get (z, 4) - gsl_vector_get (state->mu, 4)));
    gsl_vector_set (nu, 5, gslu_math_minimized_angle (gsl_vector_get (z, 5) - gsl_vector_get (state->mu, 5)));

}

/*************************************************************************************
 * APRIL Tag Observations
 * This observation model function handles *individual* APRIL tag observations. Thus,
 * we are directly observing the pose from the camera to the *tag* (x_st). Note, however,
 * that our state vector models the pose from the local frame to the *target* (x_lT), 
 * so we must have navigator::x_active_tag_to_target set to the transformation from
 * the tag currently being viewed to the center of the target.
 *************************************************************************************/
void
om_ekf_april_tag (const est_estimator_t *state, const gsl_vector *z,
        const void *index_map, const gsl_matrix *R,
        gsl_vector *nu, gsl_matrix *H, void *user)
{
    /* must fill nu and H */

    navigator *nav = (navigator *) user;

    gsl_vector *x_vt            = gsl_vector_calloc (6);
    gsl_vector *x_stag          = gsl_vector_calloc (6);
    gsl_vector *x_lv            = gsl_vector_calloc (6);
    gsl_vector *x_lt            = gsl_vector_calloc (6);
    gsl_vector *x_starget       = gsl_vector_calloc (6);
    gsl_matrix *J_A             = gsl_matrix_calloc (6, 12);
    gsl_matrix *J_B             = gsl_matrix_calloc (6, 12);
    gsl_matrix *J_B2            = gsl_matrix_calloc (6, 6);
    gsl_matrix *J_C             = gsl_matrix_calloc (6, 12);
    gsl_matrix *J_C1            = gsl_matrix_calloc (6, 6);
    gsl_matrix *J_CB            = gsl_matrix_calloc (6, 6);
    gsl_matrix *T               = gsl_matrix_calloc (12, 24);
    gsl_vector *x_target_to_tag = gsl_vector_calloc (6);

    for (int i=0; i<6; i++) {
        gsl_vector_set (x_lv, i, gsl_vector_get (state->mu, i));
        gsl_vector_set (x_lt, i, gsl_vector_get (state->mu, i+12));
    }

    /* compute h (expected observation -- sensor to *tag*) */
    ssc_tail2tail_gsl (x_vt, J_A, x_lv, x_lt);
    ssc_tail2tail_gsl (x_starget, J_B, nav->get_quad_to_cam(), x_vt);
    ssc_inverse_gsl (x_target_to_tag, NULL, nav->get_tag_to_target());
    ssc_head2tail_gsl (x_stag, J_C, x_starget, x_target_to_tag);
    

    for (int i=0; i<6; i++) {
        gsl_vector_set (nu, i, gsl_vector_get (z, i) - gsl_vector_get (x_stag, i));

        for (int j=0; j<6; j++) {
            gsl_matrix_set (J_B2, i, j, gsl_matrix_get (J_B, i, j+6));
            gsl_matrix_set (J_C1, i, j, gsl_matrix_get (J_C, i, j));
        }
    }

    /* fill out T matrix that maps 24x24 cov to 12x12 (ie. elim. velocity terms) */
    for (int i=0; i<6; i++) {
        gsl_matrix_set (T, i, i, 1);
        gsl_matrix_set (T, i+6, i+12, 1);
    }

    gsl_matrix_view H_part = gsl_matrix_submatrix (H, 0, 0, 6, 24);

    gslu_blas_mm (J_CB, J_C1, J_B2);
    gslu_blas_mmm (&H_part.matrix, J_CB, J_A, T, NULL);

    printf ("------>meas: %4.4f, pred: %4.4f\n", gsl_vector_get(z,6), gsl_vector_get(state->mu, 2));
    gsl_vector_set (nu, 6, gsl_vector_get (z, 6) - gsl_vector_get (state->mu, 2));
    gsl_matrix_set (H, 6, 2, 1);

    /* cleanup */
    gsl_matrix_free (J_A);
    gsl_matrix_free (J_B);
    gsl_matrix_free (J_B2);
    gsl_matrix_free (J_C);
    gsl_matrix_free (J_C1);
    gsl_matrix_free (J_CB);
    gsl_matrix_free (T);
    gsl_vector_free (x_vt);
    gsl_vector_free (x_stag);
    gsl_vector_free (x_starget);
    gsl_vector_free (x_lv);
    gsl_vector_free (x_lt);
    gsl_vector_free (x_target_to_tag);
}
