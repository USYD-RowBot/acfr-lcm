#include <stdio.h>
#include <stdlib.h>

// external linking req'd
#include <glib.h>

#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/units.h"

#include "perls-math/fasttrig.h"
#include "perls-math/gsl_util.h"

// Input lcmdefs from sensors
#include "perls-lcmtypes/perllcm_segway_state_t.h"
#include "perls-lcmtypes/senlcm_gpsd_t.h"
#include "perls-lcmtypes/senlcm_kvh_dsp3000_t.h"
#include "perls-lcmtypes/senlcm_ms_gx1_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/senlcm_ms_gx3_t.h"
#include "perls-lcmtypes/senlcm_ms_gx3_25_t.h"

// Output lcmdefs
#include "perls-lcmtypes/perllcm_est_navigator_index_t.h"
#include "perls-lcmtypes/perllcm_segway_navigator_t.h"
#include "perls-lcmtypes/perllcm_est_navigator_debug_meas_t.h"
#include "perls-lcmtypes/perllcm_est_navigator_debug_pred_t.h"

#include "lcm_callbacks.h"
#include "navigator.h"

#define DO_INIT 0


//Init the state structure
state_t *state = NULL;

//------------------------------------------------------------------------------
// Called when program shuts down
//------------------------------------------------------------------------------
static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("my_signal_handler()\n");
    if (state->done)
    {
        printf ("Goodbye\n");
        exit (EXIT_FAILURE);
    }
    else
        state->done = 1;
}

//------------------------------------------------------------------------------
// Map string keys in config file to function pointers in lib_est
// also fills the index_t structure
// 0 for success, -1 if failure
//------------------------------------------------------------------------------
static int
pmf_key_2_fptr (char *key)
{
    if (0==strcmp (key, "PMF_DIFFERENTIAL_DRIVE"))
    {
        // if we start adding a ton of PMF we can move these to a speperate file

        // set callback
        state->pred_t->pmf_ekf = est_pmf_differential_drive;
        state->pred_t->pmf_ukf = NULL;
        state->pred_t->pmf_pf = NULL;

        // get indexing stucture
        est_pmf_differential_drive_get_index_t (state->index_t);
        // allocate user data structure
        state->pred_t->user = est_pmf_differential_drive_alloc_user_t ();
        // save a pointer to the free function
        state->pred_user_t_free = &est_pmf_differential_drive_free_user_t;
        // fill the user structure
        est_pmf_differential_drive_user_t *usr = state->pred_t->user;
        // noise
        botu_param_read_covariance (usr->Qu, state->param, "navigator.pmf.Qu");
        botu_param_read_covariance (usr->Qadd, state->param, "navigator.pmf.Qadd");
        // differential drive params
        usr->axel_track = bot_param_get_double_or_fail (state->param, "navigator.pmf.axel_track");
        usr->wheel_radius = bot_param_get_double_or_fail (state->param, "navigator.pmf.wheel_radius");
        usr->wheel_size_fudge_factor = 1;
        bot_param_get_double (state->param, "navigator.pmf.wheel_size_fudge_factor", &usr->wheel_size_fudge_factor);

        return 0;
    }
    else
        return -1;

}

//------------------------------------------------------------------------------
// Map string keys in config file to lcm callback
// 0 for success, -1 if failure
//------------------------------------------------------------------------------
static int
omf_key_2_fptr (char *key, char *lcm_chan)
{
    if (0==strcmp (key, "OMF_MS_GX3_RPH"))
    {
        senlcm_ms_gx3_t_subscribe (state->lcm, lcm_chan, &ms_gx3_rph_cb, state);
        return 0;
    }
    else if (0==strcmp (key, "OMF_MS_GX3_ABC"))
    {
        senlcm_ms_gx3_t_subscribe (state->lcm, lcm_chan, &ms_gx3_abc_cb, state);
        return 0;
    }
    else if (0==strcmp (key, "OMF_MS_GX3_RP"))
    {
        senlcm_ms_gx3_t_subscribe (state->lcm, lcm_chan, &ms_gx3_rp_cb, state);
        return 0;
    }
    else if (0==strcmp (key, "OMF_MS_GX3_AB"))
    {
        senlcm_ms_gx3_t_subscribe (state->lcm, lcm_chan, &ms_gx3_ab_cb, state);
        return 0;
    }
    else if (0==strcmp (key, "OMF_MS_GX3_25_RP"))
    {
        senlcm_ms_gx3_25_t_subscribe (state->lcm, lcm_chan, &ms_gx3_rp_25_cb, state);
        return 0;
    }
    else if (0==strcmp (key, "OMF_MS_GX3_25_AB"))
    {
        senlcm_ms_gx3_25_t_subscribe (state->lcm, lcm_chan, &ms_gx3_ab_25_cb, state);
        return 0;
    }
    else
        return -1;
}

//------------------------------------------------------------------------------
// init filter from config file
//------------------------------------------------------------------------------
static void
init_param (void)
{
    printf ("Initializing navigator from config file.\n");

    fasttrig_init ();

    state->meas_t_next = NULL;
    state->utime_last_heartbeat = 0;
    state->filter_running = 0;

    // parse the config file
    char *nav_param_fname = BOTU_PARAM_DEFAULT_CFG;
    state->param = bot_param_new_from_file (nav_param_fname);

    // parse the config file
    state->debug = 0;
    bot_param_get_int (state->param, "navigator.est.debug", &state->debug);

    state->max_pm_dt = 100000;
    bot_param_get_int (state->param, "navigator.est.max_pm_dt", &state->max_pm_dt);

    state->est_engine = "EKF";
    bot_param_get_str (state->param, "navigator.est.est_engine", &state->est_engine);

    state->org_lat = bot_param_get_double_or_fail (state->param, "navigator.est.org_lat")* UNITS_DEGREE_TO_RADIAN;
    state->org_lon = bot_param_get_double_or_fail (state->param, "navigator.est.org_lon")* UNITS_DEGREE_TO_RADIAN;

    // parse lcm channel names
    state->prefix = bot_param_get_str_or_fail (state->param, "vehicle.lcm_channel_prefix");
    state->channel = bot_param_get_str_or_fail (state->param, "navigator.channel");
    state->channel_debug_meas = bot_param_get_str_or_fail (state->param, "navigator.channel_debug_meas");
    state->channel_debug_pred = bot_param_get_str_or_fail (state->param, "navigator.channel_debug_pred");
    state->channel_drop_ds = bot_param_get_str_or_fail (state->param, "navigator.channel_drop_ds");
    state->channel_drop_ds_ack = bot_param_get_str_or_fail (state->param, "navigator.channel_drop_ds_ack");


    state->hb1_channel = lcmu_channel_get_heartbeat (state->prefix, 1);
    state->hb5_channel = lcmu_channel_get_heartbeat (state->prefix, 5);
    state->hb10_channel = lcmu_channel_get_heartbeat (state->prefix, 10);
    state->hb20_channel = lcmu_channel_get_heartbeat (state->prefix, 20);
    state->hb30_channel = lcmu_channel_get_heartbeat (state->prefix, 30);
    state->hb40_channel = lcmu_channel_get_heartbeat (state->prefix, 40);
    state->hb50_channel = lcmu_channel_get_heartbeat (state->prefix, 50);

    state->kvh_channel = bot_param_get_str_or_fail (state->param, "sensors.kvh-dsp3000.gsd.channel");

    // read in process model information
    state->pred_t = calloc (1, sizeof (*state->pred_t));
    state->index_t = calloc (1, sizeof (*state->index_t));
    // load string name of process model from config file
    char *pmf_str_key = bot_param_get_str_or_fail (state->param, "navigator.pmf.pmf_name");
    if (pmf_key_2_fptr (pmf_str_key))
        ERROR ("PMF key \"%s\" not found.\n", pmf_str_key);

    int state_len = state->index_t->proc_state_len;
    if (state->index_t->u_len)
        state->pred_t->u = gsl_vector_calloc (state->index_t->u_len);
    else
        state->pred_t->u = NULL;
    printf ("Added Process Model: %s\n", pmf_str_key);

    // allocate process model noise
    state->pred_t->Q = gsl_matrix_calloc (state_len, state_len);

    // load observations model information
    int num_omfs = bot_param_get_num_subkeys (state->param, "navigator.omfs");
    printf ("Number of Observation Models %d\n", num_omfs);
    char **omf_keys = bot_param_get_subkeys (state->param, "navigator.omfs");
    // loop over each observation model
    for (int i=0; i<num_omfs; i++)
    {
        char *omf_lcm_chan=NULL, *sensor_key=NULL;

        char cfg_str_key[256];
        sprintf (cfg_str_key, "navigator.omfs.%s.omf_name", omf_keys[i]);
        char *omf_name = bot_param_get_str_or_fail (state->param, cfg_str_key);

        sprintf (cfg_str_key, "navigator.omfs.%s.sensor_key", omf_keys[i]);
        if (0==bot_param_get_str (state->param, cfg_str_key, &sensor_key))
        {
            sprintf (cfg_str_key, "sensors.%s.gsd.channel", sensor_key);
            if (bot_param_get_str (state->param, cfg_str_key, &omf_lcm_chan))
                ERROR ("ERROR: channel not found for \"sensors.%s.gsd\" Assuming identity.\n", sensor_key);
        }
        else
        {
            sprintf (cfg_str_key, "navigator.omfs.%s.lcm_chan", omf_keys[i]);
            if (bot_param_get_str (state->param, cfg_str_key, &omf_lcm_chan))
                ERROR ("ERROR: lcm channel not found for \"%s\" through lcm_chan nor sensor_key.\n", omf_keys[i]);
        }

        // Map config key to lcm publish
        if (omf_key_2_fptr (omf_name, omf_lcm_chan))
        {
            printf ("OMF key \"%s\" not found. SKIPPED!\n", omf_name);
            continue;
        }

        printf ("Added Observation Model: %s\n", omf_keys[i]);
    }

    // initial conditions for filter
    int mu_len =  bot_param_get_array_len (state->param, "navigator.est.ini_mu");
    if (mu_len != state_len)
        ERROR ("ERROR est.ini_mu not found or incorrect length, should be %d", state_len);
    double *ini_mu = calloc (state_len, sizeof (*ini_mu));
    int *ini_mu_dtor = calloc (state_len, sizeof (*ini_mu));

    bot_param_get_double_array (state->param, "navigator.est.ini_mu", ini_mu, state_len);
    gsl_vector *ini_mu_tmp = gsl_vector_calloc (state_len);
    if (state_len == bot_param_get_int_array (state->param, "navigator.est.ini_mu_dtor", ini_mu_dtor, state_len))
    {
        for (int k=0; k<state_len ; k++)
        {
            if (ini_mu_dtor[k])
                ini_mu[k] = ini_mu[k] * UNITS_DEGREE_TO_RADIAN;
        }

    }
    memcpy (ini_mu_tmp->data, ini_mu, state_len*sizeof(double));

    gsl_matrix *ini_Sigma_tmp = gsl_matrix_calloc (state_len, state_len);
    botu_param_read_covariance (ini_Sigma_tmp, state->param, "navigator.est.ini_Sigma");


    // instantiate estimator
    if (0==strcmp ("EKF", state->est_engine))
    {
        state->estimator = est_init_ekf (state_len, ini_mu_tmp, ini_Sigma_tmp);
        //est_init_ds (state->estimator, MAX_DELAYED_STATES, state_len);
        est_init_ds (state->estimator, -1, state_len); // unlimited delayed states, manually manage marginalization
    }
    else if (0==strcmp ("UKF", state->est_engine))
    {
        //state->estimator = est_init_ukf (state_len, ini_state, ini_cov);
        printf ("UKF not yet implemented, use EKF\n");
    }
    else if (0==strcmp ("PF", state->est_engine))
    {
        //state->estimator = est_init_pf (state_len, ini_state, ini_cov);
        printf ("PF not yet implemented, use EKF\n");
    }
    else
        ERROR ("ERROR estimator engine %s not found, try \"EKF\", \"UKF\" or \"PF\"\n", state->est_engine);

    //clean up tmp vars
    free (ini_mu);
    free (ini_mu_dtor);
    gsl_vector_free (ini_mu_tmp);
    gsl_matrix_free (ini_Sigma_tmp);
}

//------------------------------------------------------------------------------
// Clean up before exit
//------------------------------------------------------------------------------
static void
clean_up (void)
{
    printf ("Cleaning up\n");

    if (state->param) bot_param_destroy(state->param);
    if (state->index_t) free (state->index_t);

    if (state->pred_t)
    {
        if (state->pred_t->user)
        {
            (*state->pred_user_t_free) (state->pred_t->user);
        }
        free (state->pred_t);
    }

    if (state->estimator) est_free (state->estimator);
    if (state->lcm)  lcm_destroy (state->lcm);

    printf ("done\n");
}

// control callbacks
//------------------------------------------------------------------------------
// Segway state
//------------------------------------------------------------------------------
static void
perllcm_segway_state_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                           const perllcm_segway_state_t *msg, void *user)
{

    state_t *state = user;

    state->current_wl = msg->left_wheel_velocity;
    state->current_wr = msg->right_wheel_velocity;
}

//------------------------------------------------------------------------------
// KVH controll callback
//------------------------------------------------------------------------------
static void
senlcm_kvh_dsp3000_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                         const senlcm_kvh_dsp3000_t *msg, void *user)
{

    state_t *state = user;

    if (msg->mode == SENLCM_KVH_DSP3000_T_ANGLE_MODE)
    {
        // negative because upside down
        state->current_kvh_angle = -msg->data;
    }

}


//------------------------------------------------------------------------------
// Heartbeat callback used to regularly publish status
//------------------------------------------------------------------------------
static void
perllcm_heartbeat_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                        const perllcm_heartbeat_t *msg, void *user)
{
    state_t *state = user;


    // used to set dt for prection in main loop
    state->utime_last_heartbeat = msg->utime;

    perllcm_segway_navigator_t *nav_out = calloc (1, sizeof (*nav_out));

    nav_out->utime = state->utime_last_update;
    nav_out->est_method = state->estimator->est_method;
    nav_out->filter_running = state->filter_running;

    // current estimator state
    if (state->filter_running)
    {

        int sl = state->estimator->state_len;
        int nds = state->estimator->delayed_states_cnt;

        nav_out->state_len = sl;

        nav_out->delayed_state_len = state->estimator->delayed_state_len;
        nav_out->max_delayed_states = state->estimator->max_delayed_states;
        nav_out->delayed_states_cnt = nds;
        nav_out->delayed_states_utime = calloc (nds, sizeof (*nav_out->delayed_states_utime));

        memcpy (nav_out->delayed_states_utime, state->utime_delayed_states,
                sizeof(nav_out->delayed_states_utime)*nds);

        nav_out->mu_len = sl;
        nav_out->Sigma_len = sl*sl;
        nav_out->mu = calloc (sl, sizeof (*nav_out->mu));
        nav_out->Sigma = calloc (sl*sl, sizeof (*nav_out->Sigma));

        memcpy (nav_out->mu, state->estimator->mu->data,
                sizeof(nav_out->mu)*sl);
        memcpy (nav_out->Sigma, state->estimator->Sigma->data,
                sizeof(nav_out->Sigma)*sl*sl);
    }
    else   // if the filter isnt running don't publish state
    {
        nav_out->mu_len = 0;
        nav_out->Sigma_len = 0;
        nav_out->mu = NULL;
        nav_out->Sigma = NULL;
    }
    // copy index structure from state
    memcpy (&nav_out->index, state->index_t,  sizeof (*state->index_t));

    nav_out->org_latitude = state->org_lat;
    nav_out->org_longitude = state->org_lon;

    // publish the message
    perllcm_segway_navigator_t_publish (state->lcm, state->channel, nav_out);

    // free allocated arrays
    free (nav_out->delayed_states_utime);
    free (nav_out->mu);
    free (nav_out->Sigma);
    free (nav_out);
}

//------------------------------------------------------------------------------
// Publish navigator_debug_meas_t msgs
//------------------------------------------------------------------------------
static void
pub_debug_meas (void)
{
    perllcm_est_navigator_debug_meas_t *debug_out = calloc (1, sizeof (*debug_out));
    debug_out->utime = state->estimator->last_utime;
    debug_out->id_str = state->estimator->last_id_str;
    debug_out->meas_len = state->estimator->last_z->size;
    debug_out->z = state->estimator->last_z->data;
    debug_out->nu = state->estimator->last_nu->data;
    debug_out->nis = state->estimator->last_nis;
    debug_out->mahal_innov_passed = state->estimator->last_mahal_innov_passed;

    perllcm_est_navigator_debug_meas_t_publish (state->lcm, state->channel_debug_meas, debug_out);

    free (debug_out);
}

//------------------------------------------------------------------------------
// Publish navigator_debug_pred_t msgs
//------------------------------------------------------------------------------
static void
pub_debug_pred (void)
{
    perllcm_est_navigator_debug_pred_t *debug_out = calloc (1, sizeof (*debug_out));
    debug_out->utime = state->utime_last_update;
    debug_out->dt = state->pred_t->dt;
    perllcm_est_navigator_debug_pred_t_publish (state->lcm, state->channel_debug_pred, debug_out);
    free (debug_out);
}

// -----------------------------------------------------------------------------
// build control vector
// -----------------------------------------------------------------------------
void
build_control_vector (void)
{
    gsl_vector_set (state->pred_t->u, 0, state->current_wl); // left wheel velocity
    gsl_vector_set (state->pred_t->u, 1, state->current_wr); // right wheel velocity
    // find the delta heading and use it as control
    double dh_corrected = state->current_kvh_angle - state->last_predict_kvh_angle; // delta heading

    // baias from pitch and roll (doesnt work! probably a bug)
    //double r = gsl_vector_get (state->estimator->mu, state->index_t->r);
    //double p = gsl_vector_get (state->estimator->mu, state->index_t->p);
    //double b = gsl_vector_get (state->estimator->mu, state->index_t->b);
    //double c = gsl_vector_get (state->estimator->mu, state->index_t->c);
    //double r_bias = ((cos(r)*b - sin(r)*c)/cos(p) * r) * state->pred_t->dt;
    //double p_bias = (tan(p)*(cos(r)*b + sin(r)*c)/cos(p) * p) * state->pred_t->dt;
    //printf ("r_bias = %lf, p_bias = %lf, total = %lf \n", r_bias, p_bias, r_bias+p_bias);
    //dh_corrected = dh_corrected - r_bias - p_bias;

    //// different way of pitch roll corrections (doesnt work either! probably a bug)
    //double r = gsl_vector_get (state->estimator->mu, state->index_t->r);
    //double p = gsl_vector_get (state->estimator->mu, state->index_t->p);
    //double b = gsl_vector_get (state->estimator->mu, state->index_t->b);
    //double c = dh_corrected/state->pred_t->dt; // mean body yaw rate
    //double h_dot = (sin(r)*b + cos(r)*c)/cos(p);
    ////double h_dot = cos(r)*c/cos(p); // mean Euler yaw rate
    //dh_corrected =  h_dot*state->pred_t->dt;

    // correct for earth rate using origin latitude
    double earth_rate = lat_to_earth_rate (state->org_lat);
    dh_corrected = dh_corrected - earth_rate*state->pred_t->dt;

    // approximate kvh bias (showed a pretty big improvement)
    // dh_corrected = dh_corrected - (-5e-5)*state->pred_t->dt;

    gsl_vector_set (state->pred_t->u, 2, dh_corrected); // delta heading
}



//------------------------------------------------------------------------------
// Heartbeat callback used to regularly drop delayed states
//------------------------------------------------------------------------------
static void
delayed_state_aug_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                      const perllcm_heartbeat_t *msg, void *user)
{
    // check if this is the first time through and initialize time
    if (state->utime_last_update == 0)
    {
        state->utime_last_update = msg->utime;
    }

    // calculate dt
    state->pred_t->dt = (msg->utime - state->utime_last_update)/1e6;

    if (state->pred_t->dt > 1 )
        printf ("WARNING: DT > 1 SECONDS: LCM PACKET LOSS?\n");

    if (state->pred_t->dt > 10 )
    {
        ERROR ("ERROR: DT > 10 SECONDS: LCM PACKET LOSS?");
        printf ("HEARTBEAT UTIME = %"PRId64"\n", state->utime_last_heartbeat);
        state->done = 1; //quit
    }

    // if we have a very small or negative (due to lcm network lag order)
    // then do do dt=0 augmentation
    if (state->pred_t->dt <= DT_EPS_SEC)
    {
        state->pred_t->dt = 0.0;
    }

    // build control vector
    build_control_vector ();

    //  perform prediction step (with augmentation)
    est_predict_ds (state->estimator, state->pred_t, 1);

    // keep track of delayed state update times
    push_utime_delayed_states (state->utime_delayed_states, msg->utime);
    state->idx_last_graph_node++;

    // marganilize out unwanted poses
    // first marginalize out poses after current graph node if there are any
    int ind = state->idx_last_graph_node+1;
    while (state->estimator->delayed_states_cnt > ind)
    {
        est_marginalize_ds (state->estimator, ind);
        remove_utime_delayed_states (state->utime_delayed_states, ind);
    }
    // if we still have more than MAX_DELAYED_STATES remove states before current graph node
    while (state->estimator->delayed_states_cnt > MAX_DELAYED_STATES)
    {
        int ind = state->idx_last_graph_node-1;
        est_marginalize_ds (state->estimator, ind);
        remove_utime_delayed_states (state->utime_delayed_states, ind);
        state->idx_last_graph_node--;
    }

    // update time predicted till
    state->utime_last_update = msg->utime;

    // start fresh calculating the next delta heading
    state->last_predict_kvh_angle = state->current_kvh_angle;

    // publish a debug message
    if (state->debug)
        pub_debug_pred ();
}

//------------------------------------------------------------------------------
// Heartbeat callback to drop ds command from seg_graph_manager
//------------------------------------------------------------------------------
static void
drop_ds_cb (const lcm_recv_buf_t *rbuf, const char *channel,
            const perllcm_heartbeat_t *msg, void *user)
{
    state_t *state = user;

    // find the delayed state utime closest to the requested utime
    int64_t max_dt = 1e6/20; // we have 20hz delayed states
    int64_t dt = max_dt;
    int idx_closest = -1;
    for (int i=0; i<state->idx_last_graph_node-1 ; i++)
    {
        int64_t tmp = abs (msg->utime - state->utime_delayed_states[i]);
        if (tmp < dt)
        {
            dt = tmp;
            idx_closest = i;
        }
    }

    int found = 0;
    if (dt == max_dt)
    {
        ERROR ("ERROR: Unable to find delayed state within %lf secs!", max_dt/1e6);
    }
    else
    {
        found = 1;
    }

    // pull out the data for this pose and its previous pose and publish it
    perllcm_segway_navigator_t *nav_out = calloc (1, sizeof (*nav_out));

    nav_out->utime = state->utime_last_update;
    nav_out->est_method = state->estimator->est_method;
    nav_out->filter_running = state->filter_running;

    // current estimator state
    if (state->filter_running && found &&
            state->estimator->state_len > state->idx_last_graph_node*state->estimator->delayed_state_len)
    {

        int dsl = state->estimator->delayed_state_len;
        int sl = 2*dsl;
        int mds = 2;
        nav_out->state_len = sl;

        nav_out->delayed_state_len = state->estimator->delayed_state_len;
        nav_out->delayed_states_cnt = 2;
        nav_out->max_delayed_states = mds;
        nav_out->delayed_states_utime = calloc (mds, sizeof (*nav_out->delayed_states_utime));
        nav_out->delayed_states_utime[0] = state->utime_delayed_states[idx_closest];
        nav_out->delayed_states_utime[1] = state->utime_delayed_states[state->idx_last_graph_node];

        nav_out->mu_len = sl;
        nav_out->Sigma_len = sl*sl;
        nav_out->mu = calloc (sl, sizeof (*nav_out->mu));
        nav_out->Sigma = calloc (sl*sl, sizeof (*nav_out->Sigma));
        // copy state
        memcpy (&(nav_out->mu[0]),
                &(state->estimator->mu->data[idx_closest*dsl]),
                sizeof(nav_out->mu)*dsl);
        memcpy (&(nav_out->mu[dsl]),
                &(state->estimator->mu->data[state->idx_last_graph_node*dsl]),
                sizeof(nav_out->mu)*dsl);
        // copy covariance
        gsl_matrix *Sigma_out = gsl_matrix_calloc (sl, sl);
        // upper left
        gsl_matrix_view upper_left = gsl_matrix_submatrix (state->estimator->Sigma,
                                     idx_closest*dsl, idx_closest*dsl,
                                     dsl, dsl);
        gslu_matrix_set_submatrix (Sigma_out, 0, 0, &upper_left.matrix);
        // upper right
        gsl_matrix_view upper_right = gsl_matrix_submatrix (state->estimator->Sigma,
                                      idx_closest*dsl, state->idx_last_graph_node*dsl,
                                      dsl, dsl);
        gslu_matrix_set_submatrix (Sigma_out, 0, dsl, &upper_right.matrix);
        // lower left
        gsl_matrix_view lower_left = gsl_matrix_submatrix (state->estimator->Sigma,
                                     state->idx_last_graph_node*dsl, idx_closest*dsl,
                                     dsl, dsl);
        gslu_matrix_set_submatrix (Sigma_out, dsl,0 , &lower_left.matrix);
        // lower right
        gsl_matrix_view lower_right = gsl_matrix_submatrix (state->estimator->Sigma,
                                      state->idx_last_graph_node*dsl, state->idx_last_graph_node*dsl,
                                      dsl, dsl);
        gslu_matrix_set_submatrix (Sigma_out, dsl, dsl, &lower_right.matrix);


        memcpy (nav_out->Sigma, Sigma_out->data, sizeof(nav_out->Sigma)*sl*sl);
        gsl_matrix_free (Sigma_out);
    }
    else   // if the filter isnt running don't publish state
    {
        nav_out->mu_len = 0;
        nav_out->Sigma_len = 0;
        nav_out->mu = NULL;
        nav_out->Sigma = NULL;
    }
    // copy index structure from state
    memcpy (&nav_out->index, state->index_t,  sizeof (*state->index_t));

    nav_out->org_latitude = state->org_lat;
    nav_out->org_longitude = state->org_lon;

    // publish the message
    perllcm_segway_navigator_t_publish (state->lcm, state->channel_drop_ds_ack, nav_out);

    // free allocated arrays
    free (nav_out->delayed_states_utime);
    free (nav_out->mu);
    free (nav_out->Sigma);
    free (nav_out);

    if (found)
    {
        state->idx_last_graph_node = idx_closest;

        // for some reason this isnt working ... if we have low delay drop msgs
        // need to fix it, not a problem when using the camera
        //// force an augmentation otherwise new predictions will modify this pose and
        //// when we later compute the odometry we will find that it has changed, producing
        //// incorrect odometry
        //if (idx_closest == 0) {
        //
        //    // calculate dt
        //    state->pred_t->dt = 0.0;
        //    gsl_vector_set (state->pred_t->u, 0, 0.0);
        //    gsl_vector_set (state->pred_t->u, 0, 0.0);
        //    gsl_vector_set (state->pred_t->u, 0, 0.0);
        //
        //    //  perform prediction step (with augmentation)
        //    est_predict_ds (state->estimator, state->pred_t, 1);
        //
        //    // keep track of delayed state update times
        //    push_utime_delayed_states (state->utime_delayed_states, state->utime_delayed_states[idx_closest]);
        //    state->idx_last_graph_node++;
        //
        //}
        if (idx_closest == 0)
            ERROR ("BUG: NOT ENOUGH LAG IN DROP DS MSGS");
    }
}


// -----------------------------------------------------------------------------
// predict and correct function called in main loop
// -----------------------------------------------------------------------------
static void
predict_correct (void)
{
    int64_t update_utime_now;
    int have_meas = (NULL != state->meas_t_next);
    int have_heartbeat = (0 != state->utime_last_heartbeat);

    if (have_meas && have_heartbeat)
        ERROR ("ERROR: have_meas && have_heartbeat. Should not happen");

    // check if this is the first time through and initialize time
    if (state->utime_last_update == 0)
    {
        if (have_meas)
            state->utime_last_update = state->meas_t_next->utime;
        else if (have_heartbeat)
            state->utime_last_update = state->utime_last_heartbeat;
    }

    // calculate dt
    if (have_meas)
    {
        update_utime_now = state->meas_t_next->utime;
    }
    else if (have_heartbeat)
    {
        update_utime_now = state->utime_last_heartbeat;
    }
    else     // not a heartbeat or a measurement nothing else to do
    {
        return;
    }
    state->pred_t->dt = (update_utime_now - state->utime_last_update)/1e6;


    if (state->pred_t->dt > 1 )
        printf ("WARNING: DT > 1 SECONDS: LCM PACKET LOSS?\n");

    if (state->pred_t->dt > 10 )
    {
        ERROR ("ERROR: DT > 10 SECONDS: LCM PACKET LOSS?");
        if (have_meas)
            printf ("MEAS %s UTIME = %"PRId64"\n", state->meas_t_next->id_str, state->meas_t_next->utime );
        else if (have_heartbeat)
            printf ("HEARTBEAT UTIME = %"PRId64"\n", state->utime_last_heartbeat);
        state->done = 1; //quit
    }

    // should we preform a predict step?
    if (state->pred_t->dt > DT_EPS_SEC)
    {

        // yes, it has been more than our eps

        // build control vector
        build_control_vector ();

        //  perform prediction step (without augmentation)
        est_predict_ds (state->estimator, state->pred_t, 0);

        // update time predicted till
        state->utime_last_update = update_utime_now;

        // start fresh calculating the next delta heading
        state->last_predict_kvh_angle = state->current_kvh_angle;

        // publish a debug message
        if (state->debug)
            pub_debug_pred ();

    }
    else if (state->pred_t->dt < -DT_EPS_SEC)
    {
        // error we have a negative dt (more negative than eps) reset and continue
        // TOO MUCH PRINTING TO SCREEN
        //ERROR ("\nERROR: Negative DT = %f", state->pred_t->dt);
        if (have_meas)
        {
            //printf ("For %s\n", state->meas_t_next->id_str);
            free_meas_t (state->meas_t_next);
            state->meas_t_next = NULL;
        }
        return;

    } //else (in between) its fine to correct but too soon to update again

    // if we have a measurement correct
    if (have_meas)
    {
        // correct
        est_correct (state->estimator, state->meas_t_next);
        // publish a debug message
        if (state->debug)
            pub_debug_meas ();

        // reset next measurement, freeing what was allocated during lcm callback
        free_meas_t (state->meas_t_next);
        state->meas_t_next = NULL;
    }

}

//------------------------------------------------------------------------------
// Microstrain callback for initing filter
//------------------------------------------------------------------------------
static void
senlcm_ms_gx3_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                    const senlcm_ms_gx3_t *msg, void *user)
{
    state_t *state = user;

    ERROR ("ERROR INIT NOT IN ROBOT FRAME YET!");

    // only look at updates to stabilized euler angles
    if (msg->bitmask & SENLCM_MS_GX1_T_STAB_EULER)
    {

        int ii = state->init_rph_cnt % INIT_SAMPLES_RPH;
        state->init_rph[0][ii] = msg->sEuler[0];
        state->init_rph[1][ii] = msg->sEuler[1];
        state->init_rph[2][ii] = msg->sEuler[2];

        //init the last update time to the most msg utime
        state->utime_last_update = msg->utime;
        state->init_rph_cnt++;

    }
    else if (msg->bitmask & SENLCM_MS_GX1_T_STAB_ANGRATE)
    {
        int ii = state->init_abc_cnt % INIT_SAMPLES_ABC;
        state->init_abc[0][ii] = msg->sAngRate[0];
        state->init_abc[1][ii] = msg->sAngRate[1];
        state->init_abc[2][ii] = msg->sAngRate[2];

        //init the last update time to the most msg utime
        state->utime_last_update = msg->utime;
        state->init_abc_cnt++;
    }
}

//------------------------------------------------------------------------------
// Check if we have all the data we need to init filter
//------------------------------------------------------------------------------
static int
is_init_data (void)
{
    if (state->init_rph_cnt >= INIT_SAMPLES_RPH &&
            state->init_abc_cnt >= INIT_SAMPLES_ABC
       )
    {
        state->init_data_ready = 1;
        return 1;
    }
    else
    {
        printf ("INIT MEAS: rph->%d/%d, abc %d/%d \r",
                state->init_rph_cnt, INIT_SAMPLES_RPH,
                state->init_abc_cnt, INIT_SAMPLES_ABC);
        return 0;
    }
}

//------------------------------------------------------------------------------
// After initialization data has been collected use it to set the mean of the filter
//------------------------------------------------------------------------------
static void
init_filter (void)
{
    // set initial state for filter
    gsl_vector_set (state->estimator->mu, state->index_t->r,
                    gslu_stats_median_array (state->init_rph[0], 1, INIT_SAMPLES_RPH));
    gsl_vector_set (state->estimator->mu, state->index_t->p,
                    gslu_stats_median_array (state->init_rph[1], 1, INIT_SAMPLES_RPH));
    gsl_vector_set (state->estimator->mu, state->index_t->h,
                    gslu_stats_median_array (state->init_rph[2], 1, INIT_SAMPLES_RPH));
    gsl_vector_set (state->estimator->mu, state->index_t->a,
                    gslu_stats_median_array (state->init_abc[0], 1, INIT_SAMPLES_ABC));
    gsl_vector_set (state->estimator->mu, state->index_t->b,
                    gslu_stats_median_array (state->init_abc[1], 1, INIT_SAMPLES_ABC));
    gsl_vector_set (state->estimator->mu, state->index_t->c,
                    gslu_stats_median_array (state->init_abc[2], 1, INIT_SAMPLES_ABC));

    gslu_vector_printf (state->estimator->mu, "Mu initially set to:");
    printf ("INIT utime = %"PRId64"\n", state->utime_last_update);

    // start the filter
    state->filter_running = 1;
}


//------------------------------------------------------------------------------
// MAIN
//------------------------------------------------------------------------------
int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    state = calloc (1, sizeof (*state));

    state->idx_last_graph_node = 0;

    // install custom signal handler
    struct sigaction act = { .sa_sigaction = my_signal_handler };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);

    // read in the command line options
    state->gopt = getopt_create ();
    getopt_add_description (state->gopt, "Navigator: Estimates Segway pose during mission");
    getopt_add_bool (state->gopt,    'D', "daemon",                0, "Run as system daemon");
    getopt_add_bool (state->gopt,    'h', "help",                  0, "Display Help");

    if (!getopt_parse (state->gopt, argc, argv, 1))
    {
        getopt_do_usage (state->gopt, "");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (state->gopt, "help"))
    {
        getopt_do_usage (state->gopt, "");
        exit (EXIT_SUCCESS);
    }

    // fork as daemon?
    if (getopt_get_bool (state->gopt, "daemon"))
    {
        daemon_fork ();
        state->is_daemon = 1;
    }

    // connect to LCM
    state->lcm = lcm_create (NULL);
    if (!state->lcm)
    {
        printf ("lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    }

    // Initialize from config
    // Setup observation models and process models
    // Install LCM callbacks
    init_param ();

    // hook onto 50 hz heartbeat for publishing
    perllcm_heartbeat_t_subscribe (state->lcm, state->hb50_channel, &perllcm_heartbeat_t_cb, state);
    // subscriptions for current control
    perllcm_segway_state_t_subscribe (state->lcm, SEGWAY_STATE_CHANNEL, &perllcm_segway_state_t_cb, state);
    senlcm_kvh_dsp3000_t_subscribe (state->lcm, state->kvh_channel, &senlcm_kvh_dsp3000_t_cb, state);

#if 0
    // setup initialization lcm callbacks
    char *lcm_channel = NULL, *key = NULL;

    key = g_strconcat (INIT_SENSORKEY_RPH, ".gsd.channel", NULL);
    lcm_channel = bot_param_get_str_or_fail (state->param, key);
    senlcm_ms_gx3_t_subscription_t *sub_ms =
        senlcm_ms_gx3_t_subscribe (state->lcm, lcm_channel, &senlcm_ms_gx3_t_cb, state);
    free (lcm_channel);
    free (key);

    // filter initilization loop
    while (!is_init_data () && !state->done)
    {
        lcm_handle (state->lcm);

        // check to make sure that the omf lcm callbacks aren't running yet,
        // we want to set them up before initilization so that errors in the config file
        // are caught before initilization, but we dont want them to run until after
        // initilization. This check should prevent any adverse effects but the check should
        // happen in the omf lcm callback first
        if (NULL != state->meas_t_next)
        {
            ERROR ("ERROR: OMF LCM Callbacks should not return before filter is initialized");
            free_meas_t (state->meas_t_next);
            state->meas_t_next = NULL;
        }
    }
    printf ("\n");

    state->init_data_ready = 1;
    if (state->done)
    {
        clean_up ();
        exit (EXIT_SUCCESS);
    }

    senlcm_ms_gx3_t_unsubscribe (state->lcm, sub_ms);

    // take init data and set the filter
    init_filter ();
#else

    // start the filter
    state->filter_running = 1;

#endif


    // subscriptions for delayed state augmentation (TODO MOVE RATE TO CONFIG FILE)
    // 8 delayed states at a rate of 20 hz gives us just under half a sec of lag. which we need
    // image syncs by the time the get to us arrive 1/4 of a second late.
    perllcm_heartbeat_t_subscribe (state->lcm, state->hb20_channel, &delayed_state_aug_cb, state);
    perllcm_heartbeat_t_subscribe (state->lcm, state->channel_drop_ds, &drop_ds_cb, state);

    // Main loop
    while (!state->done)
    {
        // see if we receive a heartbeat
        state->utime_last_heartbeat = 0;

        //lcmu_handle_timeout (state->lcm, &tv);
        lcm_handle (state->lcm);

        predict_correct ();
    }

    // clean up
    clean_up ();
    exit (EXIT_SUCCESS);
}
