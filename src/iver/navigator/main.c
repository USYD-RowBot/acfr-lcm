#include <stdio.h>
#include <stdlib.h>

#include <bot_core/bot_core.h>
#include <glib.h>

// Input lcmdefs from sensors
#include "perls-lcmtypes/senlcm_dstar_ssp1_t.h"
#include "perls-lcmtypes/senlcm_gpsd_t.h"
#include "perls-lcmtypes/senlcm_kvh_dsp3000_t.h"
#include "perls-lcmtypes/senlcm_ms_gx1_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd4_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/perllcm_auv_mission_status_t.h"

// Output lcmdefs
#include "perls-lcmtypes/perllcm_est_navigator_index_t.h"
#include "perls-lcmtypes/perllcm_auv_navigator_t.h"
#include "perls-lcmtypes/perllcm_est_navigator_debug_meas_t.h"
#include "perls-lcmtypes/perllcm_est_navigator_debug_pred_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/units.h"

#include "perls-math/gsl_util.h"
#include "perls-math/fasttrig.h"

#include "perls-est/est.h"

#include "lcm_callbacks.h"
#include "navigator.h"


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
    if (0==strcmp (key, "PMF_CONST_VEL_BODY"))
    {
        // if we start adding a ton of PMF we can move these to a speperate file

        // set callback
        state->pred_t->pmf_ekf = est_pmf_const_vel_body;
        state->pred_t->pmf_ukf = NULL;
        state->pred_t->pmf_pf = NULL;

        // get indexing stucture
        est_pmf_const_vel_body_get_index_t (state->index_t);
        // allocate user data structure
        state->pred_t->user = est_pmf_const_vel_body_alloc_user_t ();
        // save a pointer to the free function
        state->pred_user_t_free = &est_pmf_const_vel_body_free_user_t;
        // fill the user structure
        est_pmf_const_vel_body_user_t *usr = state->pred_t->user;
        // continous noise (converted into dt noise in pm)

        botu_param_read_covariance (usr->Qv, state->param, "navigator.pmf.Qv");

        return 0;
    }
    else if (0==strcmp (key, "PMF_CONST_VEL_WORLD"))
        return -1;
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
    if (0==strcmp (key, "OMF_DSTAR_SSP1_Z"))
    {
        senlcm_dstar_ssp1_t_subscribe (state->lcm, lcm_chan, &dstar_ssp1_z_cb, state);
        return 0;
    }
    else if (0==strcmp (key, "OMF_RDI_PD4_UVW"))
    {
        senlcm_rdi_pd4_t_subscribe (state->lcm, lcm_chan, &rdi_pd4_uvw_cb, state);
        return 0;
    }
    else if (0==strcmp (key, "OMF_MS_GX1_RPH"))
    {
        senlcm_ms_gx1_t_subscribe (state->lcm, lcm_chan, &ms_gx1_rph_cb, state);
        return 0;
    }
    else if (0==strcmp (key, "OMF_MS_GX1_ABC"))
    {
        senlcm_ms_gx1_t_subscribe (state->lcm, lcm_chan, &ms_gx1_abc_cb, state);
        return 0;
    }
    else if (0==strcmp (key, "OMF_KVH_DSP3000_C"))
    {
        senlcm_kvh_dsp3000_t_subscribe (state->lcm, lcm_chan, &kvh_dsp3000_c_cb, state);
        return 0;
    }
    else if (0==strcmp (key, "OMF_GPSD_XY"))
    {
        senlcm_gpsd_t_subscribe (state->lcm, lcm_chan, &gpsd_xy_cb, state);
        return 0;
    }
    else if (0==strcmp (key, "OMF_GPSD3_XY"))
    {
        senlcm_gpsd3_t_subscribe (state->lcm, lcm_chan, &gpsd3_xy_cb, state);
        return 0;
    }
    else if (0==strcmp (key, "OMF_OSI_MOTOR_COUNT_U"))
    {
        senlcm_uvc_osi_t_subscribe (state->lcm, lcm_chan, &osi_motor_count_u_cb, state);
        return 0;
    }
    else
        return -1;
}

//------------------------------------------------------------------------------
// init filter from config file
//------------------------------------------------------------------------------
static void
init_param ()
{
    printf ("Initializing navigator from config file.\n");

    fasttrig_init ();

    state->meas_t_next = NULL;
    state->utime_last_heartbeat = 0;
    state->init_data_ready = 0;
    state->filter_running = 0;

    // parse the config file
    state->param = botu_param_new_from_getopt_or_fail (state->gopt, state->lcm);
    if (!state->param)
        exit(EXIT_FAILURE);

    // parse the config file
    state->debug = 0;
    bot_param_get_int (state->param, "navigator.est.debug", &state->debug);

    state->max_pm_dt = 100000;
    bot_param_get_int (state->param, "navigator.est.max_pm_dt", &state->max_pm_dt);

    state->est_engine = "EKF";
    bot_param_get_str (state->param, "navigator.est.est_engine", &state->est_engine);

    // parse lcm channel names
    state->prefix = bot_param_get_str_or_fail (state->param, "vehicle.lcm_channel_prefix");
    state->channel = bot_param_get_str_or_fail (state->param, "navigator.channel");
    state->channel_debug_meas = bot_param_get_str_or_fail (state->param, "navigator.channel_debug_meas");
    state->channel_debug_pred = bot_param_get_str_or_fail (state->param, "navigator.channel_debug_pred");

    state->hb1_channel = lcmu_channel_get_heartbeat (state->prefix, 1);
    state->hb5_channel = lcmu_channel_get_heartbeat (state->prefix, 5);
    state->hb10_channel = lcmu_channel_get_heartbeat (state->prefix, 10);

    double ll_deg[2];
    ll_deg[0] = bot_param_get_double_or_fail (state->param, "navigator.est.org_lat");
    ll_deg[1] = bot_param_get_double_or_fail (state->param, "navigator.est.org_lon");
    state->llxy = calloc(1, sizeof (*state->llxy));
    bot_gps_linearize_init (state->llxy, ll_deg);

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
    state->pred_t->Q = gsl_matrix_calloc (state_len,state_len);

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

    bot_param_get_double_array (state->param, "navigator.est.ini_mu", ini_mu, state_len);
    gsl_vector *ini_mu_tmp = gsl_vector_calloc (state_len);
    memcpy (ini_mu_tmp->data, ini_mu, state_len*sizeof(double));
    gsl_matrix *ini_Sigma_tmp = gsl_matrix_calloc (state_len, state_len);
    botu_param_read_covariance (ini_Sigma_tmp, state->param, "navigator.est.ini_Sigma");


    // instantiate estimator
    if (0==strcmp ("EKF", state->est_engine))
        state->estimator = est_init_ekf (state_len, ini_mu_tmp, ini_Sigma_tmp);
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

    if (state->param) bot_param_destroy (state->param);
    if (state->llxy) free (state->llxy);
    if (state->index_t) free (state->index_t);

    if (state->pred_t)
    {
        if (state->pred_t->user)
            (*state->pred_user_t_free) (state->pred_t->user);
        free (state->pred_t);
    }

    if (state->estimator) est_free (state->estimator);
    if (state->lcm)  lcm_destroy (state->lcm);

    printf ("done\n");
}

//------------------------------------------------------------------------------
// After initialization data has been collected use it to set the mean of the filter
//------------------------------------------------------------------------------
static void
init_filter (void)
{
    // set initial state for filter
    gsl_vector_set (state->estimator->mu, state->index_t->x,
                    gslu_stats_median_array (state->init_xy[0], 1, INIT_SAMPLES_XY));
    gsl_vector_set (state->estimator->mu, state->index_t->y,
                    gslu_stats_median_array (state->init_xy[1], 1, INIT_SAMPLES_XY));
    gsl_vector_set (state->estimator->mu, state->index_t->z,
                    gslu_stats_median_array (state->init_z, 1, INIT_SAMPLES_Z));
    gsl_vector_set (state->estimator->mu, state->index_t->r,
                    gslu_stats_median_array (state->init_rph[0], 1, INIT_SAMPLES_RPH));
    gsl_vector_set (state->estimator->mu, state->index_t->p,
                    gslu_stats_median_array (state->init_rph[1], 1, INIT_SAMPLES_RPH));
    gsl_vector_set (state->estimator->mu, state->index_t->h,
                    gslu_stats_median_array (state->init_rph[2], 1, INIT_SAMPLES_RPH));
    gsl_vector_set (state->estimator->mu, state->index_t->u,
                    gslu_stats_median_array (state->init_uvw[0], 1, INIT_SAMPLES_UVW));
    gsl_vector_set (state->estimator->mu, state->index_t->v,
                    gslu_stats_median_array (state->init_uvw[1], 1, INIT_SAMPLES_UVW));
    gsl_vector_set (state->estimator->mu, state->index_t->w,
                    gslu_stats_median_array (state->init_uvw[2], 1, INIT_SAMPLES_UVW));
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
// GPS callback for initing filter
//------------------------------------------------------------------------------
static void
senlcm_gpsd_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                  const senlcm_gpsd_t *msg, void *user)
{
    state_t *state = user;

    // make sure GPS is reporting that it is in a mode to produce lat lon
    if (msg->mode >= 2)
    {
        double ll_deg[2] = { UNITS_RADIAN_TO_DEGREE * msg->latitude,
                             UNITS_RADIAN_TO_DEGREE * msg->longitude
                           };
        double yx_tmp[2] = {0};
        int ii = state->init_xy_cnt % INIT_SAMPLES_XY;

        // NOTE returns yx -> returns in ENU not NED
        bot_gps_linearize_to_xy (state->llxy, ll_deg, yx_tmp);
        state->init_xy[0][ii] = yx_tmp[1];
        state->init_xy[1][ii] = yx_tmp[0];

        state->utime_last_update = msg->utime;
        state->init_xy_cnt++;
    }
}

static void
senlcm_gpsd3_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                   const senlcm_gpsd3_t *msg, void *user)
{
    state_t *state = user;

    // make sure GPS is reporting that it is in a mode to produce lat lon
    if (msg->fix.mode >= 2)
    {
        double ll_deg[2] = { UNITS_RADIAN_TO_DEGREE * msg->fix.latitude,
                             UNITS_RADIAN_TO_DEGREE * msg->fix.longitude
                           };
        double yx_tmp[2] = {0};
        int ii = state->init_xy_cnt % INIT_SAMPLES_XY;

        // NOTE returns yx -> returns in ENU not NED
        bot_gps_linearize_to_xy (state->llxy, ll_deg, yx_tmp);
        state->init_xy[0][ii] = yx_tmp[1];
        state->init_xy[1][ii] = yx_tmp[0];

        state->utime_last_update = msg->utime;
        state->init_xy_cnt++;
    }
}

//------------------------------------------------------------------------------
// Desert star callback for initing filter
//------------------------------------------------------------------------------
static void
senlcm_dstar_ssp1_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                        const senlcm_dstar_ssp1_t *msg, void *user)
{
    state_t *state = user;

    state->init_z[state->init_z_cnt % INIT_SAMPLES_Z] = msg->depth;
    state->utime_last_update = msg->utime;
    state->init_z_cnt++;
}

//------------------------------------------------------------------------------
// Microstrain callback for initing filter
//------------------------------------------------------------------------------
static void
senlcm_ms_gx1_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                    const senlcm_ms_gx1_t *msg, void *user)
{
    state_t *state = user;

    // only look at updates to stabilized euler angles
    if (msg->bitmask & SENLCM_MS_GX1_T_STAB_EULER)
    {
        int ii = state->init_rph_cnt % INIT_SAMPLES_RPH;
        state->init_rph[0][ii] = -msg->sEuler[0];
        state->init_rph[1][ii] = -msg->sEuler[1];
        state->init_rph[2][ii] =  msg->sEuler[2] + M_PI;

        //init the last update time to the most msg utime
        state->utime_last_update = msg->utime;
        state->init_rph_cnt++;

    }
    else if (msg->bitmask & SENLCM_MS_GX1_T_STAB_ANGRATE)
    {
        int ii = state->init_abc_cnt % INIT_SAMPLES_ABC;
        state->init_abc[0][ii] = -msg->sAngRate[0];
        state->init_abc[1][ii] = -msg->sAngRate[1];
        state->init_abc[2][ii] =  msg->sAngRate[2];

        //init the last update time to the most msg utime
        state->utime_last_update = msg->utime;
        state->init_abc_cnt++;
    }
}

//------------------------------------------------------------------------------
// rdi callback for initing filter
//------------------------------------------------------------------------------
static void
senlcm_rdi_pd4_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                     const senlcm_rdi_pd4_t *msg, void *user)
{
    state_t *state = user;

    // check for bad data based on sentinal
    if (msg->btv[0] > SENLCM_RDI_PD4_T_BTV_SENTINAL &&
            msg->btv[1] > SENLCM_RDI_PD4_T_BTV_SENTINAL &&
            msg->btv[2] > SENLCM_RDI_PD4_T_BTV_SENTINAL)
    {

        int ii = state->init_uvw_cnt % INIT_SAMPLES_UVW;
        // TODO NEED TO ROTATE INTO THE CORRECT FRAME
        // HACK FOR NOW ROTATE NOT IN CONFIG FILE
        GSLU_MATRIX_VIEW (R_sv, 3, 3, {-0.7071, 0.7071, 0.0000,
                                       0.7071, 0.7071, 0.0000,
                                       0.0000, 0.0000,-1.0000
                                      });
        GSLU_VECTOR_VIEW (uvw_s, 3, {0});
        gsl_vector_set (&uvw_s.vector, 0, msg->btv[0]);
        gsl_vector_set (&uvw_s.vector, 1, msg->btv[1]);
        gsl_vector_set (&uvw_s.vector, 2, msg->btv[2]);
        gsl_vector *uvw_v = gslu_blas_mv_alloc (&R_sv.matrix, &uvw_s.vector);
        state->init_uvw[0][ii] = gsl_vector_get (uvw_v, 0);
        state->init_uvw[1][ii] = gsl_vector_get (uvw_v, 1);;
        state->init_uvw[2][ii] = gsl_vector_get (uvw_v, 2);;

        //init the last update time to the most msg utime
        state->utime_last_update = msg->utime;
        state->init_uvw_cnt++;
    }
}

//------------------------------------------------------------------------------
// os-remotehelm callback
//------------------------------------------------------------------------------
static void
perllcm_auv_mission_status_t_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const perllcm_auv_mission_status_t *msg, void *user)
{
    state_t *state = user;

    state->os_remotehelm_waiting_to_start = msg->waiting_to_start;
    state->os_remotehelm_mission_running = msg->mission_running;
}

//------------------------------------------------------------------------------
// Check if we have all the data we need to init filter
//------------------------------------------------------------------------------
static int
is_init_data (void)
{
    if (state->init_xy_cnt  >= INIT_SAMPLES_XY &&
            state->init_z_cnt   >= INIT_SAMPLES_Z &&
            state->init_rph_cnt >= INIT_SAMPLES_RPH &&
            state->init_uvw_cnt >= INIT_SAMPLES_UVW &&
            state->init_abc_cnt >= INIT_SAMPLES_ABC
            //&& state->mission_running // wait for os_remote helm to start mission
       )
    {
        state->init_data_ready = 1;
        return 1;
    }
    else
    {
        //printf ("INIT MEAS: xy->%d/%d, z->%d/%d, rph->%d/%d, uvw=%d/%d. abc %d/%d \r",
        //        state->init_xy_cnt,  INIT_SAMPLES_XY,
        //        state->init_z_cnt,   INIT_SAMPLES_Z,
        //        state->init_rph_cnt, INIT_SAMPLES_RPH,
        //        state->init_uvw_cnt, INIT_SAMPLES_UVW,
        //        state->init_abc_cnt, INIT_SAMPLES_ABC);
        return 0;
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

    perllcm_auv_navigator_t *nav_out = calloc (1, sizeof (*nav_out));

    nav_out->utime = state->utime_last_update;
    nav_out->est_method = state->estimator->est_method;
    nav_out->init_data_ready = state->init_data_ready;
    nav_out->filter_running = state->filter_running;

    // current estimator state
    if (state->filter_running)
    {
        int sl = state->estimator->state_len;
        nav_out->state_len = sl;
        nav_out->mu_len = sl;
        nav_out->Sigma_len = sl*sl;
        nav_out->mu = calloc (sl, sizeof (*nav_out->mu));
        nav_out->Sigma = calloc (sl*sl, sizeof (*nav_out->Sigma));
        memcpy (nav_out->mu, state->estimator->mu->data, sizeof(nav_out->mu)*sl);
        memcpy (nav_out->Sigma, state->estimator->Sigma->data, sizeof(nav_out->Sigma)*sl*sl);
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

    // TODO world frame pose for plotting and other stuff

    // conversion of curent mean [x y] to [lat lon]
    double xy[2] = {gsl_vector_get(state->estimator->mu, state->index_t->x),
                    gsl_vector_get(state->estimator->mu, state->index_t->y)
                   };
    double ll_deg_tmp[2] = {0};
    bot_gps_linearize_to_lat_lon (state->llxy, xy, ll_deg_tmp);
    nav_out->mu_latitude = ll_deg_tmp[0] * UNITS_DEGREE_TO_RADIAN;
    nav_out->mu_longitude = ll_deg_tmp[1] * UNITS_DEGREE_TO_RADIAN;
    nav_out->org_latitude = state->llxy->lat0_deg * UNITS_DEGREE_TO_RADIAN;
    nav_out->org_longitude = state->llxy->lon0_deg * UNITS_DEGREE_TO_RADIAN;

    // publish the message
    perllcm_auv_navigator_t_publish (state->lcm, state->channel, nav_out);

    // free allocated arrays before nav_out goes out of scope
    free (nav_out->mu);
    free (nav_out->Sigma);
    free (nav_out);

    // test order of LCM packets
    // printf("DT test = %f\n", (msg->utime - state->test_last_utime)/1e6);
    // state->test_last_utime = msg->utime;
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

    //print meas debug lcm chan name i.e. IVER28_NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_RPH
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
// predict and correct function called in main loop
// -----------------------------------------------------------------------------
static void
predict_correct (void)
{
    int have_meas = (NULL != state->meas_t_next);
    int have_heartbeat = (0 != state->utime_last_heartbeat);

    if (have_meas && have_heartbeat)
        ERROR ("ERROR: have_meas && have_heartbeat. Should not happen");

    // calculate dt
    if (have_meas)
        state->pred_t->dt = (state->meas_t_next->utime - state->utime_last_update)/1e6;
    else if (have_heartbeat)
        state->pred_t->dt = (state->utime_last_heartbeat - state->utime_last_update)/1e6;
    else // not a heartbeat or a measurement nothing else to do
        return;

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
        est_predict (state->estimator, state->pred_t);
        // update time predicted till
        if (have_meas)
            state->utime_last_update = state->meas_t_next->utime;
        else if (have_heartbeat)
            state->utime_last_update = state->utime_last_heartbeat;

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
// MAIN
//------------------------------------------------------------------------------
int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    state = calloc (1, sizeof (*state));

    // install custom signal handler
    struct sigaction act = { .sa_sigaction = my_signal_handler };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);

    // read in the command line options
    state->gopt = getopt_create ();
    getopt_add_description (state->gopt, "Navigator: Estimates AUV pose during mission");
    getopt_add_bool (state->gopt,    'D', "daemon",                0, "Run as system daemon");
    getopt_add_bool (state->gopt,    'i', "ignore-os-remotehelm",  0, "Don't wait for os-remotehelm to start navigator");
    getopt_add_bool (state->gopt,    'h', "help",                  0, "Display Help");
    botu_param_add_pserver_to_getopt (state->gopt);

    if (!getopt_parse (state->gopt, argc, argv, 1))
    {
        getopt_do_usage (state->gopt, "mission-file");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (state->gopt, "help"))
    {
        getopt_do_usage (state->gopt, "mission-file");
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

    // hook onto 10 hz heartbeat for publishing
    perllcm_heartbeat_t_subscribe (state->lcm, state->hb10_channel, &perllcm_heartbeat_t_cb, state);

    // setup initialization lcm callbacks
    char *lcm_channel = NULL, *key = NULL;

    // OLD GPS INIT
    // key = g_strconcat (INIT_SENSORKEY_XY_OLD, ".gsd.channel", NULL);
    // lcm_channel = bot_param_get_str_or_fail (state->param, key);
    // senlcm_gpsd_t_subscription_t *sub_gpsd =
    //             senlcm_gpsd_t_subscribe (state->lcm, lcm_channel, &senlcm_gpsd_t_cb, state);
    // free (lcm_channel);
    // free (key);

    key = g_strconcat (INIT_SENSORKEY_XY, ".gsd.channel", NULL);
    lcm_channel = bot_param_get_str_or_fail (state->param, key);
    senlcm_gpsd3_t_subscription_t *sub_gpsd3 =
        senlcm_gpsd3_t_subscribe (state->lcm, lcm_channel, &senlcm_gpsd3_t_cb, state);
    free (lcm_channel);
    free (key);

    key = g_strconcat (INIT_SENSORKEY_Z, ".gsd.channel", NULL);
    lcm_channel = bot_param_get_str_or_fail (state->param, key);
    senlcm_dstar_ssp1_t_subscription_t *sub_dstar =
        senlcm_dstar_ssp1_t_subscribe (state->lcm, lcm_channel, &senlcm_dstar_ssp1_t_cb, state);
    free (lcm_channel);
    free (key);

    key = g_strconcat (INIT_SENSORKEY_RPH, ".gsd.channel", NULL);
    lcm_channel = bot_param_get_str_or_fail (state->param, key);
    senlcm_ms_gx1_t_subscription_t *sub_ms =
        senlcm_ms_gx1_t_subscribe (state->lcm, lcm_channel, &senlcm_ms_gx1_t_cb, state);
    free (lcm_channel);
    free (key);

    key = g_strconcat (INIT_SENSORKEY_UVW, ".gsd.channel", NULL);
    lcm_channel = bot_param_get_str_or_fail (state->param, key);
    senlcm_rdi_pd4_t_subscription_t *sub_rdi =
        senlcm_rdi_pd4_t_subscribe (state->lcm, lcm_channel, &senlcm_rdi_pd4_t_cb, state);
    free (lcm_channel);
    free (key);

    lcm_channel = bot_param_get_str_or_fail (state->param, "os-remotehelm.mission_channel");
    perllcm_auv_mission_status_t_subscribe (state->lcm, lcm_channel, &perllcm_auv_mission_status_t_cb, state);
    free (lcm_channel);

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

    // wait for os-remotehelm to start filtering
    if (!getopt_get_bool (state->gopt, "ignore-os-remotehelm"))
    {
        printf ("Waiting for os-remotehelm ...\n");

        while (!state->os_remotehelm_waiting_to_start       // os-remotehelm is not waiting for us
                && !state->os_remotehelm_mission_running     // nor has os-remotehelm already started (can happen when ignore navigator flag set)
                && !state->done)                             // nor have we received a ctrl+c
        {
            lcm_handle (state->lcm);

            if (NULL != state->meas_t_next)
            {
                ERROR ("ERROR: OMF LCM Callbacks should not return before filter is initialized");
                free_meas_t (state->meas_t_next);
                state->meas_t_next = NULL;
            }
        }

        if (state->done)
        {
            clean_up ();
            exit (EXIT_SUCCESS);
        }
    }
    printf ("os-remotehelm mission started.\n");

    // remove initilization lcm callbacks
    // senlcm_gpsd_t_unsubscribe (state->lcm, sub_gpsd);
    senlcm_gpsd3_t_unsubscribe (state->lcm, sub_gpsd3);
    senlcm_dstar_ssp1_t_unsubscribe (state->lcm, sub_dstar);
    senlcm_ms_gx1_t_unsubscribe (state->lcm, sub_ms);
    senlcm_rdi_pd4_t_unsubscribe (state->lcm, sub_rdi);

    // take init data and set the filter
    init_filter ();

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
