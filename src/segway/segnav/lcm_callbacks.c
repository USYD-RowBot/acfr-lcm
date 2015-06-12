#include <stdio.h>
#include <stdlib.h>

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/units.h"

#include "lcm_callbacks.h"

//==============================================================================
// HELPER FUNCTIONS
//==============================================================================

//------------------------------------------------------------------------------
// Initialize and allocate a measurement structure
// int meas_len --> the length of the measurment vector
// char *omf_key --> observation model key string in config file
// est_omf_ekf_t omf --> observation model callback function
//------------------------------------------------------------------------------
est_meas_t *
init_alloc_meas_t (state_t *state, int meas_len, char *omf_key,
                   est_omf_ekf_t omf_ekf, est_omf_ukf_t omf_ukf,
                   est_omf_pf_t omf_pf)
{
    char cfg_str_key[256];
    est_meas_t *meas_t = calloc (1, sizeof (*meas_t));
    meas_t->R = gsl_matrix_calloc (meas_len, meas_len);
    // read noise params from config file
    sprintf (cfg_str_key, "navigator.omfs.%s.R", omf_key);
    botu_param_read_covariance (meas_t->R, state->param, cfg_str_key);
    meas_t->z = gsl_vector_calloc (meas_len);
    meas_t->index_map = state->index_t;
    sprintf (meas_t->id_str, "%s", omf_key);

    // read innov mahal params
    double thresh_tmp;
    sprintf (cfg_str_key, "navigator.omfs.%s.innov_mahal_test_thresh", omf_key);
    if (-1 == bot_param_get_double (state->param, cfg_str_key, &thresh_tmp))
    {
        // no threshold set dont use mahal test
        meas_t->use_innov_mahal_test = 0;
        meas_t->innov_mahal_test_thresh = 0;
    }
    else
    {
        meas_t->use_innov_mahal_test = 1;
        meas_t->innov_mahal_test_thresh = thresh_tmp;
    }

    // add omf callback functions
    meas_t->omf_ekf = omf_ekf;
    meas_t->omf_ukf = omf_ukf;
    meas_t->omf_pf = omf_pf;

    return meas_t;
}

//------------------------------------------------------------------------------
// Free a meas_t structure and the vectors / matracies inside
//------------------------------------------------------------------------------
void
free_meas_t (est_meas_t *meas_t)
{
    gsl_matrix_free (meas_t->R);
    gsl_vector_free (meas_t->z);
    free (meas_t->user);
    free (meas_t);
}

//------------------------------------------------------------------------------
// Load x_vs from config file
//------------------------------------------------------------------------------
void
load_x_vs_x_vr (BotParam *param, char *omf_key, double *X_vs, double *X_lr)
{
    //look up sensor we are using for this om
    char cfg_str_key[128];
    sprintf (cfg_str_key, "navigator.omfs.%s.sensor_key", omf_key);
    char *sensor_key;
    if (bot_param_get_str (param, cfg_str_key, &sensor_key))
        ERROR ("ERROR: sensor_key not found for \"%s.\"\n", omf_key);

    sprintf (cfg_str_key, "sensors.%s.x_vs", sensor_key);
    if (6 != bot_param_get_double_array (param, cfg_str_key, X_vs, 6))
        ERROR ("ERROR: x_vs not found for \"sensors.%s.\" Assuming identity.\n", sensor_key);

    //convert to radians
    X_vs[3] = X_vs[3]*UNITS_DEGREE_TO_RADIAN;
    X_vs[4] = X_vs[4]*UNITS_DEGREE_TO_RADIAN;
    X_vs[5] = X_vs[5]*UNITS_DEGREE_TO_RADIAN;

    if (NULL != X_lr)
    {
        sprintf (cfg_str_key, "sensors.%s.x_lr", sensor_key);
        if (6 != bot_param_get_double_array (param, cfg_str_key, X_lr, 6))
            ERROR ("ERROR: x_lr not found for \"sensors.%s.\" Assuming identity.\n", sensor_key);

        //convert to radians
        X_lr[3] = X_lr[3]*UNITS_DEGREE_TO_RADIAN;
        X_lr[4] = X_lr[4]*UNITS_DEGREE_TO_RADIAN;
        X_lr[5] = X_lr[5]*UNITS_DEGREE_TO_RADIAN;
    }

    free (sensor_key);
}


//==============================================================================
// LCM CALLBACKS
//==============================================================================

//------------------------------------------------------------------------------
// microstrain mems attidude sensor (rph)
//------------------------------------------------------------------------------
void
ms_gx3_rph_cb (const lcm_recv_buf_t *rbuf, const char *channel,
               const senlcm_ms_gx3_t *msg, void *user)
{
    state_t *state = user;
    // IMPORTANT, DON'T DO ANYTHING UNTIL STATE SAYS FILTER IS INITIALIZEDD
    if (!state->filter_running) return;

    // make it was an update to what we are interested in
    if (msg->bitmask & SENLCM_MS_GX3_T_STAB_EULER)
    {

        char *omf_key = "OMF_MS_GX3_RPH";
        est_meas_t *meas_t = init_alloc_meas_t (state, 3,
                                                omf_key,
                                                &est_omf_ms_gx1_rph,
                                                NULL, NULL);

        //setup user data structure
        est_omf_ms_gx1_rph_user_t *usr = calloc (1, sizeof (*usr));
        //get vehicle to sensor xfm
        load_x_vs_x_vr (state->param, omf_key, usr->X_vs, usr->X_lr);
        meas_t->user = usr;

        double h_z = msg->sEuler[2];

        // modify heading for magnetic declination
        double mag_decl = 0;
        char cfg_str_key[256];
        sprintf (cfg_str_key, "navigator.omfs.%s.mag_decl", omf_key);
        if (-1 != bot_param_get_double (state->param, cfg_str_key, &mag_decl))
            h_z = h_z - mag_decl;
        else
            printf ("MS DECL COEFS NOT FOUNT\n");

        // polynomial heading correction
        double h_corr_coefs[5] = {0};
        sprintf (cfg_str_key, "navigator.omfs.%s.h_corr_coefs", omf_key);
        if (5 == bot_param_get_double_array (state->param, cfg_str_key, h_corr_coefs, 5))
        {
            // as(1)*(h.^0)+as(2)*(h.^1)+as(3)*(h.^2)+as(4)*(h.^3)+as(5)*(h.^4);
            h_z =   h_corr_coefs[0]
                    + h_corr_coefs[1] * (h_z)
                    + h_corr_coefs[2] * (h_z*h_z)
                    + h_corr_coefs[3] * (h_z*h_z*h_z)
                    + h_corr_coefs[4] * (h_z*h_z*h_z*h_z);
        }

        //update meas_t based on lcm message
        gsl_vector_set (meas_t->z, 0, msg->sEuler[0]);
        gsl_vector_set (meas_t->z, 1, msg->sEuler[1]);
        gsl_vector_set (meas_t->z, 2, h_z);
        meas_t->utime = msg->utime;

        // assign meas_t so it is processed next
        state->meas_t_next = meas_t;
    }
}


//------------------------------------------------------------------------------
// microstrain mems attidude sensor (abc AKA pqr)
//------------------------------------------------------------------------------
void
ms_gx3_abc_cb (const lcm_recv_buf_t *rbuf, const char *channel,
               const senlcm_ms_gx3_t *msg, void *user)
{
    state_t *state = user;

    // IMPORTANT, DON'T DO ANYTHING UNTIL STATE SAYS FILTER IS INITIALIZEDD
    if (!state->filter_running) return;

    // make it was an update to what we are interested in
    if (msg->bitmask & SENLCM_MS_GX3_T_STAB_ANGRATE)
    {
        char *omf_key = "OMF_MS_GX3_ABC";
        est_meas_t *meas_t = init_alloc_meas_t (state, 3,
                                                omf_key,
                                                &est_omf_ms_gx1_abc,
                                                NULL, NULL);

        //setup user data structure
        est_omf_ms_gx1_abc_user_t *usr = calloc (1, sizeof (*usr));
        //get vehicle to sensor xfm
        load_x_vs_x_vr (state->param, omf_key, usr->X_vs, NULL);
        meas_t->user = usr;

        //update meas_t based on lcm message
        gsl_vector_set (meas_t->z, 0, msg->sAngRate[0]);
        gsl_vector_set (meas_t->z, 1, msg->sAngRate[1]);
        gsl_vector_set (meas_t->z, 2, msg->sAngRate[2]);
        meas_t->utime = msg->utime;

        // assign meas_t so it is processed next
        state->meas_t_next = meas_t;
    }
}


//------------------------------------------------------------------------------
// microstrain mems attidude sensor (rph)
//------------------------------------------------------------------------------
void
ms_gx3_rp_cb (const lcm_recv_buf_t *rbuf, const char *channel,
              const senlcm_ms_gx3_t *msg, void *user)
{
    state_t *state = user;
    // IMPORTANT, DON'T DO ANYTHING UNTIL STATE SAYS FILTER IS INITIALIZEDD
    if (!state->filter_running) return;

    // make it was an update to what we are interested in
    if (msg->bitmask & SENLCM_MS_GX3_T_STAB_EULER)
    {

        char *omf_key = "OMF_MS_GX3_RP";
        est_meas_t *meas_t = init_alloc_meas_t (state, 2,
                                                omf_key,
                                                &est_omf_ms_gx1_rp,
                                                NULL, NULL);

        //setup user data structure
        est_omf_ms_gx1_rp_user_t *usr = calloc (1, sizeof (*usr));
        //get vehicle to sensor xfm
        load_x_vs_x_vr (state->param, omf_key, usr->X_vs, usr->X_lr);
        meas_t->user = usr;

        //update meas_t based on lcm message
        gsl_vector_set (meas_t->z, 0, msg->sEuler[0]);
        gsl_vector_set (meas_t->z, 1, msg->sEuler[1]);
        meas_t->utime = msg->utime;

        // assign meas_t so it is processed next
        state->meas_t_next = meas_t;
    }
}


//------------------------------------------------------------------------------
// microstrain mems attidude sensor (abc AKA pqr)
//------------------------------------------------------------------------------
void
ms_gx3_ab_cb (const lcm_recv_buf_t *rbuf, const char *channel,
              const senlcm_ms_gx3_t *msg, void *user)
{
    state_t *state = user;

    // IMPORTANT, DON'T DO ANYTHING UNTIL STATE SAYS FILTER IS INITIALIZEDD
    if (!state->filter_running) return;

    // make it was an update to what we are interested in
    if (msg->bitmask & SENLCM_MS_GX3_T_STAB_ANGRATE)
    {
        char *omf_key = "OMF_MS_GX3_AB";
        est_meas_t *meas_t = init_alloc_meas_t (state, 2,
                                                omf_key,
                                                &est_omf_ms_gx1_ab,
                                                NULL, NULL);

        //setup user data structure
        est_omf_ms_gx1_ab_user_t *usr = calloc (1, sizeof (*usr));
        //get vehicle to sensor xfm
        load_x_vs_x_vr (state->param, omf_key, usr->X_vs, NULL);
        meas_t->user = usr;

        //update meas_t based on lcm message
        gsl_vector_set (meas_t->z, 0, msg->sAngRate[0]);
        gsl_vector_set (meas_t->z, 1, msg->sAngRate[1]);
        meas_t->utime = msg->utime;

        // assign meas_t so it is processed next
        state->meas_t_next = meas_t;
    }
}

//------------------------------------------------------------------------------
// microstrain mems attidude sensor (rph)
//------------------------------------------------------------------------------
void
ms_gx3_rp_25_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                 const senlcm_ms_gx3_25_t *msg, void *user)
{
    state_t *state = user;
    // IMPORTANT, DON'T DO ANYTHING UNTIL STATE SAYS FILTER IS INITIALIZEDD
    if (!state->filter_running) return;

    // make it was an update to what we are interested in
    if (msg->bitmask & SENLCM_MS_GX3_25_T_EULER)
    {

        char *omf_key = "OMF_MS_GX3_25_RP";
        est_meas_t *meas_t = init_alloc_meas_t (state, 2,
                                                omf_key,
                                                &est_omf_ms_gx1_rp,
                                                NULL, NULL);

        //setup user data structure
        est_omf_ms_gx1_rp_user_t *usr = calloc (1, sizeof (*usr));
        //get vehicle to sensor xfm
        load_x_vs_x_vr (state->param, omf_key, usr->X_vs, usr->X_lr);
        meas_t->user = usr;

        //update meas_t based on lcm message
        gsl_vector_set (meas_t->z, 0, msg->Euler[0]);
        gsl_vector_set (meas_t->z, 1, msg->Euler[1]);
        meas_t->utime = msg->utime;

        // assign meas_t so it is processed next
        state->meas_t_next = meas_t;
    }
}


//------------------------------------------------------------------------------
// microstrain mems attidude sensor (abc AKA pqr)
//------------------------------------------------------------------------------
void
ms_gx3_ab_25_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                 const senlcm_ms_gx3_25_t *msg, void *user)
{
    state_t *state = user;

    // IMPORTANT, DON'T DO ANYTHING UNTIL STATE SAYS FILTER IS INITIALIZEDD
    if (!state->filter_running) return;

    // make it was an update to what we are interested in
    if (msg->bitmask & SENLCM_MS_GX3_25_T_STAB_ANGRATE
            || msg->bitmask & SENLCM_MS_GX3_25_T_INST_ANGRATE)
    {
        char *omf_key = "OMF_MS_GX3_25_AB";
        est_meas_t *meas_t = init_alloc_meas_t (state, 2,
                                                omf_key,
                                                &est_omf_ms_gx1_ab,
                                                NULL, NULL);

        //setup user data structure
        est_omf_ms_gx1_ab_user_t *usr = calloc (1, sizeof (*usr));
        //get vehicle to sensor xfm
        load_x_vs_x_vr (state->param, omf_key, usr->X_vs, NULL);
        meas_t->user = usr;

        //update meas_t based on lcm message
        gsl_vector_set (meas_t->z, 0, msg->AngRate[0]);
        gsl_vector_set (meas_t->z, 1, msg->AngRate[1]);
        meas_t->utime = msg->utime;

        // assign meas_t so it is processed next
        state->meas_t_next = meas_t;
    }
}

