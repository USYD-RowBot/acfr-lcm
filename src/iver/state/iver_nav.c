#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <glib.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/nmea.h"
#include "perls-common/units.h"

#include "perls-math/fasttrig.h"
#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"
#include "perls-math/ssc.h"

#include "perls-lcmtypes/senlcm_dstar_ssp1_t.h"
#include "perls-lcmtypes/senlcm_ms_gx3_25_t.h"
#include "perls-lcmtypes/senlcm_os_compass_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd4_t.h"
#include "perls-lcmtypes/senlcm_rph_t.h"
#include "perls-lcmtypes/senlcm_raw_t.h"

#include "iver_nav.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

#define RPH_MICROSTRAIN 1
#define RPH_OS_COMPASS  2
#define RPH_SENSOR RPH_MICROSTRAIN

#define DEPTH_DSTAR      1
#define DEPTH_OS_COMPASS 2
#define DEPTH_SENSOR DEPTH_DSTAR

#define RDI_ALTITUDE           1
#define RDI_ALTITUDE_CORRECTED 2
#define ALTITUDE_SENSOR RDI_ALTITUDE_CORRECTED

static void
nav_microstrain_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                          const senlcm_ms_gx3_25_t *ustrain, void *user)
{
    nav_state_t *state = user;

    if (! (ustrain->bitmask & SENLCM_MS_GX3_25_T_EULER)) return;

    if (RPH_SENSOR == RPH_MICROSTRAIN) state->temp = ustrain->Temperature;
}

static void
nav_corr_microstrain_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                               const senlcm_rph_t *ustrain, void *user)
{
    nav_state_t *state = user;

    if (RPH_SENSOR == RPH_MICROSTRAIN)
    {
        state->rph[0] = -ustrain->rph[0];
        state->rph[1] = -ustrain->rph[1];
        state->rph[2] =  ustrain->rph[2] + M_PI;

        char msg[256];
        nmea_sprintf (msg, "$C%.1lfP%.1lfR%.1lfT%.1lfD%.2lf*",
                      state->rph[2] * RTOD,              // deg
                      state->rph[1] * RTOD,              // deg
                      state->rph[0] * RTOD,              // deg
                      state->temp,                       // celsius
                      state->dfs * UNITS_METER_TO_FEET); // feet
        senlcm_raw_t gsd_msg =
        {
            .utime = ustrain->utime,
            .length = strlen (msg),
            .data = (uint8_t *) msg,
        };
        senlcm_raw_t_publish (state->lcm, state->os_conduit_write_chan, &gsd_msg);
    }
}

static void
nav_os_compass_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                         const senlcm_os_compass_t *compass, void *user)
{
    nav_state_t *state = user;
    if (DEPTH_SENSOR == DEPTH_OS_COMPASS) state->dfs = compass->depth;

    if (RPH_SENSOR == RPH_OS_COMPASS) state->temp = compass->T;
}

static void
nav_corr_os_compass_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                              const senlcm_rph_t *compass, void *user)
{
    nav_state_t *state = user;

    if (RPH_SENSOR == RPH_OS_COMPASS)
    {
        state->rph[0] = compass->rph[0];
        state->rph[1] = compass->rph[1];
        state->rph[2] = compass->rph[2];

        char msg[256];
        nmea_sprintf (msg, "$C%.1lfP%.1lfR%.1lfT%.1lfD%.2lf*",
                      state->rph[2] * RTOD,         // deg
                      state->rph[1] * RTOD,         // deg
                      state->rph[0] * RTOD,         // deg
                      state->temp,                  // celsius
                      state->dfs * UNITS_METER_TO_FEET); // feet
        senlcm_raw_t gsd_msg =
        {
            .utime = compass->utime,
            .length = strlen (msg),
            .data = (uint8_t *) msg,
        };
        senlcm_raw_t_publish (state->lcm, state->os_conduit_write_chan, &gsd_msg);
    }
}

static void
nav_dstar_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                    const senlcm_dstar_ssp1_t *dstar, void *user)
{
    nav_state_t *state = user;
    if (DEPTH_SENSOR == DEPTH_DSTAR)
        state->dfs = dstar->depth;
}

static void
nav_rdi_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                  const senlcm_rdi_pd4_t *pd4, void *user)
{
    nav_state_t *state = user;

    static bool firstime = true;
    static GSLU_VECTOR_VIEW (x_vs, 6);
    static GSLU_MATRIX_VIEW (R_vs, 3, 3);
    static GSLU_MATRIX_VIEW (bhat, 3, 4);
    if (firstime)
    {
        firstime = false;
        // static sensor to vehicle coordinate transform
        bot_param_get_double_array_or_fail (state->param, "sensors.rdi.x_vs", x_vs.data, 6);
        x_vs.data[SSC_DOF_R] *= DTOR;
        x_vs.data[SSC_DOF_P] *= DTOR;
        x_vs.data[SSC_DOF_H] *= DTOR;
        so3_rotxyz (R_vs.data, &x_vs.data[SSC_DOF_RPH]);

        // RDI beam geometry
        double s, c;
        fsincos (30.0*DTOR, &s, &c);
        /*  b1, b2, b3, b4 */
        GSLU_MATRIX_VIEW (tmp_bhat, 3, 4, {-s,  s,  0,  0,
                                           0,  0,  s, -s,
                                           -c, -c, -c, -c
                                          });
        gsl_matrix_memcpy (&bhat.matrix, &tmp_bhat.matrix);
    }


    // Altitude
    // $SDDBT,#,f,#,M,#,F<*cc> feet, Meters, Fathoms
    char msg[256];
    if (pd4->altitude > SENLCM_RDI_PD4_T_ALTITUDE_SENTINAL)
    {
        GSLU_MATRIX_VIEW (R_wv, 3, 3);
        so3_rotxyz (R_wv.data, state->rph);

        GSLU_MATRIX_VIEW (R_ws, 3, 3);
        gslu_blas_mm (&R_ws.matrix, &R_wv.matrix, &R_vs.matrix);

        int nbeams = 0;
        double altavg = 0, altmin = GSL_POSINF;
        for (int i=0; i < 4; i++)
        {
            if (pd4->range[i] > 0)
            {
                nbeams++;
                GSLU_VECTOR_VIEW (bi_s, 3);
                gsl_matrix_get_col (&bi_s.vector, &bhat.matrix, i);
                gsl_vector_scale (&bi_s.vector, pd4->range[i]);

                GSLU_VECTOR_VIEW (bi_w, 3);
                gslu_blas_mv (&bi_w.vector, &R_ws.matrix, &bi_s.vector);

                double bi_alt = bi_w.data[2];
                altavg += bi_alt;
                if (bi_alt < altmin)
                    altmin = bi_alt;
            }
        }
        altavg /= nbeams;

        double uvc_altitude = 0;
        if (ALTITUDE_SENSOR == RDI_ALTITUDE)
            uvc_altitude = pd4->altitude;
        else if (ALTITUDE_SENSOR == RDI_ALTITUDE_CORRECTED)
            uvc_altitude = altmin;
        else
            ERROR ("unknown ALTITUDE_SENSOR");

        nmea_sprintf (msg, "$SDDBT,%.1f,f,%.1f,M,%.1f,F*",
                      uvc_altitude * UNITS_METER_TO_FEET,     //feet
                      uvc_altitude,                           //meters
                      uvc_altitude * UNITS_METER_TO_FATHOM);  //fathoms
        senlcm_raw_t gsd_msg =
        {
            .utime = pd4->utime,
            .length = strlen (msg),
            .data = (uint8_t *) msg,
        };
        senlcm_raw_t_publish (state->lcm, state->os_conduit_write_chan, &gsd_msg);
    }

    // Velocity
    // $ODVL,<XSPEED>,<YSPEED>,<TMOUT><*cc>
    if (pd4->btv[0] > SENLCM_RDI_PD4_T_BTV_SENTINAL &&
            pd4->btv[1] > SENLCM_RDI_PD4_T_BTV_SENTINAL &&
            pd4->btv[2] > SENLCM_RDI_PD4_T_BTV_SENTINAL)  // three-beam or better soln
    {

        const int UVC_DVL_TIMEOUT = 5;
        gsl_vector_const_view uvw_s = gsl_vector_const_view_array (pd4->btv, 3);
        GSLU_VECTOR_VIEW (uvw_v, 3);
        gslu_blas_mv (&uvw_v.vector, &R_vs.matrix, &uvw_s.vector);

        // RME: 05/31/2011 - UMBS experiments seem to suggest that UVC is defining body-frame +X fwd, +Y port
        // which is oppposite of our NED frame convention on the vehicle, hence why the -1.0 on v
        nmea_sprintf (msg, "$ODVL,%f,%f,%d*", uvw_v.data[0], -1.0*uvw_v.data[1], UVC_DVL_TIMEOUT);
        senlcm_raw_t gsd_msg =
        {
            .utime = pd4->utime,
            .length = strlen (msg),
            .data = (uint8_t *) msg,
        };
        senlcm_raw_t_publish (state->lcm, state->os_conduit_write_chan, &gsd_msg);
    }
}

static void
nav_gpsd_client_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                          const senlcm_raw_t *gps, void *user)
{
    nav_state_t *state = user;

    /*** PeRL line feed hack for UVC4.7 with gpsd 2.92, 5/31/2011 ***/
    char nmea_str[1024] = {'\0'};
    snprintf (nmea_str, sizeof nmea_str, "%s\r\n", (const char *) gps->data);

    if (0 == strncmp (nmea_str, "$PPSDA", 6))
    {
        printf ("skipping $PPSDA\n");
        return;
    }

    senlcm_raw_t gsd_msg =
    {
        .utime = gps->utime,
        .length = strlen (nmea_str),
        .data = (uint8_t *) nmea_str,
    };
    senlcm_raw_t_publish (state->lcm, state->os_conduit_write_chan, &gsd_msg);
}

nav_state_t*
nav_init (lcm_t *lcm, BotParam *param)
{
    nav_state_t *state = calloc (1, sizeof (nav_state_t));
    state->dfs = GSL_POSINF; // init to pos infty, so that vehicle exceeds max depth and aborts if no sensor data
    state->lcm = lcm;
    state->param = param;

    if (botu_param_get_boolean_to_bool (state->param, "iver-state.use_navigator", &state->use_navigator))
    {
        ERROR ("bot param could not find use_navigator field!");
        return NULL;
    }

    char *lcm_chan=NULL, *lcm_raw=NULL;
    // read navigator channels
    if (state->use_navigator)
    {
        ERROR ("we do not currently support missions with navigator!");
        return NULL;
    }

    // read sensor channels
    lcm_chan = bot_param_get_str_or_fail (state->param, "sensors.ms-gx3-25.gsd.channel");
    senlcm_ms_gx3_25_t_subscribe (state->lcm, lcm_chan, &nav_microstrain_callback, state);
    free (lcm_chan);

    lcm_chan = bot_param_get_str_or_fail (state->param, "sensors.ms-gx3-25.rphcorr.channel");
    senlcm_rph_t_subscribe (state->lcm, lcm_chan, &nav_corr_microstrain_callback, state);
    free (lcm_chan);

    lcm_chan = bot_param_get_str_or_fail (state->param, "sensors.os-compass.gsd.channel");
    senlcm_os_compass_t_subscribe (state->lcm, lcm_chan, &nav_os_compass_callback, state);
    free (lcm_chan);

    lcm_chan = bot_param_get_str_or_fail (state->param, "sensors.os-compass.rphcorr.channel");
    senlcm_rph_t_subscribe (state->lcm, lcm_chan, &nav_corr_os_compass_callback, state);
    free (lcm_chan);

    lcm_chan = bot_param_get_str_or_fail (state->param, "sensors.dstar-ssp1.gsd.channel");
    senlcm_dstar_ssp1_t_subscribe (state->lcm, lcm_chan, &nav_dstar_callback, state);
    free (lcm_chan);

    lcm_chan = bot_param_get_str_or_fail (state->param, "sensors.rdi.gsd.channel");
    senlcm_rdi_pd4_t_subscribe (state->lcm, lcm_chan, &nav_rdi_callback, state);
    free (lcm_chan);

    lcm_chan = bot_param_get_str_or_fail (state->param, "sensors.gpsd3-client.gsd.channel");
    lcm_raw = g_strconcat (lcm_chan, ".RAW", NULL);
    senlcm_raw_t_subscribe (state->lcm, lcm_raw, &nav_gpsd_client_callback, state);
    free (lcm_chan);
    free (lcm_raw);

    // publish to uvc channel under gsd (os-conduit instance) hood
    lcm_chan = bot_param_get_str_or_fail (state->param, "os-conduit.gsd.channel");
    state->os_conduit_write_chan = g_strconcat (lcm_chan, ".WRITE", NULL);
    free (lcm_chan);

    return state;
}

void
nav_destroy (nav_state_t *state)
{
    if (!state) return;

    if (state->os_conduit_write_chan) free (state->os_conduit_write_chan);
    free (state);
}



