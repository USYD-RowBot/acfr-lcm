#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include <glib.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/units.h"

#include "perls-math/fasttrig.h"
#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"
#include "perls-math/ssc.h"

// Input lcm defs from sensors
#include "perls-lcmtypes/senlcm_dstar_ssp1_t.h"
#include "perls-lcmtypes/senlcm_ms_gx3_25_t.h"
#include "perls-lcmtypes/senlcm_ms_gx1_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd4_t.h"
#include "perls-lcmtypes/senlcm_tritech_es_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

// Output lcm defs
#include "perls-lcmtypes/perllcm_auv_es_control_t.h"
#include "perls-lcmtypes/perllcm_auv_es_horizon_t.h"

#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)

// Using coordinate system NED measurement for ES position taken from DVL
#define ES_X              0.290 //m
#define ES_Z              -0.060 //m
#define ES_ANGLE          6*DTOR  //rad
#define ES_RANGE          50 // m
#define CLEAR_ANGLE       2*DTOR  //rad
#define SIZE_BIN          1.0 //m
#define NUM_BINS          ES_RANGE/SIZE_BIN
#define TURN_ANGLE        (state->max_pitch -  5)*DTOR  // rad
#define GREEN_ANGLE       (state->max_pitch - 10)*DTOR // rad
#define YELLOW_ANGLE      (state->max_pitch - 15)*DTOR // rad
#define TURN_SLOPE        tan(TURN_ANGLE) // non-dim
#define GREEN_SLOPE       tan(GREEN_ANGLE) // non-dim
#define YELLOW_SLOPE      tan(YELLOW_ANGLE) // non-dim
#define MAX_SLOPE         tan(state->max_pitch*DTOR) //non-dim
#define WARN_ALT          fmax((state->goal_alt-1), state->min_alt) // m

typedef struct _state_t state_t;
struct _state_t
{
    // LCM Outputs
    perllcm_auv_es_control_t control;
    perllcm_auv_es_horizon_t horizon;
    char *prefix;
    char *control_channel;
    char *horizon_channel;
    lcm_t *lcm;

    // LCM Inputs
    int64_t utime;
    uint8_t hb;
    double  depth;
    uint8_t dstar_new;
    double  rph[3];
    int64_t ms_utime;
    uint8_t ms_new;
    double  dx_c;
    double  dx_o;
    double  alt;
    int64_t rdi_utime;
    uint8_t rdi_new_o;
    uint8_t rdi_new_c;
    double  range;
    uint8_t es_new_h;
    uint8_t es_new_c;

    // Misc
    int is_daemon;
    BotParam *param;       //config file handle
    getopt_t *gopt;         //getopt

    int nBins;
    double goal_alt;
    double min_alt;
    double control_hz;
    double max_pitch;
    double peek_dx;
    double timeout;
    double es_angle_offset;
    int    cleared;
    int    done;
};

int done = 0;

static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("my_signal_handler()\n");
    if (done)
    {
        printf ("Goodbye\n");
        exit (EXIT_FAILURE);
    }
    else
        printf("Quitting!\n");
    done = 1;
}


static void
heartbeat_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                    const perllcm_heartbeat_t *hb, void *user)
{
    state_t *state = user;

    state->utime = hb->utime;
    state->hb = 1;
} // heartbeat_callback


static void
desert_star_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                      const senlcm_dstar_ssp1_t *dstar, void *user)
{
    state_t *state = user;

    state->depth = dstar->depth;
    state->dstar_new   = 1;
} // desert_star_callback


static void
ms_gx3_25_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                    const senlcm_ms_gx3_25_t *ustrain, void *user)
{
    state_t *state = user;

    // only send fresh Euler data
    if (! (ustrain->bitmask & SENLCM_MS_GX3_25_T_EULER))
        return;
    if (ustrain->utime > state->ms_utime)
    {
        state->rph[0]   = -ustrain->Euler[0];
        state->rph[1]   = -ustrain->Euler[1];
        state->rph[2]   =  ustrain->Euler[2] + M_PI;
        state->ms_utime =  ustrain->utime;
        state->ms_new   =  1;
    }
} // microstrain_callback

static void
ms_gx1_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                 const senlcm_ms_gx1_t *ustrain, void *user)
{
    state_t *state = user;

    // only send fresh Euler data
    if (! (ustrain->bitmask & SENLCM_MS_GX1_T_STAB_EULER))
        return;
    if (ustrain->utime > state->ms_utime)
    {
        state->rph[0]   = -ustrain->sEuler[0];
        state->rph[1]   = -ustrain->sEuler[1];
        state->rph[2]   =  ustrain->sEuler[2] + M_PI;
        state->ms_utime =  ustrain->utime;
        state->ms_new   =  1;
    }
}

static void
rdi_pd4_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                  const senlcm_rdi_pd4_t *pd4, void *user)
{
    state_t *state = user;

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
    // three-beam or better soln
    if (pd4->altitude > SENLCM_RDI_PD4_T_ALTITUDE_SENTINAL &&
            pd4->btv[0] > SENLCM_RDI_PD4_T_BTV_SENTINAL &&
            pd4->btv[1] > SENLCM_RDI_PD4_T_BTV_SENTINAL &&
            pd4->btv[2] > SENLCM_RDI_PD4_T_BTV_SENTINAL)
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
        // Velocity
        // $ODVL,<XSPEED>,<YSPEED>,<TMOUT><*cc>
        gsl_vector_const_view uvw_s = gsl_vector_const_view_array (pd4->btv, 3);
        GSLU_VECTOR_VIEW (uvw_v, 3);
        gslu_blas_mv (&uvw_v.vector, &R_vs.matrix, &uvw_s.vector);

        // RME: 05/31/2011 - UMBS experiments seem to suggest that UVC is defining body-frame +X fwd, +Y port
        // which is oppposite of our NED frame convention on the vehicle, hence why the -1.0 on v

        double dt = (pd4->utime - state->rdi_utime)/1E6;
        double u  = uvw_v.data[0];
        state->dx_c  += u*dt;
        state->dx_o  += u*dt;
        state->alt   = altavg;
        state->rdi_utime = pd4->utime;
        state->rdi_new_o = 1;
        state->rdi_new_c = 1;
    } // if
} // rdi_pd4_callback

static void
tritech_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                  const senlcm_tritech_es_t *tritech, void *user)
{
    state_t *state = user;

    state->range  = tritech->range;
    state->es_new_h = 1;
    state->es_new_c = 1;

} // tritech_callback


void
clear_control(state_t * state)
{
    state->control.alt_mod       = 0;
    state->control.alt_mod_flag  = 0;

    state->control.prev_wp_count = 0;
    state->control.prev_wp       = 0;
    state->control.danger_flag   = 0;
    state->control.danger_lat    = 0;
    state->control.danger_long   = 0;
    state->control.danger_depth  = 0;

    state->control.abort_count   = 0;
    state->control.abort         = 0;

    state->control.dx_up         = 0;
    state->control.dx_down       = 0;

    state->control.timeout       = 1;

    state->control.utime         = state->utime;

    perllcm_auv_es_control_t_publish (state->lcm, state->control_channel, &(state->control));
    state->dstar_new = 0;
    state->es_new_c  = 0;
    state->ms_new    = 0;
    state->rdi_new_c = 0;
} // clear_control

void
update_control(state_t * state)
{
    state->dstar_new             = 0;
    state->es_new_c              = 0;
    state->ms_new                = 0;
    state->rdi_new_c             = 0;
    state->control.timeout       = 0;
    state->control.abort         = 0;
    state->control.prev_wp       = 0;
    state->control.alt_mod_flag  = 0;

    if (state->depth < 0.5)
        return;

    state->control.utime         = state->utime;

    // Check to see if looking up too long
    if ((state->rph[1] > TURN_ANGLE) &&
            (state->control.alt_mod > 0.1))
        state->control.dx_up += state->dx_c;
    else if ((state->rph[1] < ES_ANGLE/2) ||
             (state->control.alt_mod < 0.1))
        state->control.dx_up = 0;

    if (state->rph[1] < -TURN_ANGLE)
        state->control.dx_down += state->dx_c;
    else if (state->rph[1] > -ES_ANGLE/2)
        state->control.dx_down = 0;

    state->dx_c = 0;
    if ((state->control.dx_up > state->peek_dx) &&
            ((state->horizon.index > 10) || (state->horizon.index < 1)))
    {
        state->control.alt_mod = state->alt - (state->goal_alt + 2);
        state->control.alt_mod_flag = 1;
        perllcm_auv_es_control_t_publish (state->lcm, state->control_channel, &(state->control));
        return;
    }
    else if (state->control.dx_down > state->peek_dx)
    {
        state->control.alt_mod = state->alt - 0.5;
        state->control.alt_mod_flag = 1;
        perllcm_auv_es_control_t_publish (state->lcm, state->control_channel, &(state->control));
        return;
    }
    else
    {
        if (state->horizon.flag_slope > TURN_SLOPE)
            state->control.prev_wp_count += 1;
        else
            state->control.prev_wp_count = 0;

        if (state->horizon.flag_slope < TURN_SLOPE)
            state->control.abort_count = 0;
        else if (state->horizon.flag_slope > MAX_SLOPE)
            state->control.abort_count += 1;

        if ((state->horizon.flag_slope > MAX_SLOPE) &&
                (((state->horizon.index < 6) &&
                  (state->control.abort_count > 3)) ||
                 ((state->control.abort_count > 5) &&
                  (state->horizon.index < 15))))
        {
            state->control.abort = 1;
            printf("Attempting to Abort!\n");
            state->control.prev_wp_count = 0;
            state->control.abort_count   = 0;
            state->control.dx_up         = 0;
            state->control.dx_down       = 0;
            perllcm_auv_es_control_t_publish (state->lcm, state->control_channel, &(state->control));
            return;
        }
        else if (state->control.prev_wp_count > 5 &&
                 state->horizon.index < 15)
        {
            state->control.prev_wp = 1;
            printf("Attempting to Turn Around!\n");
            // INSERT THE control.danger_*
            state->control.danger_flag = 1;
            // state->control.danger_lat    = 0;
            // state->control.danger_long   = 0;
            state->control.danger_depth  = state->horizon.depth[state->horizon.index];
            state->control.prev_wp_count = 0;
            state->control.abort_count   = 0;
            state->control.dx_up         = 0;
            state->control.dx_down       = 0;
        }
    }

    if (state->horizon.index > 0)
    {
        double dist = state->horizon.index*SIZE_BIN + state->horizon.offset;
        double pseudo_bottom = state->depth + state->alt;
        if ((state->horizon.health[state->horizon.index] > 1.1) &&
                (state->horizon.slope > GREEN_SLOPE))
        {
            pseudo_bottom =  state->horizon.depth[state->horizon.index] +
                             GREEN_SLOPE*dist;
        }
        else if ((state->horizon.health[state->horizon.index] < 1.1) &&
                 (state->horizon.slope > YELLOW_SLOPE))
        {
            pseudo_bottom = state->horizon.depth[state->horizon.index] +
                            YELLOW_SLOPE*dist;
        }
        if ((pseudo_bottom < (state->depth + fmin(state->goal_alt, state->alt))) ||
                (state->control.danger_flag > 0.1))
        {
            if (state->control.danger_flag > 0.1)
            {
                state->control.alt_mod =
                    (state->control.danger_depth + fmin(state->goal_alt, state->alt)) -
                    pseudo_bottom;
            }
            else
            {
                state->control.alt_mod = (state->depth + fmin(state->goal_alt, state->alt)) -
                                         pseudo_bottom;
            }
            state->control.alt_mod_flag = 1;
        }
        else
        {
            state->control.alt_mod      = 0;
            state->control.alt_mod_flag = 0;
        }
    }
    perllcm_auv_es_control_t_publish (state->lcm, state->control_channel, &(state->control));
}

void
update_horizon(state_t * state)
{
    double pitch = state->rph[1];  //rad
    double range = state->range;   //m


    if (range < 0.3)
        range = 50;
    double es_depth = state->depth - ES_X*sin(pitch) - ES_Z*cos(pitch);
    double es_x     = ES_X*cos(pitch) - ES_Z*sin(pitch);
    int ii = 0;
    double ii_range = ii*SIZE_BIN + state->horizon.offset;
    // Area cleared by beam
    while ((ii_range < ((range - SIZE_BIN)*cos(pitch)))  &&
            (ii < state->nBins-1))
    {
        state->horizon.depth[ii] = fmax(state->horizon.depth[ii],
                                        es_depth - ii_range*sin(pitch - CLEAR_ANGLE));
        if (state->horizon.health[ii] >= 0.9)
            state->horizon.health[ii] = 2;
        else if (state->horizon.health[ii] < 0.9)
            state->horizon.health[ii] = 1;
        ii++;
        ii_range = ii*SIZE_BIN + state->horizon.offset;
    } // while

    double temp_shal = fmax(0, es_depth - ii_range*sin(pitch + ES_ANGLE/2));
    double temp_deep = fmax(0, es_depth - ii_range*sin(pitch - ES_ANGLE/2));

    // Last cleared grid
    if ((temp_deep < state->horizon.depth[ii]) &&
            (state->horizon.health[ii] > 0.1))
    {
        state->horizon.depth[ii]  = temp_deep;
        state->horizon.health[ii] = 1;
    }
    else if ((temp_deep > state->horizon.depth[ii]) &&
             ((temp_shal <  state->horizon.depth[ii]) ||
              (state->horizon.health[ii] == 1)))
    {
        state->horizon.depth[ii]  = temp_shal;
        state->horizon.health[ii] = 1;
    }
    else if (ii < state->nBins-1)
    {
        state->horizon.depth[ii]  = temp_shal;
        state->horizon.health[ii] = 1;
    }
    else
        state->horizon.depth[ii]  = temp_shal;

    if (range > 5)
        state->horizon.health[ii] = 0;

    // Find the slope to horizon from the warning alt
    state->horizon.flag_slope = -1;
    state->horizon.index = 0;
    double rise, run, temp_slope;
    for (int jj = 0; jj < state->nBins; jj++)
    {
        if ((state->horizon.health[jj] > 0.1) && (jj > state->cleared)
                && (state->horizon.depth[jj] > 0.1))
        {
            rise = (state->depth + WARN_ALT) - state->horizon.depth[jj];
            run = es_x + state->horizon.offset + jj*SIZE_BIN;
            temp_slope = rise/run;
            if (temp_slope > state->horizon.flag_slope)
            {
                state->horizon.flag_slope = temp_slope;
                state->horizon.index = jj;
            }
        }
    } // for

    state->horizon.slope = -1;
    if (state->horizon.index > state->cleared)
    {
        rise = (state->depth + fmin(state->goal_alt, state->alt)) -
               state->horizon.depth[state->horizon.index];
        run = es_x + state->horizon.offset + state->horizon.index*SIZE_BIN;
        state->horizon.slope = rise/run;
    } // if
    state->horizon.utime = state->utime;
    perllcm_auv_es_horizon_t_publish(state->lcm, state->horizon_channel, &(state->horizon));
    state->es_new_h = 0;
} // update_horizon


void
update_offset(state_t * state)
{
    state->horizon.offset = state->horizon.offset - state->dx_o;
    while (state->horizon.offset < 0)
    {
        state->horizon.offset += 1;
        for (int i=0; i < state->nBins-1; i++)
        {
            state->horizon.depth[i]  = state->horizon.depth[i+1];
            state->horizon.health[i] = state->horizon.health[i+1];
        } // for
        state->horizon.depth[state->nBins-1]  = 0;
        state->horizon.health[state->nBins-1] = 0;
    } // while

    while (state->horizon.offset > 1)
    {
        state->horizon.offset  -= 1;
        for (int i=state->nBins-1; i>0; i--)
        {
            state->horizon.depth[i]  = state->horizon.depth[i-1];
            state->horizon.health[i] = state->horizon.health[i-1];
        } // for
        state->horizon.depth[0]  = 0;
        state->horizon.health[0] = 0;
    } // whil
    state->dx_o      = 0;
    state->rdi_new_o = 0;
} // update_offset


void my_opts (state_t * state, int argc, char *argv[])
{
    // Read in command line options
    state->gopt = getopt_create ();
    getopt_add_description (state->gopt, "TriTech EchoSounder Obstacle Avoidance Algorithm.");
    getopt_add_bool        (state->gopt, 'D',  "daemon", 0, "Run as system daemon");
    getopt_add_double      (state->gopt, 'g',  "goalAlt", "3", "Needs to be set to the goal alittude off bottom for the misson [m]");
    getopt_add_double      (state->gopt, 'm',  "minAlt", "1", "The lowest alittude off bottom the vehicle is allowed [m]");
    getopt_add_double      (state->gopt, '\0', "esAngleOffset", "0", "Mounting angle of EchoSounder wtr y-axis  [deg]");
    getopt_add_double      (state->gopt, '\0', "controlHz", "2", "Maximum update rate of the control algorithm [Hz]");
    getopt_add_double      (state->gopt, '\0', "maxPitch", "25", "Needs to be set to vehicles max allowable pitch [deg]");
    getopt_add_double      (state->gopt, '\0', "peekDx", "15", "Sets distance vehicle will travel prior to looking forward [m]");
    getopt_add_double      (state->gopt, '\0', "timeout", "5", "Discard the control output if data is too old [sec]");
    getopt_add_bool        (state->gopt, 'h',  "help", 0, "Display Help");
    botu_param_add_pserver_to_getopt (state->gopt);

    // Run command line options
    if (!getopt_parse (state->gopt, argc, argv, 1))
    {
        getopt_do_usage (state->gopt, "mission-file");
        exit (EXIT_FAILURE);
    } // if
    else if (getopt_get_bool (state->gopt, "help"))
    {
        getopt_do_usage (state->gopt, "mission-file");
        exit (EXIT_SUCCESS);
    } // else if

    // Fork as daemon
    if (getopt_get_bool (state->gopt, "daemon"))
        daemon_fork ();

    // read options
    state->goal_alt        = getopt_get_double(state->gopt, "goalAlt");
    state->min_alt         = getopt_get_double(state->gopt, "minAlt");
    state->es_angle_offset = getopt_get_double(state->gopt, "esAngleOffset");
    state->control_hz      = getopt_get_double(state->gopt, "controlHz");
    state->max_pitch       = getopt_get_double(state->gopt, "maxPitch");
    state->peek_dx         = getopt_get_double(state->gopt, "peekDx");
    state->timeout         = getopt_get_double(state->gopt, "timeout");
} // my_opts


void my_lcm(state_t * state)
{
    // Connect to LCM
    state->lcm = lcm_create (NULL);

    if (!state->lcm)
    {
        printf ("lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    } // if

    state->param = botu_param_new_from_getopt_or_fail (state->gopt, state->lcm);
    if (!state->param)
        exit (EXIT_FAILURE);
    state->prefix = bot_param_get_str_or_fail (state->param, "vehicle.lcm_channel_prefix");

    // LCM Subscpitions
    char *lcm_chan=NULL;
    lcm_chan = lcmu_channel_get_heartbeat (state->prefix, 10);
    perllcm_heartbeat_t_subscribe (state->lcm, lcm_chan, &heartbeat_callback, state);
//    free (lcm_chan);

    char *lcm_key=NULL;
    lcm_key = "sensors.dstar-ssp1.gsd.channel";
    lcm_chan = bot_param_get_str_or_fail (state->param, lcm_key);
    senlcm_dstar_ssp1_t_subscribe (state->lcm, lcm_chan, &desert_star_callback, state);
//    free (lcm_chan); free (lcm_key);

    lcm_key = "sensors.rdi.gsd.channel";
    lcm_chan = bot_param_get_str_or_fail (state->param, lcm_key);
    senlcm_rdi_pd4_t_subscribe (state->lcm, lcm_chan, &rdi_pd4_callback, state);
//    free (lcm_chan); free (lcm_key);

    lcm_key = "sensors.ms-gx3-25.gsd.channel";
    if (bot_param_has_key(state->param, lcm_key))
    {
        lcm_chan = bot_param_get_str_or_fail(state->param, lcm_key);
        senlcm_ms_gx3_25_t_subscribe (state->lcm, lcm_chan, &ms_gx3_25_callback, state);
//        free (lcm_chan);
    }
//    free (lcm_key);

    lcm_key = "sensors.ms-gx1.gsd.channel";
    if (bot_param_has_key(state->param, lcm_key))
    {
        lcm_chan = bot_param_get_str_or_fail(state->param, lcm_key);
        senlcm_ms_gx1_t_subscribe (state->lcm, lcm_chan, &ms_gx1_callback, state);
//        free (lcm_chan);
    }
//    free (lcm_key);


    lcm_key = "sensors.tritech-es.gsd.channel";
    lcm_chan = bot_param_get_str_or_fail (state->param, lcm_key);
    senlcm_tritech_es_t_subscribe (state->lcm, lcm_chan, &tritech_callback, state);
//    free (lcm_chan); free (lcm_key);

    // LCM Outputs
    lcm_key = "es-control.control.channel";
    if (bot_param_has_key(state->param, lcm_key))
        state->control_channel = bot_param_get_str_or_fail (state->param, lcm_key);
    else
        state->control_channel = "IVER31_ES_CONTROL";
//    free (lcm_key);

    lcm_key = "es-control.horizon.channel";
    if (bot_param_has_key(state->param, lcm_key))
        state->horizon_channel = bot_param_get_str_or_fail (state->param, lcm_key);
    else
        state->horizon_channel = "IVER31_ES_HORIZON";
//    free (lcm_key); free(lcm_chan);

} // my_lcm


void init_state(state_t * state)
{
    // Init part of state
    state->hb = 0;
    state->dstar_new = 0;
    state->es_new_h  = 0;
    state->es_new_c  = 0;
    state->ms_new    = 0;
    state->rdi_new_o = 0;
    state->rdi_new_c = 0;
    state->nBins = (int)NUM_BINS;
    state->control.prev_wp_count = 0;
    state->control.prev_wp       = 0;
    state->control.danger_flag   = 0;
    state->control.danger_lat    = 0;
    state->control.danger_long   = 0;
    state->control.danger_depth  = 0;
    state->control.abort_count   = 0;
    state->control.dx_up         = 0;
    state->control.dx_down       = 0;
    for (int i=0; i<state->nBins; i++)
    {
        state->horizon.depth[i]  = 0;
        state->horizon.health[i] = 0;
    } // for
    state->horizon.offset = 0;
} // init_state


int main (int argc, char *argv[])
{
    state_t *state = calloc (1, sizeof (*state));

    // install custom signal handler
    struct sigaction act = { .sa_sigaction = my_signal_handler };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);

    my_opts(state, argc, argv);
    my_lcm(state);
    init_state(state);
    fasttrig_init ();
    int64_t utime, last_utime;

    // wait for sensors to send data
    while ((!state->dstar_new == 1 || !state->es_new_c == 1 ||
            !state->ms_new == 1  || !state->rdi_new_c == 1 ||
            !state->hb == 1) && !done)
    {
        lcm_handle (state->lcm);
        last_utime = state->utime;
    }
    state->rdi_utime = state->utime;
    state->dx_o = 0;
    state->dx_c = 0;
    state->cleared  = fmax(0, floor(WARN_ALT/(MAX_SLOPE*SIZE_BIN)));
    // main loop
    while (!done)
    {
        utime = state->utime;
        lcm_handle (state->lcm);

        if (state->rdi_new_o ==1)
            update_offset(state);

        if (state->es_new_h == 1)
            update_horizon(state);

        if (state->dstar_new == 1 && state->es_new_c == 1 &&
                state->rdi_new_c == 1  &&
                ((utime - last_utime)/1e6 > (1/state->control_hz)))
        {
            update_control(state);
            last_utime = utime;
        } // if

        if ((state->control.utime - last_utime)/1e6 > state->timeout)
        {
            printf ("Sensors Timedout\n");
            clear_control(state);
            last_utime = utime;
        } // if
    } // while
    printf ("Cleaning up\n");
    getopt_destroy (state->gopt);
    bot_param_destroy (state->param);
    lcm_destroy (state->lcm);
    free (state);
    printf ("done\n");

    exit (EXIT_SUCCESS);
} // main
