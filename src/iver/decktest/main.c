#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>
#include <sys/time.h>

// external linking req'd
#include <math.h>
#include <bot_core/bot_core.h>

#include "perls-lcmtypes/senlcm_dstar_ssp1_t.h"
#include "perls-lcmtypes/senlcm_os_compass_t.h"
#include "perls-lcmtypes/senlcm_uvc_omp_t.h"
#include "perls-lcmtypes/senlcm_uvc_opi_t.h"

#include "perls-lcmtypes/perllcm_heartbeat_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/units.h"

#include "perls-iver/remotehelm.h"

#define FORMAT        "%-25s"
#define FIN_NULL      128
#define MOTOR_NULL    128
#define DURATION      10.0    // seconds

typedef struct _state_t state_t;
struct _state_t
{
    BotParam *param;
    getopt_t *gopt;
    lcm_t *lcm;

    char mode;
    bool done;

    // lcm channels
    char *prefix;

    char *opi_channel;
    char *omp_channel;

    char *os_compass_channel;
    char *dstar_ssp1_channel;

    char *hb1_channel;
    char *hb5_channel;
    char *hb10_channel;
};

static void
omp_zero (senlcm_uvc_omp_t *cmd)
{
    cmd->yaw_top = 128;
    cmd->yaw_bot = 128;
    cmd->pitch_left = 128;
    cmd->pitch_right = 128;

    cmd->motor = 128;
    cmd->timeout = 1;
}

static void
wiggle_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                 const perllcm_heartbeat_t *beat, void *user)
{
    state_t *state = user;
    senlcm_uvc_omp_t cmd;
    omp_zero (&cmd);

    // use acks with iver_rh_pub_omp to test that UVC acks are working
    static int64_t t0 = 0;
    if (t0 == 0)
    {
        t0 = beat->utime;
        struct timeval start_to = {.tv_sec = 2};
        iver_rh_pub_omp (state->lcm, state->omp_channel, &cmd, state->param, &start_to);
        return;
    }

    int64_t utime_dt = beat->utime - t0;
    double dt = timestamp_to_double (utime_dt);

    double freq = 0.4; // Hz
    double sine = sin (2.0*M_PI*freq * dt);
    int motor = dt < DURATION/2.0 ? MOTOR_NULL + 2 : MOTOR_NULL - 2;
    int finpos = FIN_NULL + round (120 * sine);
    int finneg = FIN_NULL - round (120 * sine);

    cmd = (senlcm_uvc_omp_t)
    {
        .yaw_top = finpos,
         .yaw_bot = finneg,
          .pitch_left = finpos,
           .pitch_right = finneg,
            .motor = motor,
    };
    struct timeval cmd_to = {.tv_sec = 2};
    iver_rh_pub_omp (state->lcm, state->omp_channel, &cmd, state->param, &cmd_to);

    static int i = 0;
    if ((i++ % 10)==0)
        printf (".");

    if (dt > DURATION)
    {
        state->done = 1;
        omp_zero (&cmd);
        struct timeval done_to = {.tv_sec = 2};
        iver_rh_pub_omp (state->lcm, state->omp_channel, &cmd, state->param, &done_to);
        printf ("done\n");
    }
}

static void
battery_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                  const senlcm_uvc_opi_t *msg, void *user)
{
    bool *done_batt = user;
    if (*done_batt)
        return;

    if (msg->percent < 30)
        printf ("WARNING battery low!  ");

    switch (msg->batt_state)
    {
    case SENLCM_UVC_OPI_T_BS_CHARGING:
    case SENLCM_UVC_OPI_T_BS_DISCHARGING:
        printf ("OK  %s, ", msg->current > 0 ? "charging" : "discharging");
        break;
    case SENLCM_UVC_OPI_T_BS_FAULT:
        printf ("ERROR battery fault detected, ");
        break;
    default:
        printf ("ERROR unkown state\n");
    }
    printf ("%g%% remaining\n", msg->percent);

    printf (FORMAT, "checking leak:");
    printf ("%s\n",  msg->leak ? "WARNING leak detected!" : "OK  none detected");

    *done_batt = true;
}

static void
vacuum_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                 const senlcm_os_compass_t *msg, void *user)
{
    bool *done_pvolts = user;
    if (*done_pvolts)
        return;

    if (msg->p_volts < 0.34)
        printf ("WARNING p_volts low!  ");
    else if (msg->p_volts > 0.40)
        printf ("WARNING p_volts high!  ");
    else
        printf ("OK  ");
    printf ("p_volts = %.3f  (%.2f psi)\n", msg->p_volts, -1.0*msg->p_meas * UNITS_PASCAL_TO_PSI);
    *done_pvolts = true;
}


static void
temp1_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                const senlcm_os_compass_t *msg, void *user)
{
    senlcm_os_compass_t **os_compass = user;
    if (*os_compass)
        return;
    *os_compass = senlcm_os_compass_t_copy (msg);
}

static void
temp2_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                const senlcm_dstar_ssp1_t *msg, void *user)
{
    senlcm_dstar_ssp1_t **dstar_ssp1 = user;
    if (*dstar_ssp1)
        return;
    *dstar_ssp1 = senlcm_dstar_ssp1_t_copy (msg);
}

static void
check_temperature (state_t *state)
{
    printf (FORMAT, "checking temperatures:");

    senlcm_os_compass_t *os_compass = NULL;
    senlcm_os_compass_t_subscription_t *sub_os_compass =
        senlcm_os_compass_t_subscribe (state->lcm, state->os_compass_channel, &temp1_callback, &os_compass);

    senlcm_dstar_ssp1_t *dstar_ssp1 = NULL;
    senlcm_dstar_ssp1_t_subscription_t *sub_dstar_ssp1 =
        senlcm_dstar_ssp1_t_subscribe (state->lcm, state->dstar_ssp1_channel, &temp2_callback, &dstar_ssp1);

    struct timeval tv = { .tv_sec = 2 };
    while (!os_compass || !dstar_ssp1)
    {
        switch (lcmu_handle_timeout (state->lcm, &tv))
        {
        case -1:
            printf ("ERROR\n");
            exit (EXIT_FAILURE);
        case 0:
            if (!os_compass)
                printf ("TIMEOUT: is os-compass running?\n");
            else if (!dstar_ssp1)
                printf ("TIMEOUT: is dstar running?\n");
            exit (EXIT_FAILURE);
        default:
            break;
        }
    }

    if ((os_compass && os_compass->T > 35) ||
            (dstar_ssp1 && dstar_ssp1->temperature > 35))
        printf ("WARNING, temp high!  ");
    else if ((os_compass && os_compass->T < 0) ||
             (dstar_ssp1 && dstar_ssp1->temperature < 0))
        printf ("WARNING, temp low!  ");
    else
        printf ("OK  ");

    if (os_compass)
        printf ("os_compass = %.2f C, ", os_compass->T);
    if (dstar_ssp1)
        printf ("dstar = %.2f C, ", dstar_ssp1->temperature);
    printf ("\n");

    if (os_compass)
        senlcm_os_compass_t_destroy (os_compass);
    if (dstar_ssp1)
        senlcm_dstar_ssp1_t_destroy (dstar_ssp1);

    senlcm_os_compass_t_unsubscribe (state->lcm, sub_os_compass);
    senlcm_dstar_ssp1_t_unsubscribe (state->lcm, sub_dstar_ssp1);
}


int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    state_t *state = calloc (1, sizeof (*state));

    // TODO: getopts -- just fins, just motor, wiggle, ...
    state->gopt = getopt_create ();
    getopt_add_description (state->gopt, "Decktest utility for the Ivers.");
    getopt_add_help (state->gopt, NULL);
    getopt_add_bool (state->gopt, 'a', "all",    0, "Do all tests.");
    getopt_add_bool (state->gopt, 'm', "modem",  0, "Squawk modem.");
    getopt_add_bool (state->gopt, 's', "strobe", 0, "Flash LED strobe.");
    getopt_add_bool (state->gopt, 'w', "wiggle", 0, "Wiggle prop and fins.");
    getopt_add_example (state->gopt,
                        "Just check battery and vacuum\n"
                        "%s", argv[0]);
    botu_param_add_pserver_to_getopt (state->gopt);

    state->lcm = lcm_create (NULL);
    if (!state->lcm)
    {
        ERROR ("lcm_create() failed!");
        exit (EXIT_FAILURE);
    }

    if (!getopt_parse (state->gopt, argc, argv, 1) || state->gopt->extraargs->len!=0)
    {
        getopt_do_usage (state->gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (state->gopt, "help"))
    {
        getopt_do_usage (state->gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    state->param = botu_param_new_from_getopt_or_fail (state->gopt, state->lcm);

    // lcm channels
    state->opi_channel = lcmu_channel_get_os_conduit (state->param, LCMU_CHANNEL_OS_CONDUIT_OPI);
    state->omp_channel = lcmu_channel_get_os_conduit (state->param, LCMU_CHANNEL_OS_CONDUIT_OMP);

    state->os_compass_channel = bot_param_get_str_or_fail (state->param, "sensors.os-compass.gsd.channel");
    state->dstar_ssp1_channel = bot_param_get_str_or_fail (state->param, "sensors.dstar-ssp1.gsd.channel");

    state->prefix = bot_param_get_str_or_fail (state->param, "vehicle.lcm_channel_prefix");
    state->hb1_channel  = lcmu_channel_get_heartbeat (state->prefix, 1);
    state->hb5_channel  = lcmu_channel_get_heartbeat (state->prefix, 5);
    state->hb10_channel = lcmu_channel_get_heartbeat (state->prefix, 10);

    printf ("\n");

    // check for battery level, and therefore heartbeat, os-conduit, and uvc
    bool done_batt = false;
    senlcm_uvc_opi_t_subscription_t *sub_opi =
        senlcm_uvc_opi_t_subscribe (state->lcm, state->opi_channel, &battery_callback, &done_batt);
    printf (FORMAT, "checking batteries:");
    while (!done_batt)
    {
        struct timeval tv = { .tv_sec = 2 };
        switch (lcmu_handle_timeout (state->lcm, &tv))
        {
        case -1:
            printf ("ERROR\n");
            exit (EXIT_FAILURE);
        case 0:
            printf ("TIMEOUT: are heartbeat, os-conduit, and UVC running?\n");
            exit (EXIT_FAILURE);
        default:
            break;
        }
    }
    senlcm_uvc_opi_t_unsubscribe (state->lcm, sub_opi);

    // check vacuum
    bool done_pvolts = false;
    senlcm_os_compass_t_subscription_t *sub_os_compass =
        senlcm_os_compass_t_subscribe (state->lcm, state->os_compass_channel, &vacuum_callback, &done_pvolts);
    printf (FORMAT, "checking vacuum:");
    while (!done_pvolts)
    {
        struct timeval tv = { .tv_sec = 2 };
        switch (lcmu_handle_timeout (state->lcm, &tv))
        {
        case -1:
            printf ("ERROR\n");
            exit (EXIT_FAILURE);
        case 0:
            printf ("TIMEOUT: is os-compass running?\n");
            exit (EXIT_FAILURE);
        default:
            break;
        }
    }
    senlcm_os_compass_t_unsubscribe (state->lcm, sub_os_compass);

    // check temperature
    check_temperature (state);

    // check modem
    if (getopt_get_bool (state->gopt, "modem") ||
            getopt_get_bool (state->gopt, "all"))
    {
        printf (FORMAT, "checking modem");
        printf ("not implemented yet\n");
    }

    // check led strobe
    if (getopt_get_bool (state->gopt, "strobe") ||
            getopt_get_bool (state->gopt, "all"))
    {
        printf (FORMAT, "checking strobe");
        printf ("not implemented yet\n");
    }

    // check props and fins
    if (getopt_get_bool (state->gopt, "wiggle") ||
            getopt_get_bool (state->gopt, "all"))
    {

        printf (FORMAT, "checking props/fins:");
        perllcm_heartbeat_t_subscribe (state->lcm, state->hb10_channel, &wiggle_callback, state);
        state->done = 0;
        while (!state->done)
            lcm_handle (state->lcm);
    }

    // prompt user to check remaining hardware
    printf ("\n");
    printf ("check that RF beacon, Sonotronics pinger, and NightGear light are on\n");
    printf ("\n");

    // clean up
    lcm_destroy (state->lcm);
    bot_param_destroy (state->param);
    getopt_destroy (state->gopt);
    free (state);
    exit (EXIT_SUCCESS);
}
