#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include <bot_param/param_client.h>
#include <lcm/lcm.h>

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/lcm_util.h"
#include "perls-common/units.h"

#include "perls-lcmtypes/perllcm_auv_iver_state_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/senlcm_uvc_osi_t.h"
#include "perls-lcmtypes/senlcm_uvc_rphtd_t.h"

#include "iver_safety.h"
#include "iver_nav.h"

typedef struct
{
    lcm_t *lcm;

    BotGPSLinearize *bot_llxy;

    char *iver_channel;
    perllcm_auv_iver_state_t *iver_state;
    getopt_t *gopt;
} iver_t;

void
osi_callback (const lcm_recv_buf_t *rbuf, const char *channel,
              const senlcm_uvc_osi_t *osi, void *user)
{
    iver_t *iver = user;
    iver->iver_state->utime = osi->utime;
    iver->iver_state->position.utime = osi->utime;

    double yx[2] = {0};
    double latlon[2] = {osi->latitude * UNITS_RADIAN_TO_DEGREE, osi->longitude * UNITS_RADIAN_TO_DEGREE};
    bot_gps_linearize_to_xy (iver->bot_llxy, latlon, yx);
    iver->iver_state->position.xyzrph[0] = yx[1];
    iver->iver_state->position.xyzrph[1] = yx[0];
    iver->iver_state->orglat = iver->bot_llxy->lat0_deg * UNITS_DEGREE_TO_RADIAN;
    iver->iver_state->orglon = iver->bot_llxy->lon0_deg * UNITS_DEGREE_TO_RADIAN;

    iver->iver_state->altitude = osi->altimeter;
}

void
rphtd_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                const senlcm_uvc_rphtd_t *rphtd, void *user)
{
    iver_t *iver = user;
    iver->iver_state->utime = rphtd->utime;
    iver->iver_state->position.utime = rphtd->utime;
    iver->iver_state->position.xyzrph[2] = rphtd->depth;  //z
    iver->iver_state->position.xyzrph[3] = rphtd->rph[0]; //r
    iver->iver_state->position.xyzrph[4] = rphtd->rph[1]; //p
    iver->iver_state->position.xyzrph[5] = rphtd->rph[2]; //h
}

void
beat_callback (const lcm_recv_buf_t *rbuf, const char *channel,
               const perllcm_heartbeat_t *beat, void *user)
{
    iver_t *iver = user;
    printf ("publishing state, utime = %"PRId64"\n", beat->utime);
    perllcm_auv_iver_state_t_publish (iver->lcm, iver->iver_channel, iver->iver_state);
}

int
main (int argc, char *argv[])
{

    iver_t *iver = calloc (1, sizeof (iver_t));
    iver->iver_state = calloc (1, sizeof (perllcm_auv_iver_state_t));
    iver->gopt = getopt_create();

    iver->lcm = lcm_create (NULL);
    if (!iver->lcm)
    {
        ERROR ("lcm_create() failed");
        exit (EXIT_FAILURE);
    }

    /* command line args */
    getopt_add_description (iver->gopt, "Runs iver state");
    getopt_add_bool (iver->gopt, 'h', "help",                0,  "Show this");
    botu_param_add_pserver_to_getopt (iver->gopt);

    if (!getopt_parse (iver->gopt, argc, argv, 1))
    {
        getopt_do_usage (iver->gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (iver->gopt, "help"))
    {
        getopt_do_usage (iver->gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    BotParam *param = botu_param_new_from_getopt_or_fail (iver->gopt, iver->lcm);
    if (!param)
    {
        ERROR ("botu_param_new_from_getopt_or_fail() failed!");
        exit (EXIT_FAILURE);
    }

    iver->iver_channel = bot_param_get_str_or_fail (param, "vehicle.state_channel");

    // initialize safety callbacks
    safety_t *safety = safety_init (iver->lcm, param);
    if (!safety)
    {
        ERROR ("safety rules initialization failed!");
        exit (EXIT_FAILURE);
    }

    // initialize nav callbacks
    nav_state_t *nav = nav_init (iver->lcm, param);
    if (!nav)
    {
        ERROR ("navigation switch failed!");
        exit (EXIT_FAILURE);
    }

    // latch onto iver state (from uvc) channels
    char *osi_chan=NULL, *rphtd_chan=NULL;
    osi_chan = lcmu_channel_get_os_conduit (param, LCMU_CHANNEL_OS_CONDUIT_OSI);
    senlcm_uvc_osi_t_subscribe (iver->lcm, osi_chan, &osi_callback, iver);
    free (osi_chan);

    rphtd_chan = lcmu_channel_get_os_conduit (param, LCMU_CHANNEL_OS_CONDUIT_RPHTD);
    senlcm_uvc_rphtd_t_subscribe (iver->lcm, rphtd_chan, &rphtd_callback, iver);
    free (rphtd_chan);

    // latch onto heartbeat and get publish channel
    int pub_frequency = botu_param_get_int_or_default (param, "iver-state.pub_frequency", 1);

    char *prefix=NULL, *hb_chan=NULL;
    prefix = bot_param_get_str_or_fail (param, "vehicle.lcm_channel_prefix");
    hb_chan = lcmu_channel_get_heartbeat (prefix, pub_frequency);
    perllcm_heartbeat_t_subscribe (iver->lcm, hb_chan, &beat_callback, iver);
    free (prefix);
    free (hb_chan);

    // orglat/orglon
    double orglatlon[2] = {0};
    bot_param_get_double_array_or_fail (param, "site.orglatlon", orglatlon, 2);
    iver->bot_llxy = calloc (1, sizeof(*(iver->bot_llxy)));
    bot_gps_linearize_init (iver->bot_llxy, orglatlon);

    // lcm loop until done
    while (1)
    {
        lcm_handle (iver->lcm);
    }

    lcm_destroy (iver->lcm);
    nav_destroy (nav);
    safety_destroy (safety);
    exit (EXIT_SUCCESS);
}
