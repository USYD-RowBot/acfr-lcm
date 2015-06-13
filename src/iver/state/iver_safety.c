#include <stdlib.h>

#include "iver_safety.h"

#include "perls-common/lcm_util.h"

#include "perls-lcmtypes/senlcm_dstar_ssp1_t.h"
#include "perls-lcmtypes/senlcm_ms_gx3_25_t.h"
#include "perls-lcmtypes/senlcm_os_compass_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd4_t.h"

#include "perls-lcmtypes/perllcm_auv_safety_rules_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

#define SAFETY_BUFFER_QUEUE_TIME      2000000 // us
#define SAFETY_DEPTH_CONSENSUS        PERLLCM_AUV_SAFETY_RULES_T_SAFETY_DEPTH_CONSENSUS
#define SAFETY_DEPTH_TIME_CONSENSUS   PERLLCM_AUV_SAFETY_RULES_T_SAFETY_DEPTH_TIME_CONSENSUS
#define SAFETY_HEADING_CONSENSUS      PERLLCM_AUV_SAFETY_RULES_T_SAFETY_HEADING_CONSENSUS
#define SAFETY_HEADING_TIME_CONSENSUS PERLLCM_AUV_SAFETY_RULES_T_SAFETY_HEADING_TIME_CONSENSUS

#define SAFETY_MAX_DEPTH_DIFF   5         // meters
#define SAFETY_MAX_HEADING_DIFF (25*DTOR) // radians
#define SAFETY_MAX_TIME_DIFF    2000000    // us

static void
safety_microstrain_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                             const senlcm_ms_gx3_25_t *ustrain, void *user)
{
    safety_t *state = user;
    state->microstrain.age = ustrain->utime - state->microstrain.last_utime;
    state->microstrain.last_utime = ustrain->utime;
}

static void
safety_os_compass_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                            const senlcm_os_compass_t *compass, void *user)
{
    safety_t *state = user;
    state->os_compass.age = compass->utime - state->os_compass.last_utime;
    state->os_compass.last_utime = compass->utime;
}

static void
safety_dstar_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                       const senlcm_dstar_ssp1_t *dstar, void *user)
{
    safety_t *state = user;
    state->dstar.age = dstar->utime - state->dstar.last_utime;
    state->dstar.last_utime = dstar->utime;
}

static void
safety_rdi_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                     const senlcm_rdi_pd4_t *pd4, void *user)
{
    safety_t *state = user;
    state->rdi.age = pd4->utime - state->rdi.last_utime;
    state->rdi.last_utime = pd4->utime;
}

static void
safety_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                 const perllcm_heartbeat_t *beat, void *user)
{
    safety_t *state = user;

    //evaluate safety rules and publish result

    perllcm_auv_safety_rules_t msg =
    {
        .utime = beat->utime,
        .safety_rules_active = state->safety_rules_active,
        .safety_rules_violated = state->safety_rules_violated,
    };
    perllcm_auv_safety_rules_t_publish (state->lcm, state->safety_channel, &msg);
}

safety_t*
safety_init (lcm_t *lcm, BotParam *param)
{
    safety_t *state = calloc (1, sizeof (safety_t));
    state->lcm = lcm;

    char *lcm_chan=NULL, *iver_prefix=NULL;

    state->safety_rules_active = 0;
    state->safety_rules_active |= SAFETY_DEPTH_CONSENSUS *
                                  bot_param_get_boolean_or_fail (param, "safety-rules.depth_consensus");
    state->safety_rules_active |= SAFETY_DEPTH_TIME_CONSENSUS *
                                  bot_param_get_boolean_or_fail (param, "safety-rules.depth_time_consensus");
    state->safety_rules_active |= SAFETY_HEADING_CONSENSUS *
                                  bot_param_get_boolean_or_fail (param, "safety-rules.heading_consensus");
    state->safety_rules_active |= SAFETY_HEADING_TIME_CONSENSUS *
                                  bot_param_get_boolean_or_fail (param, "safety-rules.heading_time_consensus");

    state->safety_rules_violated = 0;

    state->safety_channel = bot_param_get_str_or_fail (param, "safety-rules.safety_channel");

    lcm_chan = bot_param_get_str_or_fail (param, "sensors.ms-gx3-25.gsd.channel");
    senlcm_ms_gx3_25_t_subscribe (state->lcm, lcm_chan, &safety_microstrain_callback, state);
    free (lcm_chan);

    lcm_chan = bot_param_get_str_or_fail (param, "sensors.os-compass.gsd.channel");
    senlcm_os_compass_t_subscribe (state->lcm, lcm_chan, &safety_os_compass_callback, state);
    free (lcm_chan);

    lcm_chan = bot_param_get_str_or_fail (param, "sensors.dstar-ssp1.gsd.channel");
    senlcm_dstar_ssp1_t_subscribe (state->lcm, lcm_chan, &safety_dstar_callback, state);
    free (lcm_chan);

    lcm_chan = bot_param_get_str_or_fail (param, "sensors.rdi.gsd.channel");
    senlcm_rdi_pd4_t_subscribe (state->lcm, lcm_chan, &safety_rdi_callback, state);
    free (lcm_chan);

    iver_prefix = bot_param_get_str_or_fail (param, "vehicle.lcm_channel_prefix");
    lcm_chan = lcmu_channel_get_heartbeat (iver_prefix, 1);
    free (iver_prefix);
    perllcm_heartbeat_t_subscribe (state->lcm, lcm_chan, &safety_callback, state);
    free (lcm_chan);

    return state;
}

void
safety_destroy (safety_t *state)
{
    if (!state) return;

    if (state->safety_channel) free (state->safety_channel);
    free (state);
}

