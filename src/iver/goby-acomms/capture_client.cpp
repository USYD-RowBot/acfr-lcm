#include <stdint.h>
#include <iostream>

// external linking req'd
#include <glib.h>

#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/perllcm_acomms_iver_state_t.h"
#include "perls-lcmtypes/senlcm_raw_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"

#include "lcm_listeners.h"

static state_t *state = (state_t*) calloc (1, sizeof (*state));
std::string pub_chan_;

int
startup_failure () 
{
    std::cout << "usage: capture-client" << 
        std::endl;
    return 1;
}

void
heartbeat_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                    const perllcm_heartbeat_t *beat, void *user)
{
    std::cout << "heartbeat called" << std::endl;
    state_t *state = (state_t*) user;

    // pack perllcm_acomms_iver_state_t packet
    perllcm_acomms_iver_state_t msg = {0};
    msg.utime           = timestamp_now ();
    msg.latitude        = state->osi.latitude;
    msg.longitude       = state->osi.longitude;
    msg.heading         = state->heading;
    msg.depth           = state->depth;
    msg.altitude        = state->osi.altimeter;
    msg.next_wypnt      = state->osi.nextwp;
    msg.dist_nwp        = state->osi.dist_to_nextwp;
    msg.batt_percent    = (int) state->opi.percent;
    msg.error           = state->osi.error;       
    msg.aborted_mission = state->aborted_mission;
    
    // publish message
    perllcm_acomms_iver_state_t_publish (state->lcm, pub_chan_.c_str (), &msg);
}

int 
main (int argc, char *argv[]) 
{
    // redirected stdout won't be buffered
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // kickoff subsea/topside lcm
    state->lcm = lcm_create (NULL);
    if (!state->lcm) {
        std::cerr << "lcm_create () failed!" << std::endl;
        exit (EXIT_FAILURE);
    }

    // parse conf parameters
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    std::cout << "opened param file [" << BOTU_PARAM_DEFAULT_CFG << "]" << std::endl;

    // lcm channel names
    state->prefix = bot_param_get_str_or_fail (param, "vehicle.lcm_channel_prefix");
    state->hb_channel = lcmu_channel_get_heartbeat (state->prefix,1);

    pub_chan_ = std::string (state->prefix) + "SELF_STATE";

    state->acomms_abort_channel = lcmu_channel_get_prepost (NULL, state->acomms_channel, "_ABORT");
    state->acomms_jump_channel  = lcmu_channel_get_prepost (NULL, state->acomms_channel, "_JUMP");

    state->rh_mission_channel = botu_param_get_str_or_default (param, "os-remotehelm.mission_channel", "DEFAULT");
    state->osc_channel = botu_param_get_str_or_default (param, "os-conduit.gsd.channel", "DEFAULT");
    state->osc_opi_channel = lcmu_channel_get_prepost (NULL, state->osc_channel, "_OPI");
    state->osc_osi_channel = lcmu_channel_get_prepost (NULL, state->osc_channel, "_OSI");
    state->osc_out_channel = lcmu_channel_get_prepost (NULL, state->osc_channel, ".OUT");

    // lcm callbacks used to pack vehicle state
    perllcm_auv_mission_status_t_subscribe (state->lcm, state->rh_mission_channel, &mission_status_callback, state);
    perllcm_auv_os_conduit_t_subscribe (state->lcm, state->osc_out_channel, &os_conduit_callback, state);
    perllcm_auv_os_conduit_osi_t_subscribe (state->lcm, state->osc_osi_channel, &osi_callback, state);
    perllcm_auv_os_conduit_opi_t_subscribe (state->lcm, state->osc_opi_channel, &opi_callback, state);

    // heartbeat callback -- used to publish acomms_iver_state_t @1Hz
    perllcm_heartbeat_t_subscribe (state->lcm, state->hb_channel, &heartbeat_callback, state);

    // for gmutexs
    state->mutex = g_mutex_new ();

    // run the driver
    std::cout << "time to work ..." << std::endl;
    while (1) {
        lcm_handle (state->lcm);
    }    

    lcm_destroy (state->lcm);
    exit (EXIT_SUCCESS);
}
