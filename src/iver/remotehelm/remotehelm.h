#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdbool.h>

// external linking req'd
#include <math.h>
#include <glib.h>

#include "perls-lcmtypes/perllcm_auv_abort_t.h"
#include "perls-lcmtypes/perllcm_auv_jump_t.h"
#include "perls-lcmtypes/perllcm_auv_mission_status_t.h"
#include "perls-lcmtypes/perllcm_auv_safety_rules_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/perllcm_auv_remotehelm_cmd_t.h"
#include "perls-lcmtypes/perllcm_auv_logger_cmd_t.h"

#include "perls-lcmtypes/senlcm_easydaq_t.h"
#include "perls-lcmtypes/senlcm_uvc_ojw_t.h"
#include "perls-lcmtypes/senlcm_uvc_omload_t.h"
#include "perls-lcmtypes/senlcm_uvc_omstart_t.h"
#include "perls-lcmtypes/senlcm_uvc_omstop_t.h"
#include "perls-lcmtypes/senlcm_uvc_osi_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"
#include "perls-common/units.h"

#include "dir_mgmt.h"

// define state structure
// mission state contains all elements of state that are specific to the mission
// this struct will be cleared out between missions
typedef struct _mission_state_t mission_state_t;
struct _mission_state_t {
    
    int mission_done;
    
    char *mission_file;
    char *mission_file_fixed;
    int mission_running;
    int waiting_to_start;
    int osi_mode;
    int last_osi_mode;
    int osi_park_time_left;
    int start_mission_sleep;

    senlcm_easydaq_t easydaq;
    dir_mgmt_t dir_mgmt;    

    int next_wypnt; //the next waypoint reported by osi
    int num_wypnts; //the total number of waypoints
    GList *wypnt_list;

};


typedef struct _state_t state_t;
struct _state_t {
    
    BotParam *param;
    lcm_t *lcm;
    getopt_t *gopt;
    
    // current command operating on
    perllcm_auv_remotehelm_cmd_t *rh_cmd;

    // node id
    int node_id;

    // lcm channel prefix
    char *prefix;

    // remotehelm channels
    char *rh_mission_channel;
    char *rh_cmd_channel;
    char *rh_abort_channel;
    char *rh_jump_channel;

    // acomms channels
    char *acomms_abort_channel;
    char *acomms_jump_channel;

    // safety rules channel
    char *safety_channel;

    // os-conduit channels
    char *osc_ojw_channel;
    char *osc_osi_channel;
    char *osc_omstart_channel;
    char *osc_omstop_channel;
    char *osc_omload_channel;

    // hearbeat channels
    char *hb1_channel;
    char *hb5_channel;
    char *hb10_channel;
    
    // plcm-logger channel
    char *plcm_channel;

    // misc channels
    char *easydaq_channel;
    
    // all state elements that change with each mission
    mission_state_t *mstate;
};

