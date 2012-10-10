/************************************************************************
 *  backseat-driver/main.c run-mission                                  *
 * 
 ************************************************************************/
#include <sys/time.h>

#include "perls-iver/remotehelm.h"
#include "perls-iver/vectormap.h"

#include "remotehelm.h"
#include "callbacks.h"

#define DTOR UNITS_DEGREE_TO_RADIAN
#define RTOD UNITS_RADIAN_TO_DEGREE

// global g_done for catching
static bool g_done = false;

mission_state_t *
init_mission_state (void) {
    
    mission_state_t *mstate = calloc (1, sizeof (*mstate));
    
    return mstate;
}

void
free_mission_state (mission_state_t *mstate) {
    
    if (!mstate)
        return;

    if (mstate->mission_file) {
        free (mstate->mission_file);
        mstate->mission_file = NULL;
    }
    if (mstate->mission_file_fixed) {
        free (mstate->mission_file_fixed);
        mstate->mission_file_fixed = NULL;
    }
    if (mstate->wypnt_list) {
        iver_vectormap_free_waypoints (mstate->wypnt_list);
        mstate->wypnt_list = NULL;
    }
    free (mstate);
}


state_t *
init_state (int argc, char *argv[]) {
    
    state_t * state =  calloc (1, sizeof (*state));
    state->gopt = getopt_create ();
    state->lcm = lcm_create (NULL);
    if (!state->lcm) {
        printf ("lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    }

    
    // command line options
    getopt_add_description (state->gopt, "Runs Mission Manager");
    getopt_add_bool (state->gopt, 'h', "help",                0,  "Show this");
    botu_param_add_pserver_to_getopt (state->gopt);

    if (!getopt_parse (state->gopt, argc, argv, 1)) {
        getopt_do_usage (state->gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (state->gopt, "help")) {
        getopt_do_usage (state->gopt, NULL);
        exit (EXIT_SUCCESS);
    }    

    state->param = botu_param_new_from_getopt_or_fail (state->gopt, state->lcm);
    if (!state->param)
        exit (EXIT_FAILURE);

    state->rh_cmd = NULL;
    
    // parse node id
    state->node_id = bot_param_get_int_or_fail (state->param, "sensors.modem.id");

    // parse lcm channel names
    state->prefix = bot_param_get_str_or_fail (state->param, "vehicle.lcm_channel_prefix");

    state->rh_abort_channel   = bot_param_get_str_or_fail (state->param, "os-remotehelm.abort_channel");
    state->rh_jump_channel    = bot_param_get_str_or_fail (state->param, "os-remotehelm.jump_channel");
    state->rh_mission_channel = bot_param_get_str_or_fail (state->param, "os-remotehelm.mission_channel");
    state->rh_cmd_channel     = bot_param_get_str_or_fail (state->param, "os-remotehelm.cmd_channel");

    state->safety_channel = bot_param_get_str_or_fail (state->param, "safety-rules.safety_channel");

    char *acomms_channel = bot_param_get_str_or_fail (state->param, "sensors.modem.gsd.channel");
    state->acomms_abort_channel = lcmu_channel_get_prepost (NULL, acomms_channel, "_ABORT");
    state->acomms_jump_channel  = lcmu_channel_get_prepost (NULL, acomms_channel, "_JUMP");
    free (acomms_channel);

    state->osc_ojw_channel = lcmu_channel_get_os_conduit (state->param, LCMU_CHANNEL_OS_CONDUIT_OJW);
    state->osc_osi_channel = lcmu_channel_get_os_conduit (state->param, LCMU_CHANNEL_OS_CONDUIT_OSI);
    state->osc_omstart_channel = lcmu_channel_get_os_conduit (state->param, LCMU_CHANNEL_OS_CONDUIT_OMSTART);
    state->osc_omstop_channel = lcmu_channel_get_os_conduit (state->param, LCMU_CHANNEL_OS_CONDUIT_OMSTOP);
    state->osc_omload_channel = lcmu_channel_get_os_conduit (state->param, LCMU_CHANNEL_OS_CONDUIT_OMLOAD);

    state->hb1_channel  = lcmu_channel_get_heartbeat (state->prefix, 1);
    state->hb5_channel  = lcmu_channel_get_heartbeat (state->prefix, 5);
    state->hb10_channel = lcmu_channel_get_heartbeat (state->prefix, 10);

    state->easydaq_channel = bot_param_get_str_or_fail (state->param, "hotel.easydaq.channel");
    
    state->plcm_channel = bot_param_get_str_or_fail (state->param, "persistent-lcm-logger.cmd_channel");
    
    state->mstate = NULL;
    
    return state;
    
}

void
free_state (state_t *state) {
    
    printf ("Cleaning up state ... ");
    
    bot_param_destroy (state->param);
    getopt_destroy (state->gopt);
    lcm_destroy (state->lcm);
    
    if (NULL != state->rh_cmd)
        perllcm_auv_remotehelm_cmd_t_destroy (state->rh_cmd);
    
    free (state->prefix);
    
    free (state->rh_mission_channel);
    free (state->rh_abort_channel);
    free (state->rh_jump_channel);
    free (state->rh_cmd_channel);
    
    free (state->acomms_abort_channel);
    free (state->acomms_jump_channel);
    
    free (state->osc_ojw_channel);
    free (state->osc_osi_channel);
    free (state->osc_omstart_channel);
    free (state->osc_omstop_channel);
    free (state->osc_omload_channel);
    
    free (state->hb1_channel);
    free (state->hb5_channel);
    free (state->hb10_channel);
    
    free (state->easydaq_channel);
    
    free (state->plcm_channel);
    
    if (NULL != state->mstate) {
        free_mission_state (state->mstate);
        state->mstate = NULL;
    }
    
    free (state);
    
    printf ("done.\n");
}


void
setup_mission (state_t *state, char *mission_file, int start_mission_sleep) {
    
    state->mstate = init_mission_state ();
    mission_state_t *mstate = state->mstate;
    
    // load dir management from config 
    dir_mgmt_load_cfg (state->param, &(mstate->dir_mgmt));
    printf ("Base Dir: %s\n\r", mstate->dir_mgmt.base_dir);
    
    mstate->start_mission_sleep = start_mission_sleep;
    
    // parse the mission file
    printf ("Loading mission file... ");
    mstate->mission_file = strdup (mission_file);
    char *ext = strrchr(mstate->mission_file, '.');
    char *mission_base = strndup (mstate->mission_file, ext - mstate->mission_file);
    mstate->mission_file_fixed = g_strconcat (mission_base, "_UVC.mis", NULL);
    mstate->wypnt_list = iver_vectormap_parse_n_fix_mission (mstate->mission_file,
                                                             mstate->mission_file_fixed);
    if (!mstate->wypnt_list) {
        ERROR ("Failed to load vectormap mission %s", mstate->mission_file);
        exit (EXIT_FAILURE);
    }
    
    // waypoints setup
    //g_list_foreach (state->wypnt_list, (GFunc)&iver_vectormap_print_waypoint, NULL);
    mstate->num_wypnts = g_list_length (mstate->wypnt_list);
    mstate->next_wypnt = 1; //start off at beginning of mission
        
    //find the next dive number and increment the directory
    printf ("Creating log dir for dive... ");
    dir_mgmt_next_dive (&mstate->dir_mgmt);
    printf ("done\n");
    
    // copy the mission file into the dive dirctory (on pc104)
    printf ("Logging mission file, lcmdefs, and config... ");
    dir_mgmt_cpy_mis (&mstate->dir_mgmt, mstate->mission_file);
    dir_mgmt_cpy_mis (&mstate->dir_mgmt, mstate->mission_file_fixed);
    
    // copy the master.cfg and lcm defs into the dive dectory (on pc104)
    dir_mgmt_cpy_config (&mstate->dir_mgmt);
    printf ("done\n");
    
    // init state->easydaq
    for (size_t i=0; i<8; i++) {
        mstate->easydaq.relay[i].label = malloc (256 * sizeof (char));
        mstate->easydaq.relay[i].group = malloc (256 * sizeof (char));
    }
}

int
send_omstart (state_t *state) {
    
    senlcm_uvc_omstart_t *omstart = calloc (1, sizeof (*omstart));
    char wafer_mis_path[512];
    sprintf (wafer_mis_path, "%s\\%s",
             state->mstate->dir_mgmt.wafer_mis_dir,
             strrchr (state->mstate->mission_file_fixed, '/')+1);
    printf ("Wafer Mission Path: %s\r\n", wafer_mis_path);
    
    omstart->mission_name = strdup (wafer_mis_path);
    
    omstart->msg_flag_gps = SENLCM_UVC_OMSTART_T_MSG_FLAG_DO_NOT_IGNORE;
    omstart->msg_flag_sounder = SENLCM_UVC_OMSTART_T_MSG_FLAG_DO_NOT_IGNORE;
    omstart->msg_flag_cal_pressure = SENLCM_UVC_OMSTART_T_MSG_FLAG_DO_NOT_IGNORE;
    omstart->mission_type = SENLCM_UVC_OMSTART_T_MIS_TYPE_NORMAL;
   
    struct timeval to = {.tv_sec = 3};
    int ret = iver_rh_pub_omstart (state->lcm, state->osc_omstart_channel, omstart, state->param, &to);
    senlcm_uvc_omstart_t_destroy (omstart);
    
    return ret;  
}

void
pub_start_logging (state_t *state) {
    
    perllcm_auv_logger_cmd_t cmd = {0};
    cmd.utime = timestamp_now();
    cmd.type = PERLLCM_AUV_LOGGER_CMD_T_START_LOGGING;
    char time_str[14];
    timeutil_strftime (time_str, sizeof time_str, "%Y-%m-%d", timestamp_now ());
    char filename[128] = {0};
    sprintf(filename, "lcmlog-%s-dive.%03d", time_str, state->mstate->dir_mgmt.dive_num);
    cmd.lcmlog_name = g_strconcat (state->mstate->dir_mgmt.dive_dir, "/", filename, NULL);
    cmd.camlog_dir = g_strconcat (state->mstate->dir_mgmt.dive_dir, "/images/", NULL);
    
    perllcm_auv_logger_cmd_t_publish (state->lcm, state->plcm_channel, &cmd);
    free (cmd.lcmlog_name);
    free (cmd.camlog_dir);
}


void
pub_stop_logging (state_t *state) {
    
    perllcm_auv_logger_cmd_t cmd = {0};
    cmd.utime = timestamp_now();
    cmd.type = PERLLCM_AUV_LOGGER_CMD_T_STOP_LOGGING;
    cmd.lcmlog_name = "";
    cmd.camlog_dir = "";
    
    perllcm_auv_logger_cmd_t_publish (state->lcm, state->plcm_channel, &cmd);

}

void
run_mission (state_t *state) {

    // WAIT FOR COMMAND LOOP ---------------------------------------------------
    int cmd_wait = 1;
    // Subscribe to command channel
    perllcm_auv_remotehelm_cmd_t_subscription_t *rh_cmd_sub =
        perllcm_auv_remotehelm_cmd_t_subscribe (state->lcm, state->rh_cmd_channel,
                                                &perllcm_auv_remotehelm_cmd_t_callback, state);
    while (!g_done && cmd_wait) {
        
        printf ("Waiting for command .... \n");
        lcm_handle (state->lcm);
        
        if (NULL != state->rh_cmd) {
        
            if (state->rh_cmd->type == PERLLCM_AUV_REMOTEHELM_CMD_T_START_RH) { // setup mission
                printf ("Received start mission command. \n");
    
                setup_mission (state, state->rh_cmd->mission_name, state->
                               rh_cmd->start_mission_sleep);
                
                // publish to start logging
                pub_start_logging (state);
                
                // start mission
                int sleep = state->mstate->start_mission_sleep;
                while (sleep-- > 0) {
                    printf ("Starting mission in %d...\n", sleep);
                    timeutil_sleep (1);
                }
                
                // init status values
                state->mstate->mission_running = 0;
                state->mstate->osi_mode = 0;
                state->mstate->last_osi_mode = 0;
                
                if (1 == send_omstart (state)) {
                    printf ("caught in main OMSTART ACK in error or timeout\n");
                    senlcm_uvc_omstop_t cmd = {
                        .msg_flag = 0,
                    };
                    struct timeval to = {.tv_sec = 2};
                    while (0 != iver_rh_pub_omstop (state->lcm, state->osc_omstop_channel, &cmd, state->param, &to)) {
                        printf ("Sending OMSTOP\n");
                        to = (struct timeval){.tv_sec = 2};
                    }

                    perllcm_auv_remotehelm_cmd_t_unsubscribe (state->lcm, rh_cmd_sub);
                    free_mission_state (state->mstate);
                    state->mstate = NULL;
                }
                else {
                    cmd_wait = 0; // ready to start mission
                }
                
            } else if (state->rh_cmd->type == PERLLCM_AUV_REMOTEHELM_CMD_T_PAUSE_RH) { // stop logging
                printf ("Received pause mission command. \n");
                
                // stop logging
                pub_stop_logging (state);
            
            } else {
                ERROR ("Unkonwn remotehelm command %d \n", state->rh_cmd->type);
            }
            
            perllcm_auv_remotehelm_cmd_t_destroy (state->rh_cmd);
            state->rh_cmd = NULL;
            
        }
    }
    // if we were interupted by ctrl+c exit (no need to continue through mission loop)
    if (g_done)
        exit (EXIT_SUCCESS);
    
    // unsubscribe to command channel
    perllcm_auv_remotehelm_cmd_t_unsubscribe (state->lcm, rh_cmd_sub);
    
    // create mission lcm subscriptions
    perllcm_heartbeat_t_subscription_t *hb1_sub =
        perllcm_heartbeat_t_subscribe (state->lcm, state->hb1_channel, &hb1_callback, state);
    perllcm_auv_safety_rules_t_subscription_t *saftey_rules_sub = 
        perllcm_auv_safety_rules_t_subscribe (state->lcm, state->safety_channel, &safety_rules_callback, state);
    senlcm_uvc_osi_t_subscription_t *osi_sub = 
        senlcm_uvc_osi_t_subscribe (state->lcm, state->osc_osi_channel, &osi_callback, state);
    perllcm_auv_abort_t_subscription_t *rh_abort_sub =
        perllcm_auv_abort_t_subscribe (state->lcm, state->rh_abort_channel, &abort_callback, state);  
    perllcm_auv_jump_t_subscription_t *rh_jump_sub =
        perllcm_auv_jump_t_subscribe (state->lcm, state->rh_jump_channel, &jump_callback, state);
    perllcm_auv_abort_t_subscription_t *acomms_abort_sub =
        perllcm_auv_abort_t_subscribe (state->lcm, state->acomms_abort_channel, &abort_callback, state);
    perllcm_auv_jump_t_subscription_t *acomms_jump_sub =
        perllcm_auv_jump_t_subscribe (state->lcm, state->acomms_jump_channel, &jump_callback, state);  
    senlcm_easydaq_t_subscription_t *easydaq_sub = 
        senlcm_easydaq_t_subscribe (state->lcm, state->easydaq_channel, &easydaq_callback, state);
    
    
    // MISSION LOOP ------------------------------------------------------------
    printf ("Running Mission.\n");
    while (!g_done && !state->mstate->mission_done) {
        lcm_handle (state->lcm);  
    }
    state->mstate->mission_running = 0;
    
    // End of Mission or Abort
    // remove mission lcm subscriptions
    perllcm_heartbeat_t_unsubscribe (state->lcm, hb1_sub);
    perllcm_auv_safety_rules_t_unsubscribe (state->lcm, saftey_rules_sub);
    senlcm_uvc_osi_t_unsubscribe (state->lcm, osi_sub);
    perllcm_auv_abort_t_unsubscribe (state->lcm, rh_abort_sub);
    perllcm_auv_jump_t_unsubscribe (state->lcm, rh_jump_sub);
    perllcm_auv_abort_t_unsubscribe (state->lcm, acomms_abort_sub);
    perllcm_auv_jump_t_unsubscribe (state->lcm, acomms_jump_sub);
    senlcm_easydaq_t_unsubscribe (state->lcm, easydaq_sub);
    
    // send saftey omstp to be really sure that it stops 
    senlcm_uvc_omstop_t cmd = {
        .msg_flag = 0,
    };
    struct timeval to = {.tv_sec = 2};
    while (0 != iver_rh_pub_omstop (state->lcm, state->osc_omstop_channel, &cmd, state->param, &to)) {
        printf ("Sending OMSTOP\n");
        to = (struct timeval){.tv_sec = 2};
    }
    
    // copy uvc log into mission dir
    printf ("Mission ended, moving uvc log\n");
    dir_mgmt_mv_uvc_log (&state->mstate->dir_mgmt, state->mstate->mission_file);
    
    // clean up
    free_mission_state (state->mstate);
    state->mstate = NULL;
    
}


static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("my_signal_handler()\n");
    if (g_done) {
        printf ("Goodbye\n");
        exit (EXIT_FAILURE);
    }
    else
        g_done = 1;
}

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // install custom signal handler
    struct sigaction act = { .sa_sigaction = my_signal_handler };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);

    state_t *state = init_state (argc, argv);

    while (!g_done) {
        run_mission (state);
    }
    
    // clean up
    free_state (state);
    
    exit (EXIT_SUCCESS);
}
