#include "callbacks.h"
#include "remotehelm.h"

#include "perls-iver/vectormap.h"
#include "perls-iver/remotehelm.h"

void 
perllcm_auv_remotehelm_cmd_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                       const perllcm_auv_remotehelm_cmd_t *msg, void *user) {
    
    state_t *state = user;

    if (NULL != state->rh_cmd) {
        perllcm_auv_remotehelm_cmd_t_destroy (state->rh_cmd);
        state->rh_cmd = NULL;
    }
    state->rh_cmd = perllcm_auv_remotehelm_cmd_t_copy (msg);
}

void
disp_hb1_status (state_t *state) {
    
    mission_state_t *mstate = state->mstate;
    
    // display status text
    switch (mstate->osi_mode) {
        case SENLCM_UVC_OSI_T_MODE_NORMAL:
            if (mstate->last_osi_mode != SENLCM_UVC_OSI_T_MODE_NORMAL)
                printf ("Normal Mode (Mission Running)\n");
            mstate->last_osi_mode = SENLCM_UVC_OSI_T_MODE_NORMAL;
            break;
        case SENLCM_UVC_OSI_T_MODE_PARK:
            if (mstate->osi_park_time_left > 0 || mstate->last_osi_mode != SENLCM_UVC_OSI_T_MODE_PARK)
                printf ("Park Mode %d seconds left \n", mstate->osi_park_time_left);
            mstate->last_osi_mode = SENLCM_UVC_OSI_T_MODE_PARK;
            break;
        case SENLCM_UVC_OSI_T_MODE_STOPPED:
            if (mstate->last_osi_mode != SENLCM_UVC_OSI_T_MODE_STOPPED)
                printf ("Stopped Mode (Mission Finished) \n");
            mstate->last_osi_mode = SENLCM_UVC_OSI_T_MODE_STOPPED;
            break;
        case SENLCM_UVC_OSI_T_MODE_MANUAL_PARK:
            if (mstate->last_osi_mode != SENLCM_UVC_OSI_T_MODE_MANUAL_PARK)
                printf ("Manual Park Mode \n");
            mstate->last_osi_mode = SENLCM_UVC_OSI_T_MODE_MANUAL_PARK;
            break;
        case SENLCM_UVC_OSI_T_MODE_MANUAL_OVERRIDE:
            if (mstate->last_osi_mode != SENLCM_UVC_OSI_T_MODE_MANUAL_OVERRIDE)
                printf ("Manual Override Mode \n");
            mstate->last_osi_mode = SENLCM_UVC_OSI_T_MODE_MANUAL_OVERRIDE;
            break;
        case SENLCM_UVC_OSI_T_MODE_SERVO:
            if (mstate->last_osi_mode != SENLCM_UVC_OSI_T_MODE_SERVO)
                printf ("Servo Mode \n");
            mstate->last_osi_mode = SENLCM_UVC_OSI_T_MODE_SERVO;
            break;
        case SENLCM_UVC_OSI_T_MODE_MISSION:
            if (mstate->last_osi_mode != SENLCM_UVC_OSI_T_MODE_MISSION)
                printf ("Mission Mode \n");
            mstate->last_osi_mode = SENLCM_UVC_OSI_T_MODE_MISSION;
            break;
    }
}


static int 
modify_easydaq_cam (char * label, int on_state , mission_state_t *mstate)
{
    int modified = 0;
    for (int i=0; i<8; i++) {
        if (0==strcmp (mstate->easydaq.relay[i].label, label) && mstate->easydaq.relay[i].state != on_state) {
            mstate->easydaq.relay[i].state = on_state;
            modified = 1;
        }
    }
    return modified;
}


void
update_cameras (state_t *state) {
    
    mission_state_t *mstate = state->mstate;
    
    iver_vectormap_waypoint_t *wypnt = mstate->wypnt_list->data;
    static int wypnt_change = 1; //waypoint change flag
    
    // restart if the next waypoint is less than where we are currently in the list
    if (mstate->next_wypnt < wypnt->num) {
        mstate->wypnt_list = g_list_first (mstate->wypnt_list);
        mstate->next_wypnt = 1;
        wypnt = mstate->wypnt_list->data; //get the first waypoint
        wypnt_change = 1;
    }

    //jump to the current waypoint in the list (we want to apply settings from next waypoint
    // to the current leg, this is how vectormap works)
    while (g_list_next (mstate->wypnt_list) && wypnt->num < mstate->next_wypnt) {
        mstate->wypnt_list = mstate->wypnt_list->next; //move to next wypnt in list
        wypnt = mstate->wypnt_list->data; //update the data variable
        wypnt_change = 1;
    }
    
    int modified = 0;
    if (mstate->next_wypnt <= mstate->num_wypnts) {
        if (wypnt->cam1.mode) {
            modified = modify_easydaq_cam ("cam1" , 1 , mstate)? : modified;
            if (wypnt_change)
                printf ("Next Waypoint %d: CAM1 (Color) On\r\n", mstate->next_wypnt);
        }
        else {
            modified = modify_easydaq_cam ("cam1" , 0 , mstate)? : modified;
            if (wypnt_change)
                printf ("Next Waypoint %d: CAM1 (Color) Off\r\n", mstate->next_wypnt);
        }
        
        if (wypnt->cam2.mode) {
            modified = modify_easydaq_cam ("cam2" , 1 , mstate)? : modified;
            if (wypnt_change)
                printf ("Next Waypoint %d: CAM2 (Mono) On\r\n", mstate->next_wypnt);
        }
        else {
            modified = modify_easydaq_cam ("cam2" , 0 , mstate)? : modified;
            if (wypnt_change)
                printf ("Next Waypoint %d: CAM2 (Mono) Off\r\n", mstate->next_wypnt);
        }
        
    } 
    else { //when next_wypnt is beyond the end of the mission always set both cameras off
        modified = modify_easydaq_cam ("cam1" , 0 , mstate) ? : modified;
        modified = modify_easydaq_cam ("cam2" , 0 , mstate) ? : modified;
        
    }

    if (modified) {
        mstate->easydaq.self = 0;
        mstate->easydaq.utime = timestamp_now();  
        senlcm_easydaq_t_publish (state->lcm, state->easydaq_channel, &(mstate->easydaq));
    }

    wypnt_change = 0;    
    
}

void
hb1_callback (const lcm_recv_buf_t *rbuf, const char *channel,
              const perllcm_heartbeat_t *msg, void *user) {
    
    state_t *state = user;
    
    // print status message
    disp_hb1_status (state);
    
    // update cameras
    update_cameras (state);
    
    // publish a mission status message
    perllcm_auv_mission_status_t mission_status = {
        .mission_running = state->mstate->mission_running,
    };
    perllcm_auv_mission_status_t_publish (state->lcm,
                                          state->rh_mission_channel,
                                          &mission_status);
    
}
  

void
safety_rules_callback (const lcm_recv_buf_t *rbuf, const char *channel,
        const perllcm_auv_safety_rules_t *rules, void *user)
{
    state_t *state = user;
    
    // abort mission if safety rules are violated
    if (rules->safety_rules_violated & rules->safety_rules_active) {
        perllcm_auv_abort_t cmd = {
            .utime = timestamp_now (),
            .dest  = state->node_id,
            .abort = PERLLCM_AUV_ABORT_T_ABORT_HARD 
        };
        perllcm_auv_abort_t_publish (state->lcm, state->rh_abort_channel, &cmd);
    }
}

void 
osi_callback (const lcm_recv_buf_t *rbuf, const char * channel, 
              const senlcm_uvc_osi_t * msg, void * user)
{
    state_t *state = user;
    mission_state_t *mstate = state->mstate;
    //Update the state to reflect the current waypoint
    //next waypoint in osi is indexed 0 num_waypnts-1
    state->mstate->next_wypnt = msg->nextwp;
    state->mstate->osi_mode = msg->mode;
    state->mstate->osi_park_time_left = msg->park_time;
    
    // check if mission is done
    switch (mstate->osi_mode) {
        case SENLCM_UVC_OSI_T_MODE_NORMAL:
            state->mstate->mission_running = 1;
            break;
        case SENLCM_UVC_OSI_T_MODE_PARK:
            state->mstate->mission_running = 1;
            break;
        case SENLCM_UVC_OSI_T_MODE_STOPPED:
            if (state->mstate->mission_running) // we were running but now we've stopped, mission over
                state->mstate->mission_done = 1;
            break;
        case SENLCM_UVC_OSI_T_MODE_MANUAL_PARK:
            ERROR ("Unsure of what to do in SENLCM_UVC_OSI_T_MODE_MANUAL_PARK aborting!");
            state->mstate->mission_done = 1;
            break;
        case SENLCM_UVC_OSI_T_MODE_MANUAL_OVERRIDE:
            ERROR ("Unsure of what to do in SENLCM_UVC_OSI_T_MODE_MANUAL_OVERRIDE aborting!");
            state->mstate->mission_done = 1;
            break;
        case SENLCM_UVC_OSI_T_MODE_SERVO:
            ERROR ("Unsure of what to do in SENLCM_UVC_OSI_T_MODE_SERVO aborting!");
            state->mstate->mission_done = 1;
            break;
        case SENLCM_UVC_OSI_T_MODE_MISSION:
            ERROR ("Unsure of what to do in SENLCM_UVC_OSI_T_MODE_MISSION aborting!");
            state->mstate->mission_done = 1;
            break;
    }
    
}


void
abort_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                const perllcm_auv_abort_t *msg, void *user)
{
    state_t *state = user;
    // exit abort_callback if this abort message is not meant for us
    if (msg->dest != state->node_id)
        return;

    // abort destination meant for us
    switch (msg->abort) {
        case PERLLCM_AUV_ABORT_T_ABORT_FALSE:
            break;
        case PERLLCM_AUV_ABORT_T_ABORT_TO_SURFACE:
            // insert waypoint on surface, then end mission
        case PERLLCM_AUV_ABORT_T_ABORT_TO_WAYPOINT:
        case PERLLCM_AUV_ABORT_T_ABORT_TO_POS:
        case PERLLCM_AUV_ABORT_T_ABORT_HARD:
        default: 
        {
            printf ("Abort: Sending OMSTOP\n");
            senlcm_uvc_omstop_t cmd = {
                .msg_flag = 0,
            };
            struct timeval to = {.tv_sec = 2};
            if (0 == iver_rh_pub_omstop (state->lcm, state->osc_omstop_channel, &cmd, state->param, &to)) {
                printf ("Abort: Ack received on OMSTOP\n");
                state->mstate->mission_done = 0;
            }
        }
    }
}

void
jump_callback (const lcm_recv_buf_t *rbuf, const char *channel,
               const perllcm_auv_jump_t *msg, void *user)
{
    state_t *state = user;
    // make sure this cmd is for us
    if (msg->dest != state->node_id)
        return;

    // if the jump's next_wypnt is nonsensical, go to last waypoint
    senlcm_uvc_ojw_t cmd = {
        .waypoint = ((msg->next_wypnt < state->mstate->num_wypnts &&
                      msg->next_wypnt >= 1) ? msg->next_wypnt : state->mstate->num_wypnts),
    };
    if (cmd.waypoint != msg->next_wypnt)
        printf ("Jump: Clipping nonsensical waypoint [%d]->[%d]\n",
                msg->next_wypnt, cmd.waypoint);

    printf ("Jump: Sending OJW to WP%d\n", cmd.waypoint);
    struct timeval to = {.tv_sec = 2};
    if (0 == iver_rh_pub_ojw (state->lcm, state->osc_ojw_channel, &cmd, state->param, &to))
        printf ("Jump: Ack received on OJW to WP%d\n", cmd.waypoint);
}

void
easydaq_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                  const senlcm_easydaq_t *msg, void *user)
{
    state_t *state = user;
    
    //only update easydaq state if it is reported by itself
    if (msg->self) {
        state->mstate->easydaq.utime = msg->utime;
        state->mstate->easydaq.self  = msg->self;
        for (size_t i=0; i<8; i++) {
            strcpy (state->mstate->easydaq.relay[i].label, msg->relay[i].label);
            strcpy (state->mstate->easydaq.relay[i].group, msg->relay[i].group);
            state->mstate->easydaq.relay[i].state = msg->relay[i].state;
        }
    }
}
