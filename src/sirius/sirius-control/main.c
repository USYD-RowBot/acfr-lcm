


#include "control.h"
#include "control_utils.h"


#ifndef BOT_CONF_DIR
#define DEFAULT_BOT_CONF_PATH "../config/master.cfg"
#else
#define DEFAULT_BOT_CONF_PATH BOT_CONF_DIR "/master.cfg"
#endif


int read_config_file(char *program_name, controller_config_t *cc, trajectory_config_t *tc, debug_t *debug)
{

    // read the config file
    BotParam *cfg;
    char rootkey[64];
    char key[64];

    char *path = getenv ("BOT_CONF_PATH");
    if (!path)
        path = DEFAULT_BOT_CONF_PATH;
    cfg = bot_param_new_from_file(path);
    if(cfg == NULL)
    {
        printf("cound not open config file\n");
        return 0;
    }

    sprintf(rootkey, "acfr.%s", basename(program_name));

    // PID values
    sprintf(key, "%s.heading.kp", rootkey);
    cc->gains.heading.kp = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.heading.ki", rootkey);
    cc->gains.heading.ki = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.heading.kd", rootkey);
    cc->gains.heading.kd = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.heading.sat", rootkey);
    cc->gains.heading.sat = bot_param_get_double_or_fail(cfg, key);
    cc->gains.heading.integral = 0;

    sprintf(key, "%s.depth.kp", rootkey);
    cc->gains.depth.kp = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.depth.ki", rootkey);
    cc->gains.depth.ki = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.depth.kd", rootkey);
    cc->gains.depth.kd = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.depth.sat", rootkey);
    cc->gains.depth.sat = bot_param_get_double_or_fail(cfg, key);
    cc->gains.depth.integral = 0;

    sprintf(key, "%s.sway.kp", rootkey);
    cc->gains.sway.kp = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.sway.ki", rootkey);
    cc->gains.sway.ki = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.sway.kd", rootkey);
    cc->gains.sway.kd = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.sway.sat", rootkey);
    cc->gains.sway.sat = bot_param_get_double_or_fail(cfg, key);
    cc->gains.sway.integral = 0;

    sprintf(key, "%s.surge.kp", rootkey);
    cc->gains.surge.kp = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.surge.ki", rootkey);
    cc->gains.surge.ki = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.surge.kd", rootkey);
    cc->gains.surge.kd = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.surge.sat", rootkey);
    cc->gains.surge.sat = bot_param_get_double_or_fail(cfg, key);
    cc->gains.surge.integral = 0;

    // Prop values
    sprintf(key, "%s.prop_factor", rootkey);
    cc->prop_factor = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.max_prop_rpm", rootkey);
    cc->max_prop_rpm = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.min_prop_rpm", rootkey);
    cc->min_prop_rpm = bot_param_get_double_or_fail(cfg, key);

    // Trajectory config
    sprintf(key, "%s.traj.heading_rate_limit", rootkey);
    tc->heading_rate_limit = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.traj.depth_rate_limit", rootkey);
    tc->depth_rate_limit = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.traj.surge_rate_limit", rootkey);
    tc->surge_rate_limit = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.traj.sway_rate_limit", rootkey);
    tc->sway_rate_limit = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.traj.heading_tau", rootkey);
    tc->heading_tau = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.traj.depth_tau", rootkey);
    tc->depth_tau = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.traj.surge_tau", rootkey);
    tc->surge_tau = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.traj.sway_tau", rootkey);
    tc->sway_tau = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.traj.two_point_max_step", rootkey);
    tc->two_point_max_step = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.traj.two_point_gain", rootkey);
    tc->two_point_gain = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.traj.tl_band", rootkey);
    tc->tl_band = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.traj.tl_gain", rootkey);
    tc->tl_gain = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.traj.fwd_distance_min", rootkey);
    tc->fwd_distance_min = bot_param_get_double_or_fail(cfg, key);
    sprintf(key, "%s.traj.fwd_distance_slowdown", rootkey);
    tc->fwd_distance_slowdown = bot_param_get_double_or_fail(cfg, key);

    sprintf(key, "%s.debug.mp", rootkey);
    debug->debug_mp = bot_param_get_boolean_or_fail(cfg, key);
    sprintf(key, "%s.debug.control", rootkey);
    debug->debug_control = bot_param_get_boolean_or_fail(cfg, key);
    sprintf(key, "%s.debug.trajectory", rootkey);
    debug->debug_trajectory = bot_param_get_boolean_or_fail(cfg, key);
    sprintf(key, "%s.debug.main", rootkey);
    debug->debug_main = bot_param_get_boolean_or_fail(cfg, key);

    return 1;
}

void init_controller_state(controller_state_t *cs, trajectory_state_t *ts, thruster_cmd_t *tc, ctl_status_t *ct, goal_setpoint_t *gs)
{
    // we are starting on the surface so the trajectory state get initialised to zero
    ts->heading.reference = 0.0;
    ts->heading.velocity = 0.0;
    ts->heading.acc = 0.0;

    ts->surge.reference = 0.0;
    ts->surge.velocity = 0.0;
    ts->surge.acc = 0.0;

    ts->sway.reference = 0.0;
    ts->sway.velocity = 0.0;
    ts->sway.acc = 0.0;

    ts->depth.reference = 0.0;
    ts->depth.velocity = 0.0;
    ts->depth.acc = 0.0;

    cs->heading_int = 0.0;
    cs->surge_int = 0.0;
    cs->sway_int = 0.0;
    cs->depth_int = 0.0;

    tc->lat = 0.0;
    tc->vert = 0.0;
    tc->port = 0.0;
    tc->stbd = 0.0;

    ct->at_goal = 0;
    ct->time_to_goal = 0.0;
    ct->dis_to_goal = 0.0;

    gs->xpos1 = NO_VALUE;
    gs->ypos1 = NO_VALUE;
    gs->zpos1 = NO_VALUE;
    gs->xpos2 = NO_VALUE;
    gs->ypos2 = NO_VALUE;
    gs->zpos2 = NO_VALUE;
    gs->xy_vel = NO_VALUE;
    gs->z_vel = NO_VALUE;
    gs->heading = NO_VALUE;
}

void init_modes(trajectory_mode_t *tm,  servo_mode_t *sm)
{
    tm->heading     = TRAJ_OFF;
    tm->depth.mode  = TRAJ_OFF;
    tm->transit     = TRAJ_OFF;
    tm->depth.value = NO_VALUE;
    sm->heading     = OFF;
    sm->depth       = DEPTH_OFF;
    sm->transit     = OFF;
}

void set_ctl_modes(goal_setpoint_t *gs, trajectory_mode_t *tm, servo_mode_t *sm, control_msg_t *cm, debug_t *debug)
{
    int NOT_SPEC = NO_VALUE + 1;

    // set the heading mode
    if (((int) gs->heading < NOT_SPEC) && ((int) gs->xpos2 <  NOT_SPEC))
    {
        tm->heading = TRAJ_OFF;
        sm->heading = OFF;
    }
    else
    {
        sm->heading = ON;
        if ((int) gs->xpos2 < NOT_SPEC)
            tm->heading = MAN_HEADING;
        else // xpos2 is given
        {
            if ((int) gs->xpos1 < NOT_SPEC)
            {
                if( (int)gs->heading < NOT_SPEC )
                    tm->heading = TRACKLINE_1P;   //get to a point
                else
                    tm->heading = MAN_HEADING;
            }
            else
            {
                if((int)gs->heading > NO_VALUE)
                    tm->heading = TRACKLINE_2P_H;  //two point trackline with heading
                else
                    tm->heading = TRACKLINE_2P;  //two point trackline
            }
        }
    } // end heading mode

    // set the transit mode
    if ((int) gs->xy_vel < NOT_SPEC)
    {
        tm->transit = TRAJ_OFF;
        sm->transit = OFF;
    }
    else
    {
        if( cm->transit_mode == CLOOP )
        {
            sm->transit = TRANSIT_CLOOP;
            if(tm->heading == MAN_HEADING && (int)(gs->xy_vel*100) == 0)
                tm->transit = CLOOP_HOLD;
            else if(tm->heading == TRACKLINE_2P_H)
                tm->transit = CLOOP_CRAB;
            else
                tm->transit = CLOOP;
        }
        else
        {
            tm->transit = OLOOP;
            sm->transit = TRANSIT_ON;
        }
    } // end transit mode

    //set the depth mode
    if ((int) gs->zpos2 < NOT_SPEC)
    {
        if(cm->tracking.track_by_sensor == ALTITUDE )
        {
            tm->depth.mode  = AUTO_DEPTH;
            tm->depth.value = cm->tracking.value;

            // 2010-04-20    mvj    Servo directly off altitude instead of off depth.  This should work better in swell.
            sm->depth = DEPTH_ALT;
            // 2010-04-20    mvj    To servo off depth (original configuration) uncomment the following line.
            // sm->depth = DEPTH_ON;
        }
        else
        {
            tm->depth.mode  = TRAJ_OFF;
            tm->depth.value = NO_VALUE;
            sm->depth       = DEPTH_OFF;
        }
    }
    else
    {
        sm->depth = DEPTH_ON;
        if (gs->zpos1 < NOT_SPEC)
        {
            tm->depth.mode = TRACKLINE_1P;
            tm->depth.value = NO_VALUE;
        }
        else
        {
            tm->depth.mode = TRACKLINE_2P;
            tm->depth.value = NO_VALUE;
        }
    } // end of depth mode

    debug_printf(debug->debug_main, "SC: set_ctl_modes, SERVO_MODES TRANSIT %d  HEADING %d  DEPTH %d\t\tTRAJ_MODES  TRANSIT %d  HEADING %d  DEPTH %d ALTITUDE %f\n",
                 sm->transit, sm->heading, sm->depth,
                 tm->transit, tm->heading, tm->depth.mode, tm->depth.value);

} //end set_ctl_modes

// Messages from the vehicle controller are now going to be sent via LCM so we can
// have multiple control clients, it makes the network code easier
int send_mp_message(state_t *state, trajectory_mode_t *tm, servo_mode_t *sm)
{

    acfrlcm_auv_control_response_t cr;
    cr.utime = timestamp_now();
    cr.x = state->nav.x;
    cr.y = state->nav.y;
    cr.depth = state->nav.depth;
    cr.heading = state->nav.heading;// * RTOD;
    cr.pitch = state->nav.pitch;// * RTOD;
    cr.roll = state->nav.roll;// * RTOD;
    cr.altitude = state->nav.altitude;
    cr.goal_id = state->goal_setpoint.goal_id;
    cr.time_to_goal = state->control_status.time_to_goal;
    cr.dist_to_goal = state->control_status.dis_to_goal;
    cr.at_goal = state->control_status.at_goal;

    if(sm->depth == DEPTH_ALT)
        cr.depth_mode = ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_ALT;
    else if(sm->depth == DEPTH_ON)
        cr.depth_mode = ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_ON;
    else
        cr.depth_mode = ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_OFF;

    if(tm->heading == TRACKLINE_1P)
        cr.heading_mode = ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_1P;
    else if(tm->heading == TRACKLINE_2P)
        cr.heading_mode = ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_2P;
    else if(tm->heading == TRACKLINE_2P_H)
        cr.heading_mode = ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_2PH;
    else if(tm->heading == MAN_HEADING)
        cr.heading_mode = ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_MAN;
    else
        cr.heading_mode = ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_OFF;


    if(tm->transit == CLOOP)
        cr.transit_mode = ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_CLOOP;
    else if(tm->transit == OLOOP)
        cr.transit_mode = ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_OLOOP;
    else if(tm->transit == CLOOP_HOLD)
        cr.transit_mode = ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_CLOOP_HOLD;
    else if(tm->transit == CLOOP_CRAB)
        cr.transit_mode = ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_CLOOP_CRAB;
    else
        cr.transit_mode = ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_OFF;


    acfrlcm_auv_control_response_t_publish(state->lcm, "CONTROL_RESPONSE", &cr);

    return 1;

}







static void
acfr_nav_callback (const lcm_recv_buf_t *rbuf, const char *channel, const acfrlcm_auv_acfr_nav_t *nav, void *user)
{
    state_t *state = (state_t *)user;

    // lock the nav structure and copy the data in, this may introduce a timing problem for the main loop
    pthread_mutex_lock(state->nav_lock);
    memcpy(&state->nav, nav, sizeof(acfrlcm_auv_acfr_nav_t));
    state->nav_alive = 1;
    pthread_mutex_unlock(state->nav_lock);
}

static void
mp_message_callback(const lcm_recv_buf_t *rbuf, const char *channel, const acfrlcm_auv_goal_setpoint_t *gs, void *user)
{
    state_t *state = (state_t *)user;


    if(gs->mode == ACFRLCM_AUV_GOAL_SETPOINT_T_MODE_GPT)
    {
        // we have a GPT message
        pthread_mutex_lock(state->mp_lock);
        state->goal_setpoint.timestamp = gs->utime;
        state->goal_setpoint.goal_id = gs->id;
        state->goal_setpoint.cmd_timeout = gs->timeout;
        state->goal_setpoint.xpos1 = gs->x1;
        state->goal_setpoint.ypos1 = gs->y1;
        state->goal_setpoint.zpos1 = gs->z1;
        state->goal_setpoint.xpos2 = gs->x2;
        state->goal_setpoint.ypos2 = gs->y2;
        state->goal_setpoint.zpos2 = gs->z2;
        state->goal_setpoint.heading = gs->heading;
        state->goal_setpoint.xy_vel = gs->xy_vel;
        state->goal_setpoint.z_vel = gs->z_vel;
        strcpy(state->goal_setpoint.goal_str, gs->str);

        debug_printf(state->debug.debug_mp, "SC: GPT: id:%i goal type:%s timeout:%lf x1:%lf y1:%lf z1:%lf x2:%lf y2:%lf z2:%lf hdg:%lf xyvel:%lf zvel:%lf\n",
                     gs->id, gs->str, gs->timeout, gs->x1, gs->y1, gs->z1,
                     gs->x2, gs->y2, gs->z2, gs->heading,
                     gs->xy_vel, gs->z_vel);

        state->new_mp = 1;
        pthread_mutex_unlock(state->mp_lock);
    }
    else if(gs->mode == ACFRLCM_AUV_GOAL_SETPOINT_T_MODE_TBS)
    {
        pthread_mutex_lock(state->mp_lock);
        if(!strcasecmp(gs->str, "ALTITUDE"))
        {
            state->control_msg.tracking.track_by_sensor = ALTITUDE;
            state->control_msg.tracking.value = gs->value;
        }
        if(!strcasecmp(gs->str, "NONE"))
        {
            state->control_msg.tracking.track_by_sensor = NONE;
            state->control_msg.tracking.value = NO_VALUE;
        }
        state->new_mp = 1;
        pthread_mutex_unlock(state->mp_lock);


        debug_printf(state->debug.debug_mp, "MP: TBS: %s = %f\n", gs->str, (double)gs->value);
    }

    else if(gs->mode == ACFRLCM_AUV_GOAL_SETPOINT_T_MODE_CTL_MODE)
    {
        pthread_mutex_lock(state->mp_lock);
        if(!strcasecmp(gs->str, "openloop"))
        {
            state->control_msg.transit_mode=OLOOP;
        }
        else if(!strcasecmp(gs->str, "closeloop"))
        {
            state->control_msg.transit_mode=CLOOP;
        }

        state->new_mp = 1;
        pthread_mutex_unlock(state->mp_lock);
    }

    else if(gs->mode == ACFRLCM_AUV_GOAL_SETPOINT_T_MODE_CTL_DOPR)
    {
        pthread_mutex_lock(state->mp_lock);
        state->control_msg.doppler_reset = atoi(gs->str);
        state->new_mp = 1;
        pthread_mutex_unlock(state->mp_lock);
    }

    else if(gs->mode == ACFRLCM_AUV_GOAL_SETPOINT_T_MODE_CTL_NAVSRC)
    {
        /*                  This isn't actually used with the ACFR estimator

                        if(strcasecmp(str_value1, "best") == 0)
                            cm->estimator_mode = BEST;
                        else if(strcasecmp(str_value1, "lbl") == 0)
                            cm->estimator_mode = LBL;
                        else if(strcasecmp(str_value1, "doppler") == 0)
                            cm->estimator_mode = DOPPLER;
                        else if(strcasecmp(str_value1, "exact") == 0)
                            cm->estimator_mode = EXACT;
                        else if(strcasecmp(str_value1, "dop_lbl") == 0)
                            cm->estimator_mode = DOP_LBL;
                        else
                            return MP_PARSE_ERROR;
        */
    }

}



int program_state;
void
signal_handler(int sigNum)
{
    // do a safe exit
    program_state = 1;
}

// Process LCM messages with callbacks
static void *lcm_thread (void *context)
{
    state_t *state = (state_t *)context;

    while (program_state != 1)
    {
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        lcmu_handle_timeout(state->lcm, &tv);
    }
    return 0;
}

// Main program entry
int main(int argc, char **argv)
{
    // First create all the structures needed
    controller_config_t controller_config;
    trajectory_config_t trajectory_config;

    controller_state_t controller_state;
    trajectory_state_t trajectory_state;


    memset(&controller_config, 0, sizeof(controller_config_t));
    memset(&trajectory_config, 0, sizeof(trajectory_config_t));
    memset(&controller_state, 0, sizeof(controller_state_t));
    memset(&trajectory_state, 0, sizeof(trajectory_state_t));

    thruster_cmd_t thruster_command;
    servo_mode_t servo_mode;
    trajectory_mode_t trajectory_mode;
    thruster_cmd_t thruster_cmd;
    state_t state;
    state.start_time = timestamp_now();

    // read the LCM config file
    if(!read_config_file(argv[0], &controller_config, &trajectory_config, &state.debug))
        return 0;

    // initialise the states
    init_controller_state(&controller_state, &trajectory_state, &thruster_command, &state.control_status, &state.goal_setpoint);
    state.goal_setpoint.goal_id = 0;
    init_modes(&trajectory_mode, &servo_mode);

    // install the signal handler
    program_state = 0;
    signal(SIGINT, signal_handler);

    // start LCM and create a thread to handle incomming messages
    state.new_mp = 0;
    state.nav_alive = 0;
    state.lcm = lcm_create(NULL);
    state.nav_lock = malloc(sizeof(pthread_mutex_t));
    pthread_mutex_init(state.nav_lock, NULL);
    state.mp_lock = malloc(sizeof(pthread_mutex_t));
    pthread_mutex_init(state.mp_lock, NULL);
    pthread_t lcm_tid;
    pthread_create(&lcm_tid, NULL, lcm_thread, &state);
    pthread_detach(lcm_tid);

    // subscribe to the required LCM messages
    acfrlcm_auv_acfr_nav_t_subscribe (state.lcm, "ACFR_NAV", &acfr_nav_callback, &state);
    acfrlcm_auv_goal_setpoint_t_subscribe(state.lcm, "MP_CONTROL", &mp_message_callback, &state);

    // other LCM related stuff
    acfrlcm_auv_sirius_motor_command_t motor_command;

    int motor_count = 0;

    periodic_info timer_info;
    make_periodic (CONTROL_DT * 1000000, &timer_info);

    while(program_state == 0)
    {
        // we need to wait for the estimator to be alive, so we will wait here
//        if(!state.nav_alive)
//            continue;

        // check abort conditions and mission state
        // lock the nav memory, this is bad for determinitic behaviour
        pthread_mutex_lock(state.nav_lock);
        pthread_mutex_lock(state.mp_lock);

        // check for mp mesages
        if(state.new_mp)
        {
            set_ctl_modes(&state.goal_setpoint, &trajectory_mode, &servo_mode, &state.control_msg, &state.debug);
            state.new_mp = 0;
        }

        check_mp_goals(&state.control_status, &state.goal_setpoint, &trajectory_mode, &state.nav, &state.debug);


        // call trajectory
        trajectory(&state.goal_setpoint, &trajectory_state, &state.nav, &trajectory_config, &trajectory_mode, &controller_state, &state.debug);

        // call controller
        controller(&controller_config, &state.nav, &trajectory_state, &servo_mode, &thruster_cmd, &state.debug);

        // broadcast the AUV status
        send_mp_message(&state, &trajectory_mode, &servo_mode);

        // unlock the nav memory
        pthread_mutex_unlock(state.nav_lock);
        pthread_mutex_unlock(state.mp_lock);

        // send motor command, we don't want to do this at 10Hz
//        if(motor_count == 9)
//        {
        motor_command.utime = timestamp_now();
        motor_command.vertical = thruster_cmd.vert;
        motor_command.port = thruster_cmd.port;
        motor_command.starboard = thruster_cmd.stbd;
        acfrlcm_auv_sirius_motor_command_t_publish(state.lcm, "SIRIUS_MOTOR", &motor_command);
//            motor_count = 0;
//        }
//        else
//            motor_count++;

        // wait for a timer event
        wait_period(&timer_info);
    }


    printf("SC: Exiting\n");
    // Gracefully exit
    // set the stop mode if its not already set
    program_state = 1;

    // wait for the threads
    pthread_join(lcm_tid, NULL);
    pthread_mutex_destroy(state.nav_lock);
    pthread_mutex_destroy(state.mp_lock);
    free(state.nav_lock);
    free(state.mp_lock);


    return 0;
}
