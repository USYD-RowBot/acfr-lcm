#include <signal.h>
#include <libgen.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <bot_param/param_client.h>


#include "pid.h"
#include "timer.h"

#include "perls-common/timestamp.h"
#include "perls-common/lcm_util.h"
#include "perls-lcmtypes/acfrlcm_auv_acfr_nav_t.h"
#include "perls-lcmtypes/acfrlcm_auv_iver_motor_command_t.h"

// set the delta T to 0.1s, 10Hz loop rate
#define CONTROL_DT 0.1

typedef struct
{
    double vx;
} command_t;

typedef struct 
{
    lcm_t *lcm;
    
    // controller gains
    pid_gains_t gains_vel;
    
    // Nav solution
    acfrlcm_auv_acfr_nav_t nav;
    pthread_mutex_t nav_lock;
    int nav_alive;
    
    // controller inputs
    command_t command;
    
} state_t;

// load all the config variables
int load_config(state_t *state, char *rootkey)
{
    BotParam *param;
	char key[64];
	param = bot_param_new_from_server(state->lcm, 1);

	// velocity gains
	memset(&state->gains_vel, 0, sizeof(pid_gains_t));
	sprintf(key, "%s.velocity.kp", rootkey);
	state->gains_vel.kp = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.velocity.ki", rootkey);
	state->gains_vel.ki = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.velocity.kd", rootkey);
	state->gains_vel.kd = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.velocity.sat", rootkey);
	state->gains_vel.sat = bot_param_get_double_or_fail(param, key);


    return 1;
}

// ACFR Nav callback, as this program handles its own timing we just make a copy of this data
// every time it comes in
static void
acfr_nav_callback (const lcm_recv_buf_t *rbuf, const char *channel, const acfrlcm_auv_acfr_nav_t *nav, void *user) 
{
    state_t *state = (state_t *)user;
    
    // lock the nav structure and copy the data in, this may introduce a timing problem for the main loop
    pthread_mutex_lock(&state->nav_lock);
    memcpy(&state->nav, nav, sizeof(acfrlcm_auv_acfr_nav_t));
    state->nav_alive = 1;
    pthread_mutex_unlock(&state->nav_lock);
}

// Exit handler
int main_exit;
void signal_handler(int sigNum) 
{
    // do a safe exit
    main_exit = 1;
}

// Process LCM messages with callbacks
static void *lcm_thread (void *context) {
    state_t *state = (state_t *)context;

    while (!main_exit) {
        struct timeval tv;
	    tv.tv_sec = 1;
	    tv.tv_usec = 0;

        lcmu_handle_timeout(state->lcm, &tv);
    }
    return 0;
}

int main(int argc, char **argv)
{
    // install the signal handler
    main_exit = 0;
    signal(SIGINT, signal_handler);

    state_t state;
    state.lcm = lcm_create(NULL);
    
    char root_key[64];
    sprintf(root_key, "acfr.%s", basename(argv[0]));  
    load_config(&state, root_key);
    
    // LCM thread
    pthread_t lcm_tid;
    pthread_create(&lcm_tid, NULL, lcm_thread, &state);
    pthread_detach(lcm_tid);
    
    // locks etc
    pthread_mutex_init(&state.nav_lock, NULL);	
    state.nav_alive = 0;
 
    // LCM callbacks
    acfrlcm_auv_acfr_nav_t_subscribe (state.lcm, "ACFR_NAV", &acfr_nav_callback, &state);      
//    acfrlcm_auv_goal_setpoint_t_subscribe(state.lcm, "MP_CONTROL", &mp_message_callback, &state);
 
    periodic_info timer_info;
	make_periodic (CONTROL_DT * 1000000, &timer_info);
	
	double prop_rpm;
	double top_rudder_angle, bottom_rudder_angle, port_plane_angle, starboard_plane_angle;
	
	
	// main loop
	while(!main_exit)
	{
	    // lock the nav data
	    pthread_mutex_lock(&state.nav_lock);
	
        // X Velocity
        prop_rpm = pid(&state.gains_vel, state.nav.vx, state.command.vx, CONTROL_DT);

        // unlock the nav data
        pthread_mutex_unlock(&state.nav_lock);
        
        // send the motor command
        acfrlcm_auv_iver_motor_command_t mc;
        mc.utime = timestamp_now();
        mc.main = prop_rpm;
        mc.top = 0;
        mc.bottom = 0;
        mc.port = 0;
        mc.starboard = 0;
        mc.source = ACFRLCM_AUV_IVER_MOTOR_COMMAND_T_AUTO;
        acfrlcm_auv_iver_motor_command_t_publish(state.lcm, "IVER_MOTOR", &mc);

        // wait for a timer event
        wait_period(&timer_info);	
	}

    pthread_join(lcm_tid, NULL);
    pthread_mutex_destroy(&state.nav_lock);
	
	return 1;
}
