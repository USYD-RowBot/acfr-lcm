#include <signal.h>
#include <libgen.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <bot_param/param_client.h>
#include "acfr-common/timestamp.h"

#include "../auv-control/pid.h"
#include "../auv-control/timer.h"


#include "perls-lcmtypes/senlcm_seabotix_sensors_t.h"
#include "perls-lcmtypes/senlcm_micron_sounder_t.h"
#include "perls-lcmtypes/acfrlcm_seabotix_joystick_t.h"
#include "perls-lcmtypes/acfrlcm_seabotix_command_t.h"

// set the delta T to 0.1s, 10Hz loop rate
#define CONTROL_DT 0.1

typedef struct
{
    lcm_t *lcm;

    // controller gains
    pid_gains_t gains_heading;
    pid_gains_t gains_depth;
    pid_gains_t gains_altitude;
    
    // Seabotix sensors
    senlcm_seabotix_sensors_t seabotix;
    pthread_mutex_t seabotix_lock;
    
    // Micron altimeter
    senlcm_micron_sounder_t micron;
    pthread_mutex_t micron_lock;
    
    // command
    acfrlcm_seabotix_command_t command;
    pthread_mutex_t command_lock;

} state_t;


// Handlers
void micron_handler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_micron_sounder_t *micron, void *u)
{
    state_t *state = (state_t *)u;
    pthread_mutex_lock(&state->micron_lock);
    memcpy(&state->micron, micron, sizeof(senlcm_micron_sounder_t));
    pthread_mutex_unlock(&state->micron_lock);
}

void seabotix_handler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_seabotix_sensors_t *ss, void *u)
{
    state_t *state = (state_t *)u;
    pthread_mutex_lock(&state->seabotix_lock);
    memcpy(&state->seabotix, ss, sizeof(senlcm_seabotix_sensors_t));
    pthread_mutex_unlock(&state->seabotix_lock);
}

void command_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_seabotix_command_t *cmd, void *u)
{
    state_t *state = (state_t *)u;
    pthread_mutex_lock(&state->command_lock);
    memcpy(&state->command, cmd, sizeof(acfrlcm_seabotix_command_t));
    pthread_mutex_unlock(&state->command_lock);
}

// load all the config variables
int load_config(state_t *state, char *rootkey)
{
    BotParam *param;
    char key[64];
    param = bot_param_new_from_server(state->lcm, 1);
    
    // Depth gains
    memset(&state->gains_depth, 0, sizeof(pid_gains_t));
    sprintf(key, "%s.depth.kp", rootkey);
    state->gains_depth.kp = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.depth.ki", rootkey);
    state->gains_depth.ki = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.depth.kd", rootkey);
    state->gains_depth.kd = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.depth.sat", rootkey);
    state->gains_depth.sat = bot_param_get_double_or_fail(param, key);

    // Altitude gains
    memset(&state->gains_altitude, 0, sizeof(pid_gains_t));
    sprintf(key, "%s.altitude.kp", rootkey);
    state->gains_altitude.kp = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.altitude.ki", rootkey);
    state->gains_altitude.ki = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.altitude.kd", rootkey);
    state->gains_altitude.kd = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.altitude.sat", rootkey);
    state->gains_altitude.sat = bot_param_get_double_or_fail(param, key);

    // Heading gains
    memset(&state->gains_heading, 0, sizeof(pid_gains_t));
    sprintf(key, "%s.heading.kp", rootkey);
    state->gains_heading.kp = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.heading.ki", rootkey);
    state->gains_heading.ki = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.heading.kd", rootkey);
    state->gains_heading.kd = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.heading.sat", rootkey);
    state->gains_heading.sat = bot_param_get_double_or_fail(param, key);
    
    return 1;
}
    

// Exit handler
int main_exit;
void signal_handler(int sigNum)
{
    // do a safe exit
    main_exit = 1;
}

// Process LCM messages with callbacks
static void *lcm_thread(void *context)
{
    state_t *state = (state_t *) context;

    while (!main_exit)
    {
        lcm_handle_timeout(state->lcm, 1000);
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
    state.gains_depth.integral = 0;
    state.gains_altitude.integral = 0;
    state.gains_heading.integral = 0;

    char root_key[64];
    sprintf(root_key, "acfr.%s", basename(argv[0]));
    load_config(&state, root_key);

    // LCM thread
    pthread_t lcm_tid;
    pthread_create(&lcm_tid, NULL, lcm_thread, &state);
    pthread_detach(lcm_tid);

    pthread_mutex_init(&state.micron_lock, NULL);
    pthread_mutex_init(&state.seabotix_lock, NULL);
    
    // LCM callbacks
    senlcm_micron_sounder_t_subscribe(state.lcm, "MICRON_SOUNDER", &micron_handler, &state);
    senlcm_seabotix_sensors_t_subscribe(state.lcm, "SEABOTIX_SENSORS", &seabotix_handler, &state);
    acfrlcm_seabotix_command_t_subscribe(state.lcm, "SEABOTIX_COMMAND", &command_handler, &state);
    
    // Timer for the control loop
    periodic_info timer_info;
    make_periodic(CONTROL_DT * 1000000, &timer_info);

    double v_value;
    double z_value;

    // main loop
    while (!main_exit)
    {
        
        // lock everything
        pthread_mutex_lock(&state.command_lock);
        pthread_mutex_lock(&state.micron_lock);
        pthread_mutex_lock(&state.seabotix_lock);
        
        if(state.command.mode & ACFRLCM_SEABOTIX_COMMAND_T_DEPTH_MODE)
        {
            v_value = pid(&state.gains_depth, state.seabotix.depth, state.command.depth, CONTROL_DT);
        }
        else if(state.command.mode & ACFRLCM_SEABOTIX_COMMAND_T_ALTITUDE_MODE)
        {
            v_value = pid(&state.gains_altitude, state.micron.altitude, state.command.altitude, CONTROL_DT);
        }
        
      
        while (state.seabotix.heading < -M_PI)
            state.seabotix.heading += 2 * M_PI;
        while (state.seabotix.heading > M_PI)
            state.seabotix.heading -= 2 * M_PI;

        while (state.command.heading < -M_PI)
            state.command.heading += 2 * M_PI;
        while (state.command.heading > M_PI)
            state.command.heading -= 2 * M_PI;

        double diff_heading = state.seabotix.heading - state.command.heading;
        while( diff_heading < -M_PI )
            diff_heading += 2*M_PI;
        while( diff_heading > M_PI )
            diff_heading -= 2*M_PI;  
        
        
        if(state.command.mode & ACFRLCM_SEABOTIX_COMMAND_T_HEADING_MODE)
        {
            z_value = pid(&state.gains_heading, diff_heading, 0.0, CONTROL_DT);
        }
        
        // unlock everything
        pthread_mutex_unlock(&state.command_lock);
        pthread_mutex_unlock(&state.micron_lock);
        pthread_mutex_unlock(&state.seabotix_lock);
        
        // create the joystick commant to send
        acfrlcm_seabotix_joystick_t joystick;
        joystick.utime = timestamp_now();
        joystick.x = 0;
        joystick.y = 0;
        joystick.z = z_value;
        joystick.v = v_value;
        acfrlcm_seabotix_joystick_t_publish(state.lcm, "SEABOTIX_JOYSTICK", &joystick);
        
        // wait for a timer event
        wait_period(&timer_info);
    }
    
    pthread_join(lcm_tid, NULL);
    pthread_mutex_destroy(&state.seabotix_lock);
    pthread_mutex_destroy(&state.micron_lock);
    pthread_mutex_destroy(&state.command_lock);

    return 1;
}        
        
