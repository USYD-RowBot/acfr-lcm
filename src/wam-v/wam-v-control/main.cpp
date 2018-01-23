#include <string>
#include <signal.h>
#include <libgen.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <bot_param/param_client.h>

#include "pid.h"

#include "perls-common/timestamp.h"
#include "perls-common/lcm_util.h"
#include <lcm/lcm-cpp.hpp>
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/wam_v_control_t.hpp"
#include "perls-lcmtypes++/acfrlcm/asv_torqeedo_motor_command_t.hpp"

using namespace std;

// set the delta T to 0.1s, 10Hz loop rate
#define CONTROL_DT 0.1
//#define W_BEARING 0.95 //amount to weight the velocity bearing (slip angle) in the heading controller, to account for water currents
//#define W_HEADING 0.05 //amount to weight the heading in the heading controller

typedef struct
{
    double vx;
    double heading;
} command_t;

typedef struct
{
    lcm::LCM lcm;

    // controller gains
    pid_gains_t gains_vel;
    pid_gains_t gains_heading;

    // Nav solution
    acfrlcm::auv_acfr_nav_t nav;
    pthread_mutex_t nav_lock;
    int nav_alive;

    // controller inputs
    command_t command;
    int run_mode;
    pthread_mutex_t command_lock;

    // remote
    int64_t remote_time;
    int remote;

    // vehicle name
    string vehicle_name = "DEFAULT";
} state_t;

// load all the config variables
int load_config(state_t *state, char *rootkey)
{
    BotParam *param;
    char key[64];
    param = bot_param_new_from_server(state->lcm.getUnderlyingLCM(), 1);

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

// Trajectory planner callback
static void control_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                             const acfrlcm::wam_v_control_t *control, state_t *state)
{
    pthread_mutex_lock(&state->command_lock);

    state->command.vx = control->vx;
    state->command.heading = control->heading;
    state->run_mode = control->run_mode;

    pthread_mutex_unlock(&state->command_lock);
}

// Remote control callback
void motor_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                    const acfrlcm::asv_torqeedo_motor_command_t *mc, state_t *state)
{
    // we got a remote command, set the time and mode
    if(mc->source == acfrlcm::asv_torqeedo_motor_command_t::REMOTE)
    {
        state->remote_time = mc->utime;
        state->remote = 1;
    }
    else
    {
        state->remote = 0;
    }

}

// ACFR Nav callback, as this program handles its own timing we just make a copy of this data
// every time it comes in
static void acfr_nav_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                              const acfrlcm::auv_acfr_nav_t *nav, state_t *state)
{
    // lock the nav structure and copy the data in, this may introduce a timing problem for the main loop
    pthread_mutex_lock(&state->nav_lock);
    memcpy(&state->nav, nav, sizeof(acfrlcm::auv_acfr_nav_t));
    state->nav_alive = 1;
    pthread_mutex_unlock(&state->nav_lock);
}

void limit_value(double *val, double limit)
{
    if (*val > limit)
        *val = limit;
    else if (*val < -limit)
        *val = -limit;
}

void handle_heartbeat(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const perllcm::heartbeat_t *hb, state_t *state)
{ 
    double speed_control = 0.0;
    double heading_control = 0.0;

    static long loopCount = 0;

    acfrlcm::asv_torqeedo_motor_command_t mc_port;
    acfrlcm::asv_torqeedo_motor_command_t mc_stbd;

    loopCount++;

    // reset the motor command
    memset(&mc_port, 0, sizeof(acfrlcm::asv_torqeedo_motor_command_t));
    memset(&mc_stbd, 0, sizeof(acfrlcm::asv_torqeedo_motor_command_t));

    if( state->remote )
    {
        printf( "Remote is on! Should we (not) do something??\n" );
    }

    if (state->run_mode == acfrlcm::wam_v_control_t::RUN) 
    {
        // lock the nav and command data and get a local copy
        pthread_mutex_lock(&state->nav_lock);
        acfrlcm::auv_acfr_nav_t nav = state->nav;
        pthread_mutex_unlock(&state->nav_lock);

        pthread_mutex_lock(&state->command_lock);
        command_t cmd = state->command;
        pthread_mutex_unlock(&state->command_lock);


        // X Velocity
        speed_control = pid(&state->gains_vel, nav.vx, cmd.vx, CONTROL_DT);

        /*
         * Heading calculation
         * Calculate the diff between desired heading and actual heading.
         * Ensure this diff is between +/-PI
         */
        while (nav.heading < -M_PI)
            nav.heading += 2 * M_PI;
        while (nav.heading > M_PI)
            nav.heading -= 2 * M_PI;

        while (cmd.heading < -M_PI)
            cmd.heading += 2 * M_PI;
        while (cmd.heading > M_PI)
            cmd.heading -= 2 * M_PI;

        double diff_heading = nav.heading - cmd.heading;
        while( diff_heading < -M_PI )
            diff_heading += 2*M_PI;
        while( diff_heading > M_PI )
            diff_heading -= 2*M_PI;

        heading_control = pid(&state->gains_heading, diff_heading, 0.0, CONTROL_DT);

        //printf("hnav:%f, hcmd:%f, rangle:%f p:%.1f s:%.1f \n",
        // state->nav.heading, state->command.heading, speed_control + heading_control, speed_control - heading_control);

        // Set motor controller values
        mc_port.command_speed = speed_control + heading_control;
        mc_stbd.command_speed = speed_control - heading_control;

        // Print out and publish IVER_MOTOR.TOP status message every 10 loops
        if( loopCount % 10 == 0 )
        {
            state->lcm.publish(state->vehicle_name+".PORT_MOTOR_CONTROL.TOP", &mc_port);
            state->lcm.publish(state->vehicle_name+".STBD_MOTOR_CONTROL.TOP", &mc_stbd);
            printf( "Velocity: curr=%2.2f, des=%2.2f, diff=%2.2f\n",
                    nav.vx, cmd.vx, (cmd.vx - nav.vx) );
            printf( "Heading : curr=%3.2f, des=%3.2f, diff=%3.2f\n",
                    nav.heading/M_PI*180, cmd.heading/M_PI*180, diff_heading/M_PI*180 );
            printf( "Port   : %4d\n", (int)mc_port.command_speed);
            printf( "Stbd   : %4d\n", (int)mc_stbd.command_speed);
            printf( "\n" );
        }

    }
    else
    {
        // if we are not in RUN, zero the integral terms
        state->gains_vel.integral = 0;
        state->gains_heading.integral = 0;

        if( loopCount % 10 == 0 )
        {
            printf( "o" );
        }

    }
    mc_port.utime = timestamp_now();
    mc_port.source = acfrlcm::asv_torqeedo_motor_command_t::AUTO;
    state->lcm.publish(state->vehicle_name+".PORT_MOTOR_CONTROL", &mc_port);
    mc_stbd.utime = timestamp_now();
    mc_stbd.source = acfrlcm::asv_torqeedo_motor_command_t::AUTO;
    state->lcm.publish(state->vehicle_name+".STBD_MOTOR_CONTROL", &mc_stbd);
}


// Exit handler
int program_exit;
void signal_handler(int sigNum)
{
    // do a safe exit
    program_exit = 1;
}

void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, state_t *state)
{
    int opt;

    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            state->vehicle_name = (char*)optarg;
            break;
         }
    }
}


int main(int argc, char **argv)
{
    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);

    state_t state;
    //state.lcm = lcm_create(NULL);
    state.run_mode = acfrlcm::wam_v_control_t::STOP;
    state.gains_vel.integral = 0;
    state.gains_heading.integral = 0;
    state.remote = 0;

    char root_key[64];
    sprintf(root_key, "acfr.%s", basename(argv[0]));
    load_config(&state, root_key);

    parse_args(argc, argv, &state);

    // locks etc
    pthread_mutex_init(&state.nav_lock, NULL);
    state.nav_alive = 0;
    memset(&state.nav, 0, sizeof(acfrlcm::auv_acfr_nav_t));

    pthread_mutex_init(&state.command_lock, NULL);
    memset(&state.command, 0, sizeof(command_t));

    // LCM callbacks
    state.lcm.subscribeFunction(state.vehicle_name+".ACFR_NAV", acfr_nav_callback,
                                     &state);
    state.lcm.subscribeFunction(state.vehicle_name+".WAM_V_CONTROL", control_callback,
                                    &state);
    state.lcm.subscribeFunction(state.vehicle_name+".PORT_MOTOR_CONTROL",
            motor_callback, &state);

    state.lcm.subscribeFunction(state.vehicle_name+".STBD_MOTOR_CONTROL",
            motor_callback, &state);

    state.lcm.subscribeFunction("HEARTBEAT_10HZ",
            &handle_heartbeat, &state);

    // Loop
    int fd = state.lcm.getFileno();
    fd_set rfds;
    while (!program_exit)
    {
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int ret = select(fd + 1, &rfds, NULL, NULL, &timeout);
        if (ret > 0)
            state.lcm.handle();
    }

    pthread_mutex_destroy(&state.nav_lock);
    pthread_mutex_destroy(&state.command_lock);

    return 1;
}
