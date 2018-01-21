#include <string>
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
#include <lcm/lcm-cpp.hpp>
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_control_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_control_goal_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_iver_motor_command_t.hpp"

using namespace std;

// set the delta T to 0.1s, 10Hz loop rate
#define CONTROL_DT 0.1
//#define W_BEARING 0.95 //amount to weight the velocity bearing (slip angle) in the heading controller, to account for water currents
//#define W_HEADING 0.05 //amount to weight the heading in the heading controller

typedef enum
{
    DEPTH_MODE, PITCH_MODE, ALTITUDE_MODE
} depth_mode_t;

typedef struct
{
    double vx;
    double depth;
    double pitch;
    double heading;
    double altitude;
    depth_mode_t depth_mode;
} command_t;

typedef struct
{
    lcm::LCM lcm;

    // controller gains
    pid_gains_t gains_vel;
    pid_gains_t gains_roll;
    pid_gains_t gains_depth;
    pid_gains_t gains_altitude;
    pid_gains_t gains_pitch;
    pid_gains_t gains_pitch_r;
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

    // Roll gains
    memset(&state->gains_roll, 0, sizeof(pid_gains_t));
    sprintf(key, "%s.roll.kp", rootkey);
    state->gains_roll.kp = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.roll.ki", rootkey);
    state->gains_roll.ki = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.roll.kd", rootkey);
    state->gains_roll.kd = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.roll.sat", rootkey);
    state->gains_roll.sat = bot_param_get_double_or_fail(param, key);

    // Pitch gains
    memset(&state->gains_pitch, 0, sizeof(pid_gains_t));
    sprintf(key, "%s.pitch.kp", rootkey);
    state->gains_pitch.kp = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.pitch.ki", rootkey);
    state->gains_pitch.ki = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.pitch.kd", rootkey);
    state->gains_pitch.kd = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.pitch.sat", rootkey);
    state->gains_pitch.sat = bot_param_get_double_or_fail(param, key);

    //Pitch reverse gains
    memset(&state->gains_pitch_r, 0, sizeof(pid_gains_t));
    sprintf(key, "%s.pitch_r.kp", rootkey);
    state->gains_pitch_r.kp = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.pitch_r.ki", rootkey);
    state->gains_pitch_r.ki = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.pitch_r.kd", rootkey);
    state->gains_pitch_r.kd = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.pitch_r.sat", rootkey);
    state->gains_pitch_r.sat = bot_param_get_double_or_fail(param, key);

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

// Trajectory planner callback
static void control_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                             const acfrlcm::auv_control_t *control, state_t *state)
{
    //state_t *state = (state_t *)s;
    pthread_mutex_lock(&state->command_lock);

    state->command.vx = control->vx;
    state->command.heading = control->heading;
    state->command.depth = control->depth;
    state->command.altitude = control->altitude;
    state->command.pitch = control->pitch;
    state->run_mode = control->run_mode;

    switch (control->depth_mode)
    {
    case acfrlcm::auv_control_t::DEPTH_MODE:
        state->command.depth_mode = DEPTH_MODE;
        break;
    case acfrlcm::auv_control_t::ALTITUDE_MODE:
        state->command.depth_mode = ALTITUDE_MODE;
        break;
    case acfrlcm::auv_control_t::PITCH_MODE:
        state->command.depth_mode = PITCH_MODE;
        break;
    }

    pthread_mutex_unlock(&state->command_lock);
}

// Remote control callback
void motor_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                    const acfrlcm::auv_iver_motor_command_t *mc, state_t *state)
{
    //state_t *state = (state_t *)s;

    // we got a remote command, set the time and mode
    if(mc->source == acfrlcm::auv_iver_motor_command_t::REMOTE)
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
    //state_t *state = (state_t *)s;

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
    double prop_rpm = 0.0;
    double roll_offset = 0.0;
    double pitch = 0.0, plane_angle = 0.0, rudder_angle = 0.0;

    static long loopCount = 0;

    acfrlcm::auv_iver_motor_command_t mc;
    acfrlcm::auv_control_goal_t cg;

    loopCount++;

    // reset the motor command
    memset(&mc, 0, sizeof(acfrlcm::auv_iver_motor_command_t));

    if( state->remote )
    {
        printf( "Remote is on! Should we (not) do something??\n" );
    }

    if (state->run_mode == acfrlcm::auv_control_t::RUN ||
            state->run_mode == acfrlcm::auv_control_t::DIVE)
    {
        // lock the nav and command data and get a local copy
        pthread_mutex_lock(&state->nav_lock);
        acfrlcm::auv_acfr_nav_t nav = state->nav;
        pthread_mutex_unlock(&state->nav_lock);

        pthread_mutex_lock(&state->command_lock);
        command_t cmd = state->command;
        pthread_mutex_unlock(&state->command_lock);


        // X Velocity
        prop_rpm = pid(&state->gains_vel, nav.vx, cmd.vx, CONTROL_DT);

        // Pitch to fins
        if (cmd.depth_mode == PITCH_MODE)
        {
            pitch = cmd.pitch;
        }
        // Altitude to pitch
        // Invert sign of pitch reference to reflect pitch
        // orientation
        else if (state->command.depth_mode == ALTITUDE_MODE)
            pitch = -pid(&state->gains_altitude, nav.altitude, cmd.altitude,
                         CONTROL_DT);
        // Depth to pitch mode
        else
            pitch = -pid(&state->gains_depth, nav.depth, cmd.depth,
                         CONTROL_DT);



        if ((nav.vx > -0.05) || (prop_rpm > -100))
            plane_angle = pid(&state->gains_pitch, nav.pitch, pitch,
                              CONTROL_DT);
        else
            plane_angle = pid(&state->gains_pitch_r, nav.pitch, pitch,
                              CONTROL_DT);

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

#if 0
        double bearing = atan2(
                             nav.vy * cos(nav.heading) + nav.vx * sin(nav.heading),
                             -nav.vy * sin(nav.heading) + nav.vx * cos(nav.heading));

        while (bearing < -M_PI)
            bearing += 2 * M_PI;
        while (bearing > M_PI)
            bearing -= 2 * M_PI;


        // correctly compute the weighted bearing
        double yaw1 = bearing;
        if (yaw1 > 2 * M_PI
           )
            yaw1 -= 2 * M_PI;
        else if (yaw1 < 0)
            yaw1 += 2 * M_PI;

        double yaw2 = nav.heading;
        if (yaw2 > 2 * M_PI
           )
            yaw2 -= 2 * M_PI;
        else if (yaw2 < 0)
            yaw2 += 2 * M_PI;

        if (yaw2 - yaw1 > M_PI
           )
            yaw2 -= 2 * M_PI;
        else if (yaw1 - yaw2 > M_PI
                )
            yaw1 -= 2 * M_PI;

        //Weight the heading more as the velocity magnitude decreases

        double W_BEARING = 0;
        /* for the moment, use the heading as the yaw reference

         if (fabs(state->nav.vx) < 0.2) // at 0.2 m/s assume velocity vector magnitude is accurate relative to error
         W_BEARING = fabs(state->nav.vx) / 0.2;
         else
         W_BEARING = 1;
         */
        double W_HEADING = 1 - W_BEARING;

        double bearing_weighted = W_BEARING * yaw1 + W_HEADING * yaw2;

        while (bearing_weighted < -M_PI)
            bearing_weighted += 2 * M_PI;
        while (bearing_weighted > M_PI)
            bearing_weighted -= 2 * M_PI;

        // If the sign of the heading is not the same as the sign of the
        //		bearing...
        if ((int) (fabs(cmd.heading) / cmd.heading)
                != (int) (fabs(bearing_weighted) / bearing_weighted))
        {
            if (cmd.heading < (-M_PI / 2))
                cmd.heading += 2 * M_PI;
            else if (bearing_weighted < (-M_PI / 2))
                bearing_weighted += 2 * M_PI;
        }

            //printf("bearing: %f heading: %f bearing_w: %f\n",bearing,state->nav.heading,bearing_weighted);
#endif

        // Special dive case, no heading control
        if (state->run_mode == acfrlcm::auv_control_t::DIVE)
        {
            rudder_angle = 0;
            roll_offset = 0;
        }
        else
        {

            // Account for side slip by making the velocity bearing weighted
            // 	on the desired heading
            rudder_angle = pid(&state->gains_heading, diff_heading, 0.0, CONTROL_DT);

            // Roll compenstation
            // We try to keep the AUV level, ie roll = 0
            roll_offset = pid(&state->gains_roll, nav.roll, 0.0, CONTROL_DT);
        }

        // Add in the roll offset
        double top       = rudder_angle - roll_offset;
        double bottom    = rudder_angle + roll_offset;
        double port      = plane_angle  - roll_offset;
        double starboard = plane_angle  + roll_offset;

        //	printf("prop_rpm: %f\n",prop_rpm);
        // Reverse all the fin angles for reverse direction (given rpm is
        // 	negative and so is velocity, so water relative should be
        //	negative, or soon will be). May not be enough due to completely
        //	different dynamics in reverse, hence there are new gains for the
        //	reverse pitch control now.
        if ((nav.vx < -0.05) && (prop_rpm < -100))
        {
            printf("reversing, flipping fin control\n");
            top       = -top;
            bottom    = -bottom;
            port      = -port;
            starboard = -starboard;
        }

        //printf("hnav:%f, hcmd:%f, rangle:%f t:%.1f b:%.1f p:%.1f s:%.1f\n",
        // state->nav.heading, state->command.heading, rudder_angle, top, bottom, port, starboard);

        // Set motor controller values
        mc.main = prop_rpm;
        mc.top = top;
        mc.bottom = bottom;
        mc.port = port;
        mc.starboard = starboard;

        // Print out and publish IVER_MOTOR.TOP status message every 10 loops
        if( loopCount % 10 == 0 )
        {
            state->lcm.publish("IVER_MOTOR.TOP."+state->vehicle_name, &mc);
            printf( "Velocity: curr=%2.2f, des=%2.2f, diff=%2.2f\n",
                    nav.vx, cmd.vx, (cmd.vx - nav.vx) );
            printf( "Heading : curr=%3.2f, des=%3.2f, diff=%3.2f\n",
                    nav.heading/M_PI*180, cmd.heading/M_PI*180, diff_heading/M_PI*180 );
            printf( "Pitch : curr=%3.2f, des=%3.2f, diff=%3.2f\n",
                    nav.pitch/M_PI*180, pitch/M_PI*180, (pitch - nav.pitch)/M_PI*180 );
            printf( "Roll: curr=%3.2f, des=%3.2f, diff=%3.2f offset: %3.2f\n",
                    nav.roll/M_PI*180, 0.0, -nav.roll/M_PI*180, roll_offset/M_PI*180 );
            printf( "Motor   : main=%4d\n", (int)mc.main);
            printf( "Fins    : top=%.2f, bot=%.2f, port=%.2f star=%.2f\n",
                    mc.top, mc.bottom, mc.port, mc.starboard);
            printf( "\n" );
        }

    }
    else
    {
        // if we are not in RUN or DIVE mode, zero the integral terms
        state->gains_vel.integral = 0;
        state->gains_roll.integral = 0;
        state->gains_depth.integral = 0;
        state->gains_altitude.integral = 0;
        state->gains_pitch.integral = 0;
        state->gains_pitch_r.integral = 0;
        state->gains_heading.integral = 0;

        if( loopCount % 10 == 0 )
        {
            printf( "o" );
        }

    }
    mc.utime = timestamp_now();
    cg.utime = timestamp_now();
    cg.pitch_goal = pitch;
    mc.source = acfrlcm::auv_iver_motor_command_t::AUTO;
    state->lcm.publish("IVER_MOTOR."+state->vehicle_name, &mc);
    state->lcm.publish("CONTROL_GOAL."+state->vehicle_name, &cg);
}


// Exit handler
int program_exit;
void signal_handler(int sigNum)
{
    // do a safe exit
    program_exit = 1;
}

// Process LCM messages with callbacks
/*static void *lcm_thread(void *context)
{
    state_t *state = (state_t *) context;

    while (!program_exit)
    {
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        lcmu_handle_timeout(&state->lcm, &tv);
    }
    return 0;
}*/

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
    state.run_mode = acfrlcm::auv_control_t::STOP;
    state.gains_vel.integral = 0;
    state.gains_roll.integral = 0;
    state.gains_depth.integral = 0;
    state.gains_altitude.integral = 0;
    state.gains_pitch.integral = 0;
    state.gains_pitch_r.integral = 0;
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
    state.lcm.subscribeFunction("ACFR_NAV."+state.vehicle_name, acfr_nav_callback,
                                     &state);
    state.lcm.subscribeFunction("AUV_CONTROL."+state.vehicle_name, control_callback,
                                    &state);
    state.lcm.subscribeFunction("IVER_MOTOR."+state.vehicle_name,
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
