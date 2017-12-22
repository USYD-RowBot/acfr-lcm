#include <signal.h>
#include <libgen.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <bot_param/param_client.h>

#include "pid.h"
#include "timer.h"

#include "acfr-common/timestamp.h"
#include "acfr-common/lcm_util.h"
#include "perls-lcmtypes/acfrlcm_auv_acfr_nav_t.h"
#include "perls-lcmtypes/acfrlcm_auv_control_t.h"
#include "perls-lcmtypes/acfrlcm_auv_nga_motor_command_t.h"
#include "perls-lcmtypes/acfrlcm_auv_spektrum_control_command_t.h"

// set the delta T to 0.1s, 10Hz loop rate
#define CONTROL_DT 0.1
//#define W_BEARING 0.95 //amount to weight the velocity bearing (slip angle) in the heading controller, to account for water currents
//#define W_HEADING 0.05 //amount to weight the heading in the heading controller


// RC constants
#define RC_OFFSET 1024
#define RC_THROTTLE_OFFSET 1024     // Testing 16092016 JJM
#define RC_HALF_RANGE 685
#define RC_TO_RAD (12*M_PI/180)/RC_HALF_RANGE
#define RC_TO_RPM 8              // Mitch
#define RC_MAX_PROP_RPM 1500
#define RC_DEADZONE 80 // Testing 160902016 JJM
#define RCMULT RC_MAX_PROP_RPM/(RC_HALF_RANGE-RC_DEADZONE)
#define RC_TUNNEL_MULTI 2047/(RC_HALF_RANGE)
// RC Channel names
enum
{
    RC_THROTTLE = 0,
    RC_AILERON,
    RC_ELEVATOR,
    RC_RUDDER,
    RC_GEAR,
    RC_AUX1,
};


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
    lcm_t *lcm;

    // controller gains
    pid_gains_t gains_vel;
    pid_gains_t gains_roll;
    pid_gains_t gains_depth;
    pid_gains_t gains_altitude;
    pid_gains_t gains_pitch;
    pid_gains_t gains_pitch_r;
    pid_gains_t gains_heading;
    pid_gains_t gains_tunnel_depth;
    pid_gains_t gains_tunnel_descent;
    pid_gains_t gains_tunnel_pitch;

    // Nav solution
    acfrlcm_auv_acfr_nav_t nav;
    pthread_mutex_t nav_lock;
    int nav_alive;

    // controller inputs
    command_t command;
    int run_mode;
    pthread_mutex_t command_lock;

    // remote
    acfrlcm_auv_spektrum_control_command_t spektrum_command;
    pthread_mutex_t spektrum_lock;
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

    // Heading gains
    memset(&state->gains_tunnel_depth, 0, sizeof(pid_gains_t));
    sprintf(key, "%s.tunnel_depth.kp", rootkey);
    state->gains_tunnel_depth.kp = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.tunnel_depth.ki", rootkey);
    state->gains_tunnel_depth.ki = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.tunnel_depth.kd", rootkey);
    state->gains_tunnel_depth.kd = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.tunnel_depth.sat", rootkey);
    state->gains_tunnel_depth.sat = bot_param_get_double_or_fail(param, key);

    // Heading gains
    memset(&state->gains_tunnel_descent, 0, sizeof(pid_gains_t));
    sprintf(key, "%s.tunnel_descent.kp", rootkey);
    state->gains_tunnel_descent.kp = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.tunnel_descent.ki", rootkey);
    state->gains_tunnel_descent.ki = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.tunnel_descent.kd", rootkey);
    state->gains_tunnel_descent.kd = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.tunnel_descent.sat", rootkey);
    state->gains_tunnel_descent.sat = bot_param_get_double_or_fail(param, key);

    // Heading gains
    memset(&state->gains_tunnel_pitch, 0, sizeof(pid_gains_t));
    sprintf(key, "%s.tunnel_pitch.kp", rootkey);
    state->gains_tunnel_pitch.kp = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.tunnel_pitch.ki", rootkey);
    state->gains_tunnel_pitch.ki = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.tunnel_pitch.kd", rootkey);
    state->gains_tunnel_pitch.kd = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.tunnel_pitch.sat", rootkey);
    state->gains_tunnel_pitch.sat = bot_param_get_double_or_fail(param, key);

    return 1;
}




// Trajectory planner callback
static void control_callback(const lcm_recv_buf_t *rbuf, const char *channel,
                             const acfrlcm_auv_control_t *control, void *user)
{
    state_t *state = (state_t *) user;
    pthread_mutex_lock(&state->command_lock);

    state->command.vx = control->vx;
    state->command.heading = control->heading;
    state->command.depth = control->depth;
    state->command.altitude = control->altitude;
    state->command.pitch = control->pitch;
    state->run_mode = control->run_mode;

    switch (control->depth_mode)
    {
    case ACFRLCM_AUV_CONTROL_T_DEPTH_MODE:
        state->command.depth_mode = DEPTH_MODE;
        break;
    case ACFRLCM_AUV_CONTROL_T_ALTITUDE_MODE:
        state->command.depth_mode = ALTITUDE_MODE;
        break;
    case ACFRLCM_AUV_CONTROL_T_PITCH_MODE:
        state->command.depth_mode = PITCH_MODE;
        break;
    }

    pthread_mutex_unlock(&state->command_lock);
}

// Remote control callback
void spektrum_control_callback(const lcm_recv_buf_t *rbuf, const char *ch,
                    const acfrlcm_auv_spektrum_control_command_t *mc, void *u)
{
    state_t *state = (state_t *)u;

    // Save a copy of the data, process it in the main loop
    pthread_mutex_lock(&state->spektrum_lock);
    memcpy(&state->spektrum_command, mc, sizeof(acfrlcm_auv_spektrum_control_command_t));
    pthread_mutex_unlock(&state->spektrum_lock);
}

// ACFR Nav callback, as this program handles its own timing we just make a copy of this data
// every time it comes in
static void acfr_nav_callback(const lcm_recv_buf_t *rbuf, const char *channel,
                              const acfrlcm_auv_acfr_nav_t *nav, void *user)
{
    state_t *state = (state_t *) user;

    // lock the nav structure and copy the data in, this may introduce a timing problem for the main loop
    pthread_mutex_lock(&state->nav_lock);
    memcpy(&state->nav, nav, sizeof(acfrlcm_auv_acfr_nav_t));
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
    memset(&state, 0, sizeof(state_t));
    state.lcm = lcm_create(NULL);
    state.run_mode = ACFRLCM_AUV_CONTROL_T_STOP;
    state.gains_vel.integral = 0;
    state.gains_roll.integral = 0;
    state.gains_depth.integral = 0;
    state.gains_altitude.integral = 0;
    state.gains_pitch.integral = 0;
    state.gains_pitch_r.integral = 0;
    state.gains_heading.integral = 0;
    state.gains_tunnel_depth.integral = 0;
    state.gains_tunnel_descent.integral = 0;
    state.gains_tunnel_pitch.integral = 0;

    
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
    memset(&state.nav, 0, sizeof(acfrlcm_auv_acfr_nav_t));

    pthread_mutex_init(&state.command_lock, NULL);
    memset(&state.command, 0, sizeof(command_t));

    pthread_mutex_init(&state.spektrum_lock, NULL);

    // LCM callbacks
    acfrlcm_auv_acfr_nav_t_subscribe(state.lcm, "ACFR_NAV.*", &acfr_nav_callback,
                                     &state);
    acfrlcm_auv_control_t_subscribe(state.lcm, "AUV_CONTROL", &control_callback,
                                    &state);
    acfrlcm_auv_spektrum_control_command_t_subscribe(state.lcm, "SPEKTRUM_CONTROL",
            &spektrum_control_callback, &state);

    periodic_info timer_info;
    make_periodic(CONTROL_DT * 1000000, &timer_info);

    double prop_rpm = 0.0;
    double pitch = 0.0, plane_angle = 0.0, rudder_angle = 0.0;

    long loopCount = 0;

    acfrlcm_auv_nga_motor_command_t mc;

    // main loop
    while (!main_exit)
    {
        loopCount++;

        pthread_mutex_lock(&state.spektrum_lock);
	acfrlcm_auv_spektrum_control_command_t spektrum_command = state.spektrum_command;
        pthread_mutex_unlock(&state.spektrum_lock);

        // reset the motor command
        memset(&mc, 0, sizeof(acfrlcm_auv_nga_motor_command_t));
	// Check for remote mode
	if(timestamp_now() - spektrum_command.utime < 1e6)
	{
	    // Lateral tunel thrusters
	    int fore = 0;
	    int aft = 0;
	    double rudder;

	    fore = (spektrum_command.values[RC_RUDDER] - RC_OFFSET) * RC_TUNNEL_MULTI;
	    aft= (spektrum_command.values[RC_RUDDER] - RC_OFFSET) * RC_TUNNEL_MULTI;
            
	    // Check the steering mode switch
	    if(spektrum_command.values[RC_GEAR] > 1200)
	    {
	        fore += (spektrum_command.values[RC_AILERON] - RC_OFFSET) * RC_TUNNEL_MULTI;
	        aft -= (spektrum_command.values[RC_AILERON] - RC_OFFSET) * RC_TUNNEL_MULTI;
		rudder = 0;
	    }
	    else 
	        rudder = -(spektrum_command.values[RC_AILERON] - RC_OFFSET) * RC_TO_RAD;

            mc.tail_elevator = -(spektrum_command.values[RC_ELEVATOR] - RC_OFFSET) * RC_TO_RAD;
            mc.tail_rudder = rudder; 
	    mc.lat_aft = aft;
	    mc.lat_fore = fore; 
	    
	    
	    int rcval = spektrum_command.values[RC_THROTTLE] - RC_THROTTLE_OFFSET;
            
	    // We are in remote mode as the timestamp difference is less then 1 second
	    // if in deadzone at centre
            if (abs(rcval) < RC_DEADZONE)
            {
                mc.tail_thruster = 0;
            }
            else
            {
                mc.tail_thruster = (rcval - RC_DEADZONE) * RCMULT;
                if (mc.tail_thruster > RC_MAX_PROP_RPM)
                    mc.tail_thruster = RC_MAX_PROP_RPM;
                else if (mc.tail_thruster < -RC_MAX_PROP_RPM)
                    mc.tail_thruster = -RC_MAX_PROP_RPM;
            }        





	}        
	else if (state.run_mode == ACFRLCM_AUV_CONTROL_T_RUN)// ||
                //state.run_mode == ACFRLCM_AUV_CONTROL_T_DIVE)
        {
            // lock the nav and command data and get a local copy
            pthread_mutex_lock(&state.nav_lock);
            acfrlcm_auv_acfr_nav_t nav = state.nav;
            pthread_mutex_unlock(&state.nav_lock);

            pthread_mutex_lock(&state.command_lock);
            command_t cmd = state.command;
            pthread_mutex_unlock(&state.command_lock);


            // X Velocity
            prop_rpm = pid(&state.gains_vel, nav.vx, cmd.vx, CONTROL_DT);

            // Pitch to fins
            if (cmd.depth_mode == PITCH_MODE)
            {
                pitch = cmd.pitch;
            }
            // Altitude to pitch
            // Invert sign of pitch reference to reflect pitch
            // orientation
            else if (state.command.depth_mode == ALTITUDE_MODE)
                pitch = -pid(&state.gains_altitude, nav.altitude, cmd.altitude,
                             CONTROL_DT);
            // Depth to pitch mode
            else
                pitch = -pid(&state.gains_depth, nav.depth, cmd.depth,
                             CONTROL_DT);

            if ((nav.vx > -0.05) || (prop_rpm > -100))
                plane_angle = pid(&state.gains_pitch, nav.pitch, pitch,
                                  CONTROL_DT);
            else
                plane_angle = pid(&state.gains_pitch_r, nav.pitch, pitch,
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

             if (fabs(state.nav.vx) < 0.2) // at 0.2 m/s assume velocity vector magnitude is accurate relative to error
             W_BEARING = fabs(state.nav.vx) / 0.2;
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

            //printf("bearing: %f heading: %f bearing_w: %f\n",bearing,state.nav.heading,bearing_weighted);
#endif

            // Special dive case, no heading control
            if (state.run_mode == ACFRLCM_AUV_CONTROL_T_DIVE)
            {
                rudder_angle = 0;
            }
            else
            {

                // Account for side slip by making the velocity bearing weighted
                // 	on the desired heading
                rudder_angle = pid(&state.gains_heading, diff_heading, 0.0, CONTROL_DT);
            }


            //	printf("prop_rpm: %f\n",prop_rpm);
            // Reverse all the fin angles for reverse direction (given rpm is
            // 	negative and so is velocity, so water relative should be
            //	negative, or soon will be). May not be enough due to completely
            //	different dynamics in reverse, hence there are new gains for the
            //	reverse pitch control now.
            if ((nav.vx < -0.05) && (prop_rpm < -100))
            {
                printf("reversing, flipping fin control\n");
                rudder_angle       = -rudder_angle;
                plane_angle      = -plane_angle;
            }

            //printf("hnav:%f, hcmd:%f, rangle:%f r:%.1f p:%.1f \n",
            // state.nav.heading, state.command.heading, rudder_angle, plane_angle);

            // Set motor controller values
            mc.tail_thruster = prop_rpm;
            mc.tail_rudder = rudder_angle;
            mc.tail_elevator = plane_angle;
            mc.vert_fore = 0;
            mc.vert_aft = 0;
            mc.lat_fore = 0;
            mc.lat_aft = 0;


            // Print out and publish NEXTGEN_MOTOR.TOP status message every 10 loops
            if( loopCount % 10 == 0 )
            {
                acfrlcm_auv_nga_motor_command_t_publish(state.lcm, "NEXTGEN_MOTOR.TOP", &mc);
                printf( "Velocity: curr=%2.2f, des=%2.2f, diff=%2.2f\n",
                        nav.vx, cmd.vx, (cmd.vx - nav.vx) );
                printf( "Heading : curr=%3.2f, des=%3.2f, diff=%3.2f\n",
                        nav.heading/M_PI*180, cmd.heading/M_PI*180, diff_heading/M_PI*180 );
                printf( "Pitch : curr=%3.2f, des=%3.2f, diff=%3.2f\n",
                        nav.pitch/M_PI*180, pitch/M_PI*180, (pitch - nav.pitch)/M_PI*180 );
                printf( "Roll: curr=%3.2f, des=%3.2f, diff=%3.2f \n",
                        nav.roll/M_PI*180, 0.0, -nav.roll/M_PI*180 );
                printf( "Motor   : main=%4d\n", (int)mc.tail_thruster);
                printf( "Fins    : rudder_angle=%.2f, plane_angle=%.2f \n",
                        mc.tail_rudder, mc.tail_elevator);
                printf( "\n" );
            }

        }
        if (state.run_mode == ACFRLCM_AUV_CONTROL_T_DIVE)
        {
            // diving with NGA is a little special. we have tunnel thrusters
            // to reach a target depth
            pthread_mutex_lock(&state.nav_lock);
            acfrlcm_auv_acfr_nav_t nav = state.nav;
            pthread_mutex_unlock(&state.nav_lock);

            pthread_mutex_lock(&state.command_lock);
            command_t cmd = state.command;
            pthread_mutex_unlock(&state.command_lock);

            double target_pitch = 0.0;

            double target_descent = pid(&state.gains_tunnel_depth, 
                    nav.depth, cmd.depth, CONTROL_DT);
            double differential = pid(&state.gains_tunnel_pitch,
                    nav.pitch, target_pitch, CONTROL_DT);
            double mutual = -pid(&state.gains_tunnel_descent,
                    nav.vz, target_descent, CONTROL_DT);

            // Set motor controller values
            mc.tail_thruster = 0;
            mc.tail_rudder = 0;
            mc.tail_elevator = 0;
            mc.vert_fore = mutual + differential;
            mc.vert_aft = mutual - differential;
            mc.lat_fore = 0;
            mc.lat_aft = 0;


            // Print out and publish NEXTGEN_MOTOR.TOP status message every 10 loops
            if( loopCount % 10 == 0 )
            {
                acfrlcm_auv_nga_motor_command_t_publish(state.lcm, "NEXTGEN_MOTOR.TOP", &mc);
                printf( "Depth: curr=%2.2f, des=%2.2f, diff=%2.2f\n",
                        nav.depth, cmd.depth, (cmd.depth - nav.depth) );
                printf( "Descent Rate: curr=%2.2f, des=%2.2f, diff=%2.2f\n",
                        nav.vz, target_descent, (target_descent - nav.vz));
                printf( "Pitch : curr=%3.2f, des=%3.2f, diff=%3.2f\n",
                        nav.pitch/M_PI*180, target_pitch/M_PI*180, (target_pitch - nav.pitch)/M_PI*180 );
                printf( "Vert Tunnel: fore=%.2f, aft=%.2f \n",
                        mc.vert_fore, mc.vert_aft);
                printf( "\n" );
            }
        }
        else
        {
            // if we are not in RUN or DIVE mode, zero the integral terms
            state.gains_vel.integral = 0;
            state.gains_roll.integral = 0;
            state.gains_depth.integral = 0;
            state.gains_altitude.integral = 0;
            state.gains_pitch.integral = 0;
            state.gains_pitch_r.integral = 0;
            state.gains_heading.integral = 0;
            state.gains_tunnel_depth.integral = 0;
            state.gains_tunnel_descent.integral = 0;
            state.gains_tunnel_pitch.integral = 0;

            if( loopCount % 10 == 0 )
            {
                printf( "o" );
            }

        }
        mc.utime = timestamp_now();
        acfrlcm_auv_nga_motor_command_t_publish(state.lcm, "NGA_MOTOR", &mc);

        // wait for a timer event
        wait_period(&timer_info);
    }

    pthread_join(lcm_tid, NULL);
    pthread_mutex_destroy(&state.nav_lock);
    pthread_mutex_destroy(&state.command_lock);

    return 1;
}
