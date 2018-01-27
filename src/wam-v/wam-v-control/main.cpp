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
#include "perls-lcmtypes++/acfrlcm/auv_spektrum_control_command_t.hpp"
#include "perls-lcmtypes++/acfrlcm/asv_torqeedo_motor_command_t.hpp"
#include "perls-lcmtypes++/acfrlcm/relay_status_t.hpp"
#include "perls-lcmtypes++/acfrlcm/relay_command_t.hpp"
#include "acfr-common/spektrum-control.h"

using namespace std;

// set the delta T to 0.1s, 10Hz loop rate
#define CONTROL_DT 0.1
//#define W_BEARING 0.95 //amount to weight the velocity bearing (slip angle) in the heading controller, to account for water currents
//#define W_HEADING 0.05 //amount to weight the heading in the heading controller
#define NUM_RELAYS 24                   // Number of relays on relay board
#define TORQEEDO_CTRL_MAX 1000       // Max input control value for Torqeedo Motors

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
    int64_t prev_remote_time;
    enum rc_control_source_t control_source; // RC/ZERO/AUTO
	acfrlcm::auv_spektrum_control_command_t spektrum_msg;

    // vehicle name
    string vehicle_name = "DEFAULT";

    // motor relay
    uint8_t torqeedo_relay_no = 0; // read from the relay config
    bool torqeedo_motors_relay_enabled = 0; // how we control it
    bool torqeedo_motors_relay_reported = 0; // what it reports

	// motor speed limiting
	double motor_limit_rc = 0;
	double motor_limit_controller = 0;

	// debug verbose 
	bool verbose = 0;

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

    // Motor RPM Limiting
    sprintf(key, "%s.motor_limit_rc", rootkey);
    state->motor_limit_rc = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.motor_limit_controller", rootkey);
    state->motor_limit_controller = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.verbose", rootkey);
    state->verbose = bot_param_get_str_or_fail(param, key);

	// get relay number for torqeedos from relay device list
	char *relay_device;
    for (int i = 1; i <= NUM_RELAYS; i++)
    {
		sprintf(key, "dS2824_relays.relay_%d", i);
        //printf("Requesting Parameter: %s\n", key);
        relay_device = bot_param_get_str_or_fail(param, key);
		if (strcmp(relay_device, "torqeedos_contactor") == 0)
		{
			state->torqeedo_relay_no = i;
		    if (state->verbose)
		    {
		        printf("Torqeedo Contactor Relay Number: %d\n", i);
		    }

			continue;	// found, don't check further entries
		}
    }

    if (state->verbose)
    {
        printf("Config setup complete\n");
    }

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

// Remote control callback incoming RC message
void rc_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                    const acfrlcm::auv_spektrum_control_command_t *spektrum_cmd, state_t *state)
{
	if (spektrum_cmd->utime > state->prev_remote_time) // msg is newer than previous msg
	{
		state->prev_remote_time = spektrum_cmd->utime; // we got a remote command, set the time and mode
		memcpy(&state->spektrum_msg, spektrum_cmd, sizeof(acfrlcm::auv_spektrum_control_command_t));
		//state->spektrum_msg = spektrum_cmd;

	    if (spektrum_cmd->values[RC_AUX1] > REAR_POS_CUTOFF)
	    {
			state->control_source = RC_MODE_RC; // vehicle controlled by handheld remote control
		    if (state->verbose)
		    {
		        printf("\nRC_MODE_RC\n");
		    }
	    }
	    else if (spektrum_cmd->values[RC_AUX1] > CENTER_POS_CUTOFF)
	    {
			state->control_source = RC_MODE_ZERO; // disable motors
		    if (state->verbose)
		    {
		        printf("\nRC_MODE_ZERO\n");
		    }
	    }
		else
		{
			state->control_source = RC_MODE_AUTO; // vehicle controlled by wam-v-control 
		    if (state->verbose)
		    {
		        printf("\nRC_MODE_AUTO\n");
		    }
		}
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

void handle_relay_status(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const acfrlcm::relay_status_t *relay_status, state_t *state)
{
    // status comes in as an array of 4 bytes, each bit represents a relay or i/o
    div_t divresult;
    divresult = div((state->torqeedo_relay_no - 1), 8); // relays 1..24, bits 0..7
    state->torqeedo_motors_relay_reported = (relay_status->state_list[divresult.quot]) && (char) pow(2,divresult.rem); //TODO test
}

void send_relay_cmd(state_t *state, bool enable)
{
    if (state->verbose)
    {
        printf("Send Relay Cmd: No. %d %d\n", state->torqeedo_relay_no, (int)enable);
    }
	// publish LCM relay state request command	
	acfrlcm::relay_command_t request_msg; // create new message
    memset(&request_msg, 0, sizeof(acfrlcm::relay_command_t)); // initialise
    // set msg relay request values
	request_msg.relay_number = state->torqeedo_relay_no; 
	request_msg.relay_request = enable;
    // set non-used values to defaults
	request_msg.relay_off_delay = 0; // no off delay
	request_msg.io_number = 0; // no io
	request_msg.io_request = 0;
	state->lcm.publish(state->vehicle_name+".RELAY_CONTROL", &request_msg); 
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

	if (state->control_source == RC_MODE_RC)
	{
		// TODO - transate RC commands to motor commands
        printf( "Remote is on! Should we (not) do something??\n" );
        state->torqeedo_motors_relay_enabled = true;
        if (state->torqeedo_motors_relay_reported == false)
        {
            // turn on relay
			send_relay_cmd(state, true);
            fprintf(stderr, "Entering RC mode - enable torqeedo motors relay\n");
        }

        // check gear (drive control mode assignment switch) of remote (3 position switch top RHS)
        if (state->spektrum_msg.values[RC_GEAR] > REAR_POS_CUTOFF) // Rear Switch Position
        {
            // Differential Mode: Left Joystick (up/down): Fwd/Rev Port Motor
            //                    Right Joysick (up/down): Fwd/Rev Stbd Motor 
            int16_t port_throttle = state->spektrum_msg.values[RC_THROTTLE];
            if ((port_throttle < CENTR_POS + HALF_DEADZONE) && (port_throttle > CENTR_POS - HALF_DEADZONE))
            {
                 mc_port.command_speed = 0; // close to centre so no thrust
            }
            else
            {
                port_throttle-= CENTR_POS; //centre to zero
                if (port_throttle > 0) 
                {
                    port_throttle-= HALF_DEADZONE; // adjust for deadzone
                }
                else
                {
                    port_throttle+= HALF_DEADZONE;
                }
                mc_port.command_speed = (int)((port_throttle/AVAIL_V_RANGE)*TORQEEDO_CTRL_MAX);
			}
			int16_t stbd_throttle = state->spektrum_msg.values[RC_ELEVATOR];
			if ((stbd_throttle < CENTR_POS + HALF_DEADZONE) && (stbd_throttle > CENTR_POS - HALF_DEADZONE))
            {
                 mc_stbd.command_speed = 0; // close to centre so no thrust
            }
            else
            {
                stbd_throttle-= CENTR_POS; //centre to zero
                if (stbd_throttle > 0)
                {
                    stbd_throttle-= HALF_DEADZONE; // adjust for deadzone
                }
                else
                {
                    stbd_throttle+= HALF_DEADZONE;
                }
                mc_stbd.command_speed = (int)((stbd_throttle/AVAIL_V_RANGE)*TORQEEDO_CTRL_MAX);
			}
        }
        else if (state->spektrum_msg.values[RC_GEAR] > CENTER_POS_CUTOFF) // Center Switch Position
        {
            // Do nothing
        }
        else // Front Switch Position
        {
            // Steering Mode: Left Joystick (up/down): Fwd/Rev Both Motors
            //                Right Joystick (left/right): Port/Stbd Steering (both motors)
            

        }


        // apply proportional speed limiting - limit format percentage
        mc_port.command_speed = (int)(state->motor_limit_rc/100 * mc_port.command_speed);
        mc_stbd.command_speed = (int)(state->motor_limit_rc/100 * mc_stbd.command_speed);

    }
	else if ((state->control_source == RC_MODE_AUTO) && (state->run_mode == acfrlcm::wam_v_control_t::RUN))
	{
        // enable motors
		state->torqeedo_motors_relay_enabled = true;
        if (state->torqeedo_motors_relay_reported == false)
        {
            // turn on relay
			send_relay_cmd(state, true);
			fprintf(stderr, "Entering AUTO and RUN mode - enable torqeedo motors relay\n");
        }
        
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

		// apply proportional speed limiting - limit format percentage
        mc_port.command_speed = (int)(state->motor_limit_controller/100 * mc_port.command_speed);
        mc_stbd.command_speed = (int)(state->motor_limit_controller/100 * mc_stbd.command_speed);

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
    else // in all other states, motors should not move
    {
        // if the rc control source is RC_MODE_ZERO (not rc or auto) OR in auto, if we are not in RUN
		
		// zero the integral terms
        state->gains_vel.integral = 0;
        state->gains_heading.integral = 0;

        if( loopCount % 10 == 0 )
        {
            printf( "o" );
        }
        if( loopCount % 1000 == 0 )
        {
            printf( "\nRC Mode: %d Run Mode: %d\n", state->control_source, state->run_mode);
        }
		// set motor speed to zero
		mc_port.command_speed = 0;
		mc_stbd.command_speed = 0;

		// disable motor relay
        state->torqeedo_motors_relay_enabled = false;
        if (state->torqeedo_motors_relay_reported == true)
        {
            // turn off relay
			send_relay_cmd(state, false);
			fprintf(stderr, "Entering ZERO or STOP mode - disable torqeedo motors relay\n");
        }

	}

    mc_port.utime = timestamp_now();
    state->lcm.publish(state->vehicle_name+".PORT_MOTOR_CONTROL", &mc_port);
    mc_stbd.utime = timestamp_now();
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
    state.control_source = RC_MODE_ZERO;

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
    state.lcm.subscribeFunction(state.vehicle_name+".ACFR_NAV", acfr_nav_callback, &state);
    state.lcm.subscribeFunction(state.vehicle_name+".WAM_V_CONTROL", control_callback, &state);
    state.lcm.subscribeFunction(state.vehicle_name+".SPEKTRUM_CONTROL", rc_callback, &state);
    state.lcm.subscribeFunction(state.vehicle_name+".RELAY_STATUS", &handle_relay_status, &state);
    state.lcm.subscribeFunction("HEARTBEAT_10HZ", &handle_heartbeat, &state);

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
