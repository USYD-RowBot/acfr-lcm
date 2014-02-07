#include <iostream>
#include <ftdiexceptions.h>
#include <ftdiserver.h>
#include <ftdimodule.h>
#include <segway_rmp200.h>

#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
#include "perls-common/units.h"

#include "perls-lcmtypes/perllcm_segway_drive_t.h"
#include "perls-lcmtypes/perllcm_segway_state_t.h"

// convert radians per second to rotations per second
#define RADSPS_TO_ROTSPS 0.159155

#define MAXSPEED  2.0	// meters per second
#define MAXLR     1.8 	// radians per second

typedef struct _state_t state_t;
struct _state_t {
    int is_daemon;
    
    float vt; // meters per second
    float vr; // radians per second
};

// function to move the segway
// use this instead of calling the segway classes move method so that we
// make sure we get the units correct
void
segway_move (CSegwayRMP200 *segway, state_t *state)
{    
    // state->vt in meters per second which is what segway class expects
    // state->vr in rads per second while segway class expects revs per sec   
    segway->move (state->vt, RADSPS_TO_ROTSPS*state->vr);     
}

static void
cmd_callback (const lcm_recv_buf_t *rbuf, const char *channel,
              const perllcm_segway_drive_t *msg, void *user)
{    
    state_t *state = (state_t *) user;
	
    if (abs(msg->utime - timestamp_now()) > 1e6) {
        ERROR ("Drive cmd older than 1 second, ignored");
	return;
    }

    state->vt = msg->vt; 
    state->vr = msg->vr; 
	
    if (state->vt > MAXSPEED)
        state->vt = MAXSPEED;
    else if (state->vt < -1*MAXSPEED)
        state->vt = -1*MAXSPEED;
    
    if (state->vr > MAXLR)
        state->vr = MAXLR;
    else if (state->vr < -1*MAXLR)
        state->vr = -1*MAXLR;
}

int
main (int argc, char *argv[])
{    
    state_t *state = (state_t*) calloc (1, sizeof (*state));
    
    // Read in the command line options
    getopt_t *gopt = getopt_create ();
    
    getopt_add_description (gopt, "Segway Driver");
    getopt_add_bool (gopt,    'D',  "daemon",  0,  "Run as system daemon");
    getopt_add_bool (gopt,    'h',  "help",    0,  "Display Help");

    if (!getopt_parse (gopt, argc, argv, 1)) {
        getopt_do_usage (gopt,"");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt,"");
        exit (EXIT_SUCCESS);
    }

    //start as daemon if asked
    if (getopt_get_bool (gopt, "daemon")) {
        daemon_fork ();
        state->is_daemon = 1;
    } 
    else
        state->is_daemon = 0;

    std::string segway_name = "segway";
    CSegwayRMP200 *segway;
    CFTDIServer *ftdi_server = CFTDIServer::instance ();
    std::string serial_number;
 
    // set up lcm
    lcm_t *lcm = lcm_create (NULL);
    if (!lcm) {
        printf ("lcm_create() failed!");
        exit (EXIT_FAILURE);
    }
    perllcm_segway_drive_t_subscribe (lcm, "SEGWAY_DRIVE", &cmd_callback, state);

    // set timeout for lcmu_handle_timeout
    try {
        ftdi_server->add_custom_PID (0xE729);
        std::cout << (*ftdi_server) << std::endl;

        if (ftdi_server->get_num_devices () > 0) {
            // connect to segway
            serial_number = ftdi_server->get_serial_number (0);
            segway = new CSegwayRMP200 (segway_name);
            segway->connect (serial_number);
	    
            //  get segway ready to move
            segway->unlock_balance ();
            segway->set_operation_mode (balance);
            segway->set_gain_schedule (light);

            // reset all integrators
            segway->reset_right_wheel_integrator ();
            usleep (10000);
            segway->reset_left_wheel_integrator ();
            usleep (10000);
            segway->reset_yaw_integrator ();
            usleep (10000);
            segway->reset_forward_integrator ();
            usleep (10000);

            // start listening for move commands
	    while (1) {
                //pack lcm state struct
                perllcm_segway_state_t my_state = {
		    timestamp_now(),
                    segway->get_pitch_angle () * UNITS_DEGREE_TO_RADIAN,		
                    segway->get_pitch_rate () * UNITS_DEGREE_TO_RADIAN,
                    segway->get_roll_angle () * UNITS_DEGREE_TO_RADIAN,
                    segway->get_roll_rate () * UNITS_DEGREE_TO_RADIAN,
                    segway->get_left_wheel_velocity (), // m/s
                    segway->get_right_wheel_velocity (), // m/s
                    segway->get_yaw_rate () * UNITS_DEGREE_TO_RADIAN,
                    segway->get_servo_frames (),
                    segway->get_left_wheel_displacement (),
                    segway->get_right_wheel_displacement (),
                    segway->get_forward_displacement (),
                    segway->get_yaw_displacement (), // IRI DRIVER DOES NOT MATCH DOCUMETATION FOR THIS ELEMEN LOOK INTO BEFORE USING
                    segway->get_left_motor_torque (),
                    segway->get_right_motor_torque (),
                    segway->get_ui_battery_voltage (),
                    segway->get_powerbase_battery_voltage (),
                };
                perllcm_segway_state_t_publish (lcm, "SEGWAY_STATE", &my_state);

                state->vt = 0;
                state->vr = 0;

                // timeout for lcm handel call (100 ms)
                struct timeval tv = {0};
                tv.tv_usec = 100000;
                lcmu_handle_timeout (lcm, &tv);
                segway_move (segway, state);
	    } 
            
            segway->stop ();
            segway->close ();
        }
    }
    catch (CException &e) {
        std::cout << e.what() << std::endl;
    }

    // clean up
    getopt_destroy (gopt);
    lcm_destroy (lcm);
    exit (EXIT_SUCCESS);
}
