#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// external linking req'd
#include <math.h>

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_ardrone_cmd_t.h"
#include "perls-lcmtypes/senlcm_xbox_controller_t.h"

#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"


#define AXIS_MAX SENLCM_XBOX_CONTROLLER_T_AXIS_MAX
#define AXIS_MIN SENLCM_XBOX_CONTROLLER_T_AXIS_MIN
#define AXIS_ZERO_T_MAX 5000
#define BUTTON_TOLERANCE 300000

/**** LCM COMMUNICATION ******************************************************
 *
 *    LCM INPUTS:
 *          - xbox controller
 *    LCM OUTPUTS:
 *          - ardrone_cmd
 *
 *****************************************************************************/

/**** ARDRONE COMMANDER (XBOX) ***********************************************
 *
 *    Listens to XBox controller and publishes "cmd" for AR.Drone
 *    Determines whether "move" for AR.Drone is Xbox or PID
 *
 *****************************************************************************/

//----------------------------------------------------------------------------------
// STATE STRUCTURE
//----------------------------------------------------------------------------------
typedef struct _state_t state_t;
struct _state_t
{
    int done;
    int is_daemon;
    lcm_t *lcm;

    senlcm_xbox_controller_t xbox_ctrl_state;

    const char *xbox_lcm_chan;
    const char *cmd_pub_lcm_chan;
};
//Init the state structure to zero
state_t state = {0};

//Set button delays to 0
int64_t hover_delay, takeoff_delay, camera_delay, emergency_delay, change_speed_delay, follow_toggle_delay = 0, land_toggle_delay = 0;

//Set controller toggle to Xbox initially
bool follow_toggle = false;
bool land_toggle = false;

//----------------------------------------------------------------------------------
// callback for xbox controller
//----------------------------------------------------------------------------------
void
xbox_controller_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                    const senlcm_xbox_controller_t *msg, void *user)
{
    memcpy (&state.xbox_ctrl_state, msg, sizeof (* msg));
}

//----------------------------------------------------------------------------------
// Send LCM command messages to drone based on state
//----------------------------------------------------------------------------------
void
send_drone_cmd (void)
{
    // copy joystick state pointer over from state
    senlcm_xbox_controller_t *x = &state.xbox_ctrl_state;

    bool takeoff = false;
    bool camera = false;
    bool emergency = false;
    bool hover = false;

    int change_speed = 0;

    //set emergency kill (same as reset signal)
    if (x->l_trig == AXIS_MAX && x->r_trig == AXIS_MAX && x->l_bump && x->r_bump && timestamp_now()-emergency_delay > BUTTON_TOLERANCE)
    {
        emergency = true;
        emergency_delay = timestamp_now ();
    }

    // determine command: takeoff, land hover, reset, camera switch, ctrl_toggle or speed controls

    else if (x->start_btn && timestamp_now()-takeoff_delay > BUTTON_TOLERANCE)
    {
        takeoff = true;
        takeoff_delay = timestamp_now ();
    }

    else if (x->a_btn && timestamp_now()-hover_delay > BUTTON_TOLERANCE)
    {
        hover = true;
        hover_delay = timestamp_now ();
    }

    else if (x->b_btn && timestamp_now()-camera_delay > BUTTON_TOLERANCE)
    {
        camera = true;
        camera_delay = timestamp_now ();
    }

    else if (x->dpad_u && timestamp_now()-change_speed_delay > BUTTON_TOLERANCE)
    {
        change_speed = 1;
        change_speed_delay = timestamp_now ();
    }

    else if (x->dpad_d && timestamp_now()-change_speed_delay > BUTTON_TOLERANCE)
    {
        change_speed = -1;
        change_speed_delay = timestamp_now ();
    }

    else if (x->x_btn && timestamp_now()-follow_toggle_delay > BUTTON_TOLERANCE)
    {
        follow_toggle = !follow_toggle;
        follow_toggle_delay = timestamp_now ();
    }

    else if (x->y_btn && timestamp_now()-land_toggle_delay > BUTTON_TOLERANCE)
    {
        land_toggle = !land_toggle;
        land_toggle_delay = timestamp_now ();
    }

    //determine drive messages - BY PID CONTROLLER
    //--------------------------------------------------------------------------
    //if (ctrl_toggle == true) {
    //printf ("PID CONTROL\n");
    // set perllcm_ardrone_cmd_t.controller to TRUE
    //}

    // determine drive messages - BY XBOX CONTROLLER
    // -------------------------------------------------------------------------
    if ((fabs (x->l_stick_x) > AXIS_ZERO_T_MAX) ||
            (fabs (x->l_stick_y) > AXIS_ZERO_T_MAX) ||
            (fabs( x->r_stick_x) > AXIS_ZERO_T_MAX) ||
            (fabs (x->r_stick_y) > AXIS_ZERO_T_MAX))
    {

        // set ctrl_toggle to OFF
        follow_toggle = false;
        land_toggle = false;
        //printf ("XBOX CONTROL\n");
        // set perllcm_ardrone_cmd_t.controller to FALSE
    }

    int64_t utime = timestamp_now ();

    //send command messages
    perllcm_ardrone_cmd_t cmd_state =
    {
        .utime = utime,
        .takeoff = takeoff,
        .hover = hover,
        .camera = camera,
        .emergency = emergency,
        .controller = follow_toggle || land_toggle,
        .auth_follow = follow_toggle,
        .auth_land = land_toggle,
        .change_speed = change_speed,
    };
    perllcm_ardrone_cmd_t_publish (state.lcm, state.cmd_pub_lcm_chan, &cmd_state);
}

//----------------------------------------------------------------------------------
// Called when program shuts down
//----------------------------------------------------------------------------------
static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("\nmy_signal_handler()\n");
    if (state.done)
    {
        printf ("Goodbye\n");
        exit (EXIT_FAILURE);
    }
    else
        state.done = 1;
}

//----------------------------------------------------------------------------------
// Main
//----------------------------------------------------------------------------------
int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // install custom signal handler
    struct sigaction act =
    {
        .sa_sigaction = my_signal_handler,
    };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);

    // Read in the command line options
    getopt_t *gopt = getopt_create ();

    getopt_add_description (gopt, "Commander for Drone - XBOX");
    getopt_add_string (gopt,  'x',  "in_chan",      "XBOX_CONTROLER",       "Xbox controller LCM channel");
    getopt_add_string (gopt,  'c',  "cmd_pub_chan", "ARDRONE_CMD",          "Drone commands LCM channel");
    getopt_add_bool (gopt,    'D',  "daemon",       0,                      "Run as system daemon");
    getopt_add_bool (gopt,    'h',  "help",         0,                      "Display Help");

    if (!getopt_parse (gopt, argc, argv, 1))
    {
        getopt_do_usage (gopt,"");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help"))
    {
        getopt_do_usage (gopt,"");
        exit (EXIT_SUCCESS);;
    }

    // set the lcm publish channel
    state.xbox_lcm_chan = getopt_get_string (gopt, "in_chan");
    state.cmd_pub_lcm_chan = getopt_get_string (gopt, "cmd_pub_chan");

    //start as daemon if asked
    if (getopt_get_bool (gopt, "daemon"))
    {
        daemon_fork ();
        state.is_daemon = 1;
    }
    else
        state.is_daemon = 0;

    // initialize lcm
    state.lcm = lcm_create (NULL);
    if (!state.lcm)
    {
        ERROR ("lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    }

    // subscribe to xbox controller
    senlcm_xbox_controller_t_subscribe(state.lcm, state.xbox_lcm_chan,
                                       &xbox_controller_cb, NULL);

    while (!state.done)
    {
        lcm_handle (state.lcm);
        send_drone_cmd ();
    }

    printf ("\nDone.\n");
    exit (EXIT_SUCCESS);
}
