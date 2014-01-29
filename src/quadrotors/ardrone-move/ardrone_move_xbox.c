#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// external linking req'd
#include <math.h>

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_ardrone_drive_t.h"
#include "perls-lcmtypes/perllcm_ardrone_cmd_t.h"
#include "perls-lcmtypes/senlcm_xbox_controller_t.h"

#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"


#define AXIS_MAX SENLCM_XBOX_CONTROLLER_T_AXIS_MAX
#define AXIS_MIN SENLCM_XBOX_CONTROLLER_T_AXIS_MIN
#define AXIS_ZERO_T_MAX 5000

/**** LCM COMMUNICATION ******************************************************
 *
 *    LCM INPUTS: 
 *          - xbox controller
 *          - ardrone_cmd (from Commander)
 *    LCM OUTPUTS:
 *          - "move" commands for AR.Drone
 *
 *****************************************************************************/

/**** ARDRONE MOVER (XBOX) ***************************************************
 *
 *    Listens to Xbox joysticks, publishes AR.Drone "move" when control
 *    is given to Xbox controller by Commander
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
    perllcm_ardrone_cmd_t ardrone_cmd_state;

    const char *xbox_lcm_chan;
    const char *drive_pub_lcm_chan;
    const char *cmd_lcm_chan;    
};
//Init the state structure to zero
state_t state = {0};

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
// callback for ardrone_cmd
//----------------------------------------------------------------------------------
void
ardrone_cmd_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                    const perllcm_ardrone_cmd_t *msg, void *user)
{    
    memcpy (&state.ardrone_cmd_state, msg, sizeof (* msg));
}

//----------------------------------------------------------------------------------
// Send LCM move messages to drone based on state 
//----------------------------------------------------------------------------------
void
send_drone_drive (void)
{
    // copy joystick state pointer over from state
    senlcm_xbox_controller_t *x = &state.xbox_ctrl_state;
    
    // copy ardrone_cmd state pointer over from state
    perllcm_ardrone_cmd_t *c = &state.ardrone_cmd_state;

    float vx = 0;
    float vy = 0;
    float vz = 0; 
    float vr = 0;

    // determine drive messages - BY XBOX CONTROLLER
    // -------------------------------------------------------------------------
    if (c->controller == false) {

        // translational x speed
        if(fabs (x->l_stick_y) > AXIS_ZERO_T_MAX)
            vx = x->l_stick_y / ((float)AXIS_MAX);

        // translational y speed 
        if (fabs (x->l_stick_x) > AXIS_ZERO_T_MAX)
            vy = x->l_stick_x / ((float)AXIS_MAX);

        // translational z speed
        if (fabs (x->r_stick_y) > AXIS_ZERO_T_MAX) {
            vz = x->r_stick_y / ((float)AXIS_MAX);
            vz = -1*vz;
        }
        // rotational speed
        if (fabs (x->r_stick_x) > AXIS_ZERO_T_MAX)
            vr = x->r_stick_x / ((float)AXIS_MAX);
            
        
        int64_t utime = timestamp_now ();

        //send drive messages
        perllcm_ardrone_drive_t drive_state = {
            .utime = utime,
            .vx = vx,
            .vy = vy,
            .vz = vz,
            .vr = vr,
        };
        perllcm_ardrone_drive_t_publish (state.lcm, state.drive_pub_lcm_chan, &drive_state);
    }
}

//----------------------------------------------------------------------------------
// Called when program shuts down 
//----------------------------------------------------------------------------------
static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("\nmy_signal_handler()\n");
    if (state.done) {
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
    struct sigaction act = {
        .sa_sigaction = my_signal_handler,
    };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);
    
    // Read in the command line options
    getopt_t *gopt = getopt_create ();
    
    getopt_add_description (gopt, "Driver for Drone - XBOX");
    getopt_add_string (gopt,  'x',  "in_chan",        "XBOX_CONTROLER",     "Xbox controller LCM channel");
    getopt_add_string (gopt,  'd',  "drive_pub_chan", "ARDRONE_DRIVE",      "Drone driver LCM channel");
    getopt_add_string (gopt,  'c',  "cmd_chan",       "ARDRONE_CMD",        "Drone commands LCM channel");
    getopt_add_bool (gopt,    'D',  "daemon",       0,                      "Run as system daemon");
    getopt_add_bool (gopt,    'h',  "help",         0,                      "Display Help");

    if (!getopt_parse (gopt, argc, argv, 1)) {
        getopt_do_usage (gopt,"");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt,"");
        exit (EXIT_SUCCESS);;
    }
    
    // set the lcm publish channel
    state.xbox_lcm_chan = getopt_get_string (gopt, "in_chan");
    state.drive_pub_lcm_chan = getopt_get_string (gopt, "drive_pub_chan");
    state.cmd_lcm_chan = getopt_get_string (gopt, "cmd_chan");

    //start as daemon if asked
    if (getopt_get_bool (gopt, "daemon")) {
        daemon_fork ();
        state.is_daemon = 1;
    }
    else
        state.is_daemon = 0;
    
    // initialize lcm 
    state.lcm = lcm_create (NULL);
    if (!state.lcm) {
        ERROR ("lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    }
    
    // subscribe to xbox controller
    senlcm_xbox_controller_t_subscribe(state.lcm, state.xbox_lcm_chan,
                                       &xbox_controller_cb, NULL);
                                       
    // subscribe to ardrone_cmd
    perllcm_ardrone_cmd_t_subscribe(state.lcm, state.cmd_lcm_chan,
                                       &ardrone_cmd_cb, NULL);
   
    while (!state.done) {
        lcm_handle (state.lcm);
        send_drone_drive ();
    }
             
    printf ("\nDone.\n");
    exit (EXIT_SUCCESS);
}
