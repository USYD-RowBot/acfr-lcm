#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>

#include <math.h>

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_segway_drive_t.h"
#include "perls-lcmtypes/senlcm_xbox_controller_t.h"

#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"


#define AXIS_MAX SENLCM_XBOX_CONTROLLER_T_AXIS_MAX
#define AXIS_MIN SENLCM_XBOX_CONTROLLER_T_AXIS_MIN
#define AXIS_ZERO_T_MAX 5000

//speed control defines
#define BASE_TV  0.5	        //in meters per second
#define MAX_TV   4*BASE_TV	//in meters per second
#define BASE_RV  0.6 	        //in radians per second
#define MAX_RV   3*BASE_RV	//in radians per second

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
    const char *pub_lcm_chan;    
};

//Init the state structure to zero
state_t state = {0};

//----------------------------------------------------------------------------------
// callback for xbox controller
//----------------------------------------------------------------------------------
void
xbox_controller_cb (const lcm_recv_buf_t *rbuf, const char *channel,
		    const senlcm_xbox_controller_t *msg, void *user) {
    
    memcpy (&state.xbox_ctrl_state, msg, sizeof (* msg));
}

//----------------------------------------------------------------------------------
// Send LCM messages to segway controller based on state 
//----------------------------------------------------------------------------------
void
send_segway_ctrl (void)
{
    // copy joystick state pointer over from state
    senlcm_xbox_controller_t *x = &state.xbox_ctrl_state;


    // send kill message (l_trig + r_trig + l_bump + r_bump + xbox_btn)
    // -------------------------------------------------------------------------
    if ((x->l_trig == AXIS_MAX) && (x->r_trig == AXIS_MAX)
        && x->l_bump && x->r_bump && x->xbox_btn) {
        printf ("\n E-STOP \n");
        // add in easy daq lcm message here to switch kill switch
    }
    
    // send move message
    // -------------------------------------------------------------------------
    if ((fabs( x->l_stick_x) > AXIS_ZERO_T_MAX) || (fabs (x->l_stick_y) > AXIS_ZERO_T_MAX)) {
        float vt = 0;
        float vr = 0; 
        
        // rotational speed
        if(fabs (x->l_stick_x) > AXIS_ZERO_T_MAX) {
            vr = x->l_stick_x/((float)AXIS_MAX)*BASE_RV;
	    vr = -1*vr; //revers direction
        }
        // traslationial speed 
        if (fabs (x->l_stick_y) > AXIS_ZERO_T_MAX) {
            vt = x->l_stick_y/((float)AXIS_MAX)*BASE_TV;
            //forward on stick is negative so flip it
            vt = -1*vt;
        }
    
        // speed boost! (a to go 2X, a+b to go 3X)
	// Only boost vt not vr otherwise to sensitive to turning input
        if (x->a_btn && x->b_btn) {
            vt = vt*4;
        }
        else if(x->a_btn) {
            vt = vt*2; 
        }
    
        perllcm_segway_drive_t drive_state = {
	    timestamp_now(),
	    vt,
	    vr};
        perllcm_segway_drive_t_publish (state.lcm, state.pub_lcm_chan, &drive_state);
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
    
    getopt_add_description (gopt, "XBox Controller Driver for Segway");
    getopt_add_string (gopt,  'c',  "in_chan", 	"XBOX_CONTROLER",    "Xbox controller LCM channel");
    getopt_add_string (gopt,  'p',  "pub_chan", "SEGWAY_DRIVE",      "Segway driver LCM channel");
    getopt_add_bool (gopt,    'D',  "daemon",  	0,                   "Run as system daemon");
    getopt_add_bool (gopt,    'h',  "help",    	0,                   "Display Help");

    if (!getopt_parse (gopt, argc, argv, 1)) {
        getopt_do_usage (gopt,"");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt,"");
        exit (EXIT_SUCCESS);
    }
    
    // set the lcm publish channel
    state.xbox_lcm_chan = getopt_get_string (gopt, "in_chan");
    state.pub_lcm_chan = getopt_get_string (gopt, "pub_chan");
    
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
        printf ("ERROR: lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    }
    
    // subscribe to xbox controller
    senlcm_xbox_controller_t_subscribe (state.lcm, state.xbox_lcm_chan,
                                        &xbox_controller_cb, NULL);
    
    while (!state.done) {
        
	lcm_handle (state.lcm);
        send_segway_ctrl ();
    }
             
    printf ("\nDone.\n");
    exit (EXIT_SUCCESS);
}
