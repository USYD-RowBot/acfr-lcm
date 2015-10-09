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
#include "perls-lcmtypes/senlcm_xbox_controller_t.h"
#include "perls-lcmtypes/perllcm_xbox_command_t.h"
#include "perls-lcmtypes/perllcm_carlab_discrete_devices_t.h"
#include "perls-lcmtypes/perllcm_carlab_signals_t.h"
#include "perls-lcmtypes/perllcm_carlab_wrench_effort_t.h"

#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"


#define AXIS_MAX SENLCM_XBOX_CONTROLLER_T_AXIS_MAX
#define AXIS_MIN SENLCM_XBOX_CONTROLLER_T_AXIS_MIN
#define AXIS_ZERO_T_MAX 5000

//----------------------------------------------------------------------------------
// STATE STRUCTURE
//----------------------------------------------------------------------------------
typedef struct _state_t state_t;
struct _state_t
{
    int done;
    int is_daemon;
    lcm_t *lcm;

    senlcm_xbox_controller_t            xbox_ctrl_state;
    senlcm_xbox_controller_t            prev_xbox_ctrl_state;

    perllcm_xbox_command_t              xbox_cmd_msg;
    perllcm_carlab_discrete_devices_t   carl_discrete_devices_msg;
    perllcm_carlab_signals_t            carl_signals_msg;
    perllcm_carlab_wrench_effort_t      carl_wrench_effort_msg;

    const char *xbox_cont_channel;
    const char *xbox_cmd_channel;
    const char *discrete_devices_channel;
    const char *signals_channel;
    const char *wrench_effort_channel;

    int right_signal;
    int left_signal;
    int64_t horn_timestamp;
    int headlights, parking, highbeams, foglights;
    int speed_mode;
    int64_t rumble_timestamp;
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
    memcpy (&state.prev_xbox_ctrl_state, &state.xbox_ctrl_state, sizeof (* msg));
    memcpy (&state.xbox_ctrl_state, msg, sizeof (* msg));
}

/*
    xbox_cmd_msg: left_rumble, right_rumble, led_code
    carl_discrete_devices_msg: engine_on, gear
    carl_signals_msg: turn_signal, horn_on, headlights, highbeams_on, foglights_on
    carl_wrench_effort_msg: throttle(L), steer_angle(L), brakes(L)
*/
void
send_carlab_controls (void)
{
    senlcm_xbox_controller_t *x = &state.xbox_ctrl_state;
    senlcm_xbox_controller_t *prev = &state.prev_xbox_ctrl_state;

    int rumble_toggle = 0;

    // check emergency mode
    if ((x->l_trig == AXIS_MAX) && (x->r_trig == AXIS_MAX) &&
            x->l_bump && x->r_bump && x->xbox_btn)
    {
        printf ("\n E-STOP!! \n");
        //change led code
        rumble_toggle = 1;
    }

    // wrench effort
    state.carl_wrench_effort_msg.utime = x->utime;
    if (x->a_btn && !prev->a_btn)
    {
        state.speed_mode = (state.speed_mode + 1) % 4;
        rumble_toggle = 1;
    }

    double max_throttle = (1 / pow (2, 3 - state.speed_mode)) * 100;
    state.carl_wrench_effort_msg.throttle = pow ((double)(x->r_trig - AXIS_MIN)/(AXIS_MAX - AXIS_MIN), 2) * max_throttle;
    state.carl_wrench_effort_msg.brakes = pow ((double)(x->l_trig - AXIS_MIN)/(AXIS_MAX - AXIS_MIN), 2) * 100;
    if (fabs (x->l_stick_x) > AXIS_ZERO_T_MAX)
        state.carl_wrench_effort_msg.steer_angle = fabs (pow ((double)x->l_stick_x/AXIS_MAX, 2)) * ((x->l_stick_x>0)?1:-1) * 100;
    else
        state.carl_wrench_effort_msg.steer_angle = 0;

    // signals
    state.carl_signals_msg.utime = x->utime;
    if (x->r_bump && !prev->r_bump)
    {
        state.right_signal = !state.right_signal;
        rumble_toggle = 1;
    }
    if (x->l_bump && !prev->l_bump)
    {
        state.left_signal = !state.left_signal;
        rumble_toggle = 1;
    }
    if (x->xbox_btn)
    {
        state.horn_timestamp = timestamp_now ();
        rumble_toggle = 1;
    }
    if ( (x->y_btn && x->dpad_r) && !(prev->y_btn && prev->dpad_r))
    {
        state.headlights = !state.headlights;
        rumble_toggle = 1;
    }
    if ( (x->y_btn && x->dpad_l) && !(prev->y_btn && prev->dpad_l))
    {
        state.parking = !state.parking;
        rumble_toggle = 1;
    }
    if ( (x->y_btn && x->dpad_u) && !(prev->y_btn && prev->dpad_u))
    {
        state.highbeams = !state.highbeams;
        rumble_toggle = 1;
    }
    if ( (x->y_btn && x->dpad_d) && !(prev->y_btn && prev->dpad_d))
    {
        state.foglights = !state.foglights;
        rumble_toggle = 1;
    }

    if (state.right_signal && state.left_signal)
        state.carl_signals_msg.turn_signal = PERLLCM_CARLAB_SIGNALS_T_TURN_SIGNAL_HAZARD;
    else if (state.right_signal)
        state.carl_signals_msg.turn_signal = PERLLCM_CARLAB_SIGNALS_T_TURN_SIGNAL_RIGHT;
    else if (state.left_signal)
        state.carl_signals_msg.turn_signal = PERLLCM_CARLAB_SIGNALS_T_TURN_SIGNAL_LEFT;
    else
        state.carl_signals_msg.turn_signal = PERLLCM_CARLAB_SIGNALS_T_TURN_SIGNAL_OFF;

    if (state.headlights)
        state.carl_signals_msg.headlights = PERLLCM_CARLAB_SIGNALS_T_HEADLIGHTS_ON;
    else if (state.parking)
        state.carl_signals_msg.headlights = PERLLCM_CARLAB_SIGNALS_T_HEADLIGHTS_PARKING;
    else
        state.carl_signals_msg.headlights = PERLLCM_CARLAB_SIGNALS_T_HEADLIGHTS_OFF;

    state.carl_signals_msg.highbeams_on = state.highbeams;
    state.carl_signals_msg.foglights_on = state.foglights;


    // discrete devices
    //state.carl_discrete_devices_msg.utime = x->utime;
    //engine - check start and select (start_btn, back_btn)
    //gear - check no throttle, brake at least 50%, x+DIR (x_btn)


    // command reply to xbox controller
    //base_rumble = throttle level
    //base_rumble += toggle rumble
    //led
    if (rumble_toggle)
        state.rumble_timestamp = timestamp_now ();

    state.xbox_cmd_msg.utime = timestamp_now ();
    state.xbox_cmd_msg.left_rumble = (int)(150 * state.carl_wrench_effort_msg.throttle/100);
    state.xbox_cmd_msg.right_rumble = (int)(150 * state.carl_wrench_effort_msg.throttle/100);
    if (timestamp_now () -state.rumble_timestamp < 2e5)
    {
        state.xbox_cmd_msg.left_rumble += 100;
        state.xbox_cmd_msg.right_rumble += 100;
    }

    perllcm_xbox_command_t_publish (state.lcm, state.xbox_cmd_channel, &state.xbox_cmd_msg);
    //perllcm_carlab_discrete_devices_t_publish (state.lcm, state.discrete_devices_channel,
    //&state.carl_discrete_devices_msg);
    perllcm_carlab_signals_t_publish (state.lcm, state.signals_channel,
                                      &state.carl_signals_msg);
    perllcm_carlab_wrench_effort_t_publish (state.lcm, state.wrench_effort_channel,
                                            &state.carl_wrench_effort_msg);
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

    getopt_add_description (gopt, "XBox Controller Driver for CARLAB");
    getopt_add_string (gopt,  'c',  "cont_chan", 	"XBOX_CONTROLER",    "From Xbox controller LCM channel");
    getopt_add_string (gopt,  'C',  "cmd_chan", 	"XBOX_COMMAND",    "To Xbox controller LCM channel");
    getopt_add_string (gopt,  'd',  "discdev_chan", "CARLAB_SET_DISCRETE_DEVICES",   "CARLAB Discrete Devices LCM channel");
    getopt_add_string (gopt,  's',  "signal_chan", "CARLAB_SET_SIGNALS",             "CARLAB Signals LCM channel");
    getopt_add_string (gopt,  'w',  "wrench_chan", "CARLAB_SET_WRENCH_EFFORT",       "CARLAB Wrench Effort LCM channel");
    getopt_add_bool (gopt,    'D',  "daemon",  	0,                   "Run as system daemon");
    getopt_add_bool (gopt,    'h',  "help",    	0,                   "Display Help");

    if (!getopt_parse (gopt, argc, argv, 1))
    {
        getopt_do_usage (gopt,"");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help"))
    {
        getopt_do_usage (gopt,"");
        exit (EXIT_SUCCESS);
    }

    // set the lcm channels
    state.xbox_cont_channel         = getopt_get_string (gopt, "cont_chan");
    state.xbox_cmd_channel          = getopt_get_string (gopt, "cmd_chan");
    state.discrete_devices_channel  = getopt_get_string (gopt, "discdev_chan");
    state.signals_channel           = getopt_get_string (gopt, "signal_chan");
    state.wrench_effort_channel     = getopt_get_string (gopt, "wrench_chan");

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
        printf ("ERROR: lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    }

    // subscribe to xbox controller
    senlcm_xbox_controller_t_subscribe (state.lcm, state.xbox_cont_channel,
                                        &xbox_controller_cb, NULL);

    while (!state.done)
    {
        lcm_handle (state.lcm);
        send_carlab_controls ();
    }

    // clean up
    lcm_destroy (state.lcm);

    printf ("\nDone.\n");
    exit (EXIT_SUCCESS);
}
