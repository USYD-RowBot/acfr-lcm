#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// external linking req'd
#include <math.h>

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_ardrone_drive_t.h"
#include "perls-lcmtypes/perllcm_ardrone_state_t.h"
#include "perls-lcmtypes/perllcm_ardrone_cmd_t.h"
#include "perls-lcmtypes/perllcm_ardrone_pid_t.h"
#include "perls-lcmtypes/senlcm_mocap_t.h"
#include "perls-lcmtypes/perllcm_position_t.h"
#include "perls-lcmtypes/perllcm_pose3d_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"
#include "perls-math/gsl_util_math.h"

#define WRAP_AROUND_NAV 1
#define WRAP_AROUND_MOCAP 0

/**** LCM COMMUNICATION ******************************************************
 *
 *    LCM INPUTS: 
 *          - current quadrotor pose (from MoCap or navigator)
 *          - current waypoint (from Mission Manager)
 *          - ardrone_cmd (from Commander)
 *    LCM OUTPUTS:
 *          - "move" commands for AR.Drone by PID controller
 *
 *****************************************************************************/

/**** ARDRONE MOVER (PID) ****************************************************
 *
 *    Computes PID control from current pose and waypoint, publishes AR.Drone 
 *    "move" when control is given to PID controller by Commander
 *
 *****************************************************************************/

typedef struct _config_t config_t;
struct _config_t
{
   double pGain;
   double iGain;
   double dGain;
};

//----------------------------------------------------------------------------------
// STATE STRUCTURE 
//----------------------------------------------------------------------------------
typedef struct _state_t state_t;
struct _state_t
{
    int done;
    int is_daemon;
    lcm_t *lcm;
    
    perllcm_ardrone_cmd_t ardrone_cmd_state;
    perllcm_ardrone_state_t *ardrone_state_state;
	senlcm_mocap_t mocap_state;
    perllcm_position_t quad_state;
	perllcm_pose3d_t waypoint_state;

    long utime;
    double dt;
    int wrap_around;
    
	const char *mocap_lcm_chan;
	const char *waypoint_lcm_chan;
    const char *drive_pub_lcm_chan;
    const char *cmd_lcm_chan;    
    const char *state_lcm_chan;    
    const char *nav_lcm_chan;
    const char *pid_lcm_chan;
};
//Init the state structure to zero
state_t state = {0};

//----------------------------------------------------------------------------------
// PID STRUCTURE 
//----------------------------------------------------------------------------------
typedef struct _pid_ctrl_t pid_ctrl_t;
struct _pid_ctrl_t
{
    float dState;           // differential state
    float iState; 	        // integrator state
    float pState;           // proportional state
    float iMax, iMin;       // max and min allowable integrator state
    float prev_position;    // last position input
    float accum_error;      // accumulated error for integral term

    config_t config;        // pid gains
    
};

//Init the pid structures
pid_ctrl_t x_pid = {
    .dState = 0,
    .iState = 0,
    .pState = 0,
    .iMax = 5,
    .iMin = -5,
    .prev_position = 0,
    .accum_error = 0,
    .config = {0},
};
pid_ctrl_t y_pid = {
    .dState = 0,
    .iState = 0,
    .pState = 0,
    .iMax = 5,
    .iMin = -5,
    .prev_position = 0,
    .accum_error = 0,
    .config = {0},
};
pid_ctrl_t z_pid = {
    .dState = 0,
    .iState = 0,
    .pState = 0,
    .iMax = 1,
    .iMin = -1,
    .prev_position = 0,
    .accum_error = 0,
    .config = {0},
};
pid_ctrl_t h_pid = {
    .dState = 0,
    .iState = 0,
    .pState = 0,
    .iMax = 1,
    .iMin = -1,
    .prev_position = 0,
    .accum_error = 0,
    .config = {0},
};

//----------------------------------------------------------------------
// Loads the required info from the .cfg file into the state
//----------------------------------------------------------------------
void
xy_pid_load_cfg (config_t *config)
{
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        ERROR ("Could not create configuration parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
        exit (EXIT_FAILURE);
    }
    config->pGain = bot_param_get_double_or_fail (param, "pid_control.KP_XY");
    config->iGain = bot_param_get_double_or_fail (param, "pid_control.KI_XY");
    config->dGain = bot_param_get_double_or_fail (param, "pid_control.KD_XY");  
}

void
z_pid_load_cfg (config_t *config)
{
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        ERROR ("Could not create configuration parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
        exit (EXIT_FAILURE);
    }
    config->pGain = bot_param_get_double_or_fail (param, "pid_control.KP_Z");
    config->iGain = bot_param_get_double_or_fail (param, "pid_control.KI_Z");
    config->dGain = bot_param_get_double_or_fail (param, "pid_control.KD_Z");
}

void
h_pid_load_cfg (config_t *config)
{
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        ERROR ("Could not create configuration parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
        exit (EXIT_FAILURE);
    }
    config->pGain = bot_param_get_double_or_fail (param, "pid_control.KP_R");
    config->iGain = bot_param_get_double_or_fail (param, "pid_control.KI_R");
    config->dGain = bot_param_get_double_or_fail (param, "pid_control.KD_R");
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
// callback for ardrone_state
//----------------------------------------------------------------------------------
void
ardrone_state_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                    const perllcm_ardrone_state_t *msg, void *user)
{    
    if (!state.ardrone_state_state)
        state.ardrone_state_state = (perllcm_ardrone_state_t *) malloc (sizeof(perllcm_ardrone_state_t));
    memcpy (state.ardrone_state_state, msg, sizeof (* msg));
}
//----------------------------------------------------------------------------------
// callback for MoCap
//----------------------------------------------------------------------------------
void
mocap_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                    const senlcm_mocap_t *msg, void *user)
{    
    if (msg->valid)
        memcpy (&state.mocap_state, msg, sizeof (* msg));
}

//----------------------------------------------------------------------------------
// callback for navigator
//----------------------------------------------------------------------------------
void
quad_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                    const perllcm_position_t *msg, void *user)
{    
    memcpy (&state.quad_state, msg, sizeof (* msg));
}

//----------------------------------------------------------------------------------
// Update the PID controller
//----------------------------------------------------------------------------------
float
update_PID(pid_ctrl_t * pid, double error, double position, double velocity)
{

    float pTerm, dTerm, iTerm, output;
    pTerm = pid->config.pGain * error;     // calculate the proportional term
    pid->pState = pTerm;                   // store the proportional term

    // calculate the integral state with appropriate limiting
    pid->accum_error += error * state.dt;
    if (pid->accum_error > pid->iMax)
        pid->accum_error = pid->iMax;
    else if (pid->accum_error < pid->iMin)
        pid->accum_error = pid->iMin;

    iTerm = pid->config.iGain * pid->accum_error;    // calculate the integral term
    pid->iState = iTerm;                             // store the integral term
    
    if (fabs(velocity) > 0.0)   // 0 is adjustable threshold
        dTerm = pid->config.dGain * velocity;
    else
        dTerm = 0;

    pid->dState = dTerm;    // store the differential term

    output = pTerm + iTerm - dTerm;
    
    if (output > 1.0)
        output = 1.0;
    else if (output < -1.0)
        output = -1.0;

    return output;
}

//----------------------------------------------------------------------------------
// Update the PID controller with a given velocity
//----------------------------------------------------------------------------------
float
update_PID_diff_velocity (pid_ctrl_t * pid, double error, double position)
{
    double vel = (position - pid->prev_position) / state.dt;
    pid->prev_position = position;
    return update_PID (pid, error, position, vel);
}

//----------------------------------------------------------------------------------
// Update the PID controller with a given velocity
//----------------------------------------------------------------------------------
float
update_PID_with_velocity (pid_ctrl_t * pid, double error, double velocity)
{
    return update_PID (pid, error, 0, velocity);
}

//----------------------------------------------------------------------------------
// Send LCM messages to drone controller based on state 
//----------------------------------------------------------------------------------
void
send_drone_drive (void)
{
    // copy ardrone_cmd state pointer over from state
    perllcm_ardrone_cmd_t *c = &state.ardrone_cmd_state;

    perllcm_position_t *m = &state.quad_state;

	// copy mocap position pointer over from state
    if (state.wrap_around == WRAP_AROUND_MOCAP)
        memcpy (m->xyzrph, state.mocap_state.xyzrph, 6*sizeof (double));

    // copy waypoints pointer over from state
	perllcm_pose3d_t *w = &state.waypoint_state;

    float vx, x_p, x_i, x_d = 0;
    float vy, y_p, y_i, y_d = 0;
    float vz, z_p, z_i, z_d = 0; 
    float vr, r_p, r_i, r_d = 0;
    float vel_x, vel_y;

	//determine drive messages - BY PID CONTROLLER
	//--------------------------------------------------------------------------
    if (c->controller == true) {
        
        
        float range = sqrt( pow(w->mu[0] - m->xyzrph[0],2) + pow(w->mu[1] - m->xyzrph[1],2));
        float bearing = atan2( w->mu[1] - m->xyzrph[1], w->mu[0] - m->xyzrph[0]) - m->xyzrph[5];
        float error_x = range * cos (bearing);
        float error_y = range * sin (bearing);

        if (!state.ardrone_state_state)
            vel_x = update_PID_diff_velocity (&x_pid, error_x, m->xyzrph[0]);
        else
            vel_x = update_PID_with_velocity (&x_pid, error_x, state.ardrone_state_state->vx);

        if (!state.ardrone_state_state)
            vel_y = update_PID_diff_velocity (&y_pid, error_y, m->xyzrph[1]);
        else
            vel_y = update_PID_with_velocity (&y_pid, error_y, state.ardrone_state_state->vy);

        vx = -1*vel_x;
        x_p = x_pid.pState;
        x_i = x_pid.iState;
        x_d = x_pid.dState;

        vy = vel_y;
        y_p = y_pid.pState;
        y_i = y_pid.iState;
        y_d = y_pid.dState;
        

        // z direction
        float vel_z = update_PID_diff_velocity (&z_pid, w->mu[2] - m->xyzrph[2], m->xyzrph[2]);
        vz = -1*vel_z;
        z_p = -1*z_pid.pState;
        z_i = -1*z_pid.iState;
        z_d = -1*z_pid.dState;
        
        // mocap heading
        vr = update_PID_diff_velocity (&h_pid, gslu_math_minimized_angle (w->mu[5] - m->xyzrph[5]), m->xyzrph[5]);
        r_p = h_pid.pState;
        r_i = h_pid.iState;
        r_d = h_pid.dState;
        
        int64_t utime = state.utime;

        //send drive messages
        perllcm_ardrone_drive_t drive_state = {
            .utime = utime,
            .vx = vx,
            .vy = vy,
            .vz = vz,
            .vr = vr,
        };
        perllcm_ardrone_drive_t_publish (state.lcm, state.drive_pub_lcm_chan, &drive_state);

        //send PID state
        perllcm_ardrone_pid_t pid_state = {
            .utime = utime,
            .x_pTerm = x_p,
            .x_iTerm = x_i,
            .x_dTerm = x_d,
            .x_pid = vx,
            .y_pTerm = y_p,
            .y_iTerm = y_i,
            .y_dTerm = y_d,
            .y_pid = vy,
            .z_pTerm = z_p,
            .z_iTerm = z_i,
            .z_dTerm = z_d,
            .z_pid = vz,
            .h_pTerm = r_p,
            .h_iTerm = r_i,
            .h_dTerm = r_d,
            .h_pid = vr,
        }; 
        perllcm_ardrone_pid_t_publish (state.lcm, state.pid_lcm_chan, &pid_state);
    }
}



//----------------------------------------------------------------------------------
// callback for waypoints
//----------------------------------------------------------------------------------
void
waypoint_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                    const perllcm_pose3d_t *msg, void *user)
{    
    memcpy (&state.waypoint_state, msg, sizeof (* msg));
}

void
heartbeat_cb (const lcm_recv_buf_t *rbuf, const char *channel,
                    const perllcm_heartbeat_t *msg, void *user)
{
    state.dt = (msg->utime - state.utime) / 1e6;
    state.utime = msg->utime;

    send_drone_drive ();
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
    
    // load in config file option
    xy_pid_load_cfg (&x_pid.config);
    xy_pid_load_cfg (&y_pid.config);
    z_pid_load_cfg (&z_pid.config);
    h_pid_load_cfg (&h_pid.config);
    
    // Read in the command line options
    getopt_t *gopt = getopt_create ();
    
    getopt_add_description (gopt, "Driver for Drone - PID");
	getopt_add_string (gopt,  'm',  "mocap_chan",       "MOCAP_POSE_ARDRONE",	"MoCap LCM channel");
	getopt_add_string (gopt,  'w',  "waypoint_chan",    "ARDRONE_WAYPOINT",     "Waypoints LCM channel");	
    getopt_add_string (gopt,  'd',  "drive_pub_chan",   "ARDRONE_DRIVE",        "Drone move LCM channel");
    getopt_add_string (gopt,  'c',  "cmd_chan",         "ARDRONE_CMD",          "Drone commands LCM channel");
    getopt_add_string (gopt,  's',  "state_chan",        "ARDRONE_STATE",          "Drone state feedback LCM channel");
    getopt_add_string (gopt,  'n',  "nav_chan",         "ARDRONE_NAV",          "Drone navigator LCM channel");
    getopt_add_string (gopt,  'p',  "pid_chan",         "ARDRONE_PID",          "PID LCM channel");
    getopt_add_bool (gopt,    'v',  "nav",          0,                      "Wrap around nav");
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
    state.mocap_lcm_chan = getopt_get_string (gopt, "mocap_chan");
    state.waypoint_lcm_chan = getopt_get_string (gopt, "waypoint_chan");    
    state.drive_pub_lcm_chan = getopt_get_string (gopt, "drive_pub_chan");
    state.cmd_lcm_chan = getopt_get_string (gopt, "cmd_chan");
    state.state_lcm_chan = getopt_get_string (gopt, "state_chan");
    state.nav_lcm_chan = getopt_get_string (gopt, "nav_chan");
    state.pid_lcm_chan = getopt_get_string (gopt, "pid_chan");

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

    if (getopt_get_bool (gopt, "nav"))
        state.wrap_around = WRAP_AROUND_NAV;
    
    // subscribe to ardrone_cmd
    perllcm_ardrone_cmd_t_subscribe (state.lcm, state.cmd_lcm_chan,
                                       &ardrone_cmd_cb, NULL);
    // subscribe to ardrone_state
    perllcm_ardrone_state_t_subscribe (state.lcm, state.state_lcm_chan,
                                       &ardrone_state_cb, NULL);

    if (state.wrap_around == WRAP_AROUND_MOCAP)
        // subscribe to MoCap
        senlcm_mocap_t_subscribe (state.lcm, state.mocap_lcm_chan,
                                       &mocap_cb, NULL);
    else if (state.wrap_around == WRAP_AROUND_NAV)
        // subscribe to navigator
        perllcm_position_t_subscribe (state.lcm, state.nav_lcm_chan,
                                       &quad_cb, NULL);
                                       
    // subscribe to waypoints
    perllcm_pose3d_t_subscribe (state.lcm, state.waypoint_lcm_chan,
                                       &waypoint_cb, NULL);

    // subscribe to heartbeat
    perllcm_heartbeat_t_subscribe (state.lcm, "HEARTBEAT_10HZ", &heartbeat_cb, NULL);
    
    while (!state.done) {
        lcm_handle (state.lcm);
    }
             
    if (state.ardrone_state_state)
        perllcm_ardrone_state_t_destroy (state.ardrone_state_state);
    printf ("\nDone.\n");
    exit (EXIT_SUCCESS);
}
