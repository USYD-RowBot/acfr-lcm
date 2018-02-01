
#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <libgen.h>
#include <pthread.h>
#include <signal.h>
#include <lcm/lcm.h>
#include "timer.h"
#include <bot_param/param_client.h>
#include "perls-common/timestamp.h"
#include "perls-common/lcm_util.h"
#include "pid.h"
#include "perls-lcmtypes/acfrlcm_auv_acfr_nav_t.h"
#include "perls-lcmtypes/acfrlcm_auv_sirius_motor_command_t.h"
#include "perls-lcmtypes/acfrlcm_auv_goal_setpoint_t.h"
#include "perls-lcmtypes/acfrlcm_auv_control_response_t.h"
#include "perls-lcmtypes/senlcm_raw_ascii_t.h"
#include "perls-lcmtypes/acfrlcm_auv_relay_t.h"

// set the delta T to 0.1s, 10Hz loop rate
#define CONTROL_DT 0.1

#define PI 3.141592654
#define TWOPI (2*PI)
#define RTOD   (57.295779513082320876798154814105)
#define DTOR   (1/RTOD)
#define TO_RPM  112
#define GOAL_PT_RADIUS    1.5
#define DEPTH_BAND        0.2
#define HEADING_BAND      DTOR*1.0     //one degree

#define NO_VALUE ((long)(-98765))

#define debug_printf(debug, fmt, ...) \
            do { if (debug) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

// Sirius pid gains structure
typedef struct 
{
    pid_gains_t heading;
    pid_gains_t depth;
    pid_gains_t sway;
    pid_gains_t surge;
} sirius_pid_gains_t;

typedef struct 
{
    // pid gains
    sirius_pid_gains_t gains;
    
    // prop stuff
    double prop_factor;
    double max_prop_rpm;
    double min_prop_rpm;
} controller_config_t;

typedef struct
{
    double port;
    double stbd;
    double vert;
    double lat;
} thruster_cmd_t;

typedef struct
{
    double reference;
    double velocity;
    double acc;
    double rate_reference;
    double goal;
} traj_fields_t;

typedef struct
{
    traj_fields_t heading;
    traj_fields_t surge;
    traj_fields_t sway;
    traj_fields_t depth;
    traj_fields_t altitude;
} trajectory_state_t;

typedef enum {ON, OFF} on_off_t;
typedef enum {DEPTH_ON, DEPTH_OFF, DEPTH_ALT} depth_mode_t;
typedef enum {TRANSIT_ON, TRANSIT_OFF, TRANSIT_CLOOP} transit_mode_t;
typedef enum {NONE, ALTITUDE} track_by_sensor_t;
typedef enum {DOP_RESET, DOP_RUNNING} doppler_reset_t;
typedef enum {TRAJ_OFF, MAN_HEADING, TRACKLINE_1P, TRACKLINE_2P, HOMING_HEADING,  
              AUTO_DEPTH, CLOOP, OLOOP, TRACKLINE_2P_H, CLOOP_HOLD, CLOOP_CRAB} modes_t;

// return types from the MP routine
enum
{
    MP_TIMEOUT,
    MP_PARSE_ERROR,
    MP_SET,
    MP_SEND,
    MP_GPT,
    MP_TBS,
    MP_CTL,
    MP_THR_REF 
};

typedef struct
{
    on_off_t heading;
    depth_mode_t depth;
    transit_mode_t transit;
} servo_mode_t;

typedef struct 
{ modes_t mode;
  double value;
} depth_traj_t;

typedef struct 
{ modes_t heading;
  depth_traj_t depth;
  modes_t transit;
} trajectory_mode_t;

// Structure to define a trackline goal point
// Data written by MISSION_PLANNER and accessed by CONTROL_THREAD
typedef struct
{ int    goal_id;
  char   goal_str[25];
  long long timestamp;
  double xpos1;           // meters
  double ypos1;           // meters
  double zpos1;           // meters (depth)
  double xpos2;           // meters
  double ypos2;           // meters
  double zpos2;           // meters (depth)
  double heading;         // degrees
  double xy_vel;          // m/s
  double z_vel;           // m/s
  double cmd_timeout;        // in seconds

  // It appears that the mission planner currently doesn't send these - Ian
  double mission_timeout;  
  double max_depth;       // meters
  double min_alt;         // meters

} goal_setpoint_t;

typedef struct
{ double heading_rate_limit;
  double depth_rate_limit; 
  double surge_rate_limit; 
  double sway_rate_limit; 
  double heading_tau;
  double depth_tau;
  double surge_tau;
  double sway_tau;
  double two_point_max_step;
  double two_point_gain;
  double tl_band;
  double tl_gain;
  double fwd_distance_min;
  double fwd_distance_slowdown;
} trajectory_config_t;

typedef struct 
{ double heading_int;
  double depth_int;
  double altitude_int;
  double surge_int;
  double sway_int;
} controller_state_t;

typedef struct
{ 
  track_by_sensor_t track_by_sensor;
  double value;
} sensor_tracking_t;

typedef struct
{ modes_t transit_mode;
  doppler_reset_t doppler_reset;
  sensor_tracking_t tracking;
//  estimator_mode_t estimator_mode; *** Not used
  double reset_x;
  double reset_y;
} control_msg_t;

typedef struct
{
    int debug_mp;
    int debug_control;
    int debug_trajectory;
    int debug_main;
} debug_t;

typedef struct 
{
    int at_goal;
    double dis_to_goal;
    double time_to_goal;
    //  abort_types_t safety;
    //  data_source_t sensor_fault; 
    //  abort_types_t abort_msg;
    int goal_completed;
    int modem;
} ctl_status_t;

// a structure to contain all the structures and variables that need to be passed
// to the LCM callbacks and other routines
typedef struct
{
    lcm_t *lcm;
    acfrlcm_auv_acfr_nav_t nav;
    pthread_mutex_t *nav_lock;
    
    goal_setpoint_t goal_setpoint;
    control_msg_t control_msg;
    ctl_status_t control_status;
    int new_mp;
    pthread_mutex_t *mp_lock;
    
    long long start_time;
    int nav_alive;
    debug_t debug;
    
} state_t;


extern int controller(controller_config_t *cc, acfrlcm_auv_acfr_nav_t *es, trajectory_state_t *ts, servo_mode_t *mode, thruster_cmd_t *tc, debug_t *debug);
extern void trajectory(goal_setpoint_t *gs, trajectory_state_t *ts, 
		acfrlcm_auv_acfr_nav_t *es, trajectory_config_t *tc , trajectory_mode_t *tm, controller_state_t *cs, debug_t *debug);
#endif
