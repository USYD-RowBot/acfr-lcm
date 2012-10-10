#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "math.h"

#include <lcm/lcm.h>
#include "perls-lcmtypes/perllcm_pose3d_t.h"
#include "perls-lcmtypes/perllcm_position_t.h"
#include "perls-lcmtypes/perllcm_ardrone_cmd_t.h"
#include "perls-lcmtypes/senlcm_mocap_t.h"
#include "perls-lcmtypes/perllcm_ardrone_cmd_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"

#include <bot_lcmgl_client/lcmgl.h>

/**** LCM COMMUNICATION ******************************************************
 *
 *    LCM INPUTS: 
 *          - current quadrotor pose
 *          - current target pose
 *    LCM OUTPUTS:
 *          - waypoint pose (incremental waypoints to goal, following lookahead)
 *          - goal pose (target waypoint for each mission state)
 *
 *****************************************************************************/

/**** MISSION STATE MACHINE **************************************************
 *
 *    0. Begin Mission
 *
 *    1. Identify target -> exploration mode
 *          PROCEED TO 2: ONCE TARGET IS FOUND
 *
 *    2. Lock onto target -> navigate towards 1 meter radius
 *          GOAL: 1 meter away from target
 *          WAYPOINT: always face target, directly to GOAL, 1m above target
 *          PROCEED TO 3: WITHIN 1 METER RADIUS
 *          GO BACK TO 1: HAVEN'T SEEN TARGET IN 10 SECONDS
 *
 *    3. Prepare for landing -> move to "look behind"
 *          GOAL: 1 meter behind/1 meter above target
 *          WAYPOINT: always face target, follow 1 meter arc to GOAL
 *          PROCEED TO 4: WITHIN .25 METERS OF "LOOK BEHIND"
 *          GO BACK TO 2: FALL OUTSIDE 2 METER RADIUS
 *          GO BACK TO 1: HAVEN'T SEEN TARGET IN 10 SECONDS
 *
 *    4. Approach landing -> safely decline to just above landing pad
 *          GOAL: center of target, centimeters above
 *          WAYPOINT: direct trajectory from "look behind"
 *          PROCEED TO 5: WITHIN .1 METERS OF TARGET, .1 METERS ABOVE TARGET
 *          GO BACK TO 3: NEVER
 *          GO BACK TO 2: FALL OUTSIDE 2 METER RADIUS
 *          GO BACK TO 1: HAVEN'T SEEN TARGET IN 10 SECONDS
 *
 *    5. Execute landing -> cut motors for landing
 *
 *    6. End Mission
 *
 *****************************************************************************/

typedef enum
{
    INITIALIZATION = 0,
    IDENTIFY_TARGET,
    LOCK_ON_TARGET,
    PREPARE_LANDING,
    BEGIN_DESCENT,
    FINAL_DESCENT,
    EXECUTE_LANDING,
    MISSION_COMPLETE
} mission_state_t;

typedef struct _state_t state_t;
struct _state_t
{
    int done;
    int is_daemon;
    lcm_t *lcm;
    int64_t utime;
    int64_t time_at_look_behind;
    int64_t time_on_target;

    int64_t last_publish;

    int use_nav;

    perllcm_position_t *quadrotor_pose;
    perllcm_position_t *target_pose;
    perllcm_ardrone_cmd_t *controller;

    mission_state_t mission_state;
};

typedef struct _config_t config_t;
struct _config_t
{
    int target_timeout; // in seconds
    perllcm_pose3d_t rel_look_behind;
    double main_radius;
    double lookahead_radius;

    char quadrotor_pose_channel[256];
    char target_pose_channel[256];
    char waypoint_channel[256];
    char goal_channel[256];
    char ardrone_cmd_channel[256];
};

// Init state and config structures
state_t state = {0};
config_t config;

//----------------------------------------------------------------------
// Loads the required info from the .cfg file into the state
//----------------------------------------------------------------------
void
manager_load_cfg (config_t *config)
{
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        ERROR ("Could not create configuration parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
        exit (EXIT_FAILURE);
    }

    strcpy (config->quadrotor_pose_channel, bot_param_get_str_or_fail(param, "navigator.quad_pose_channel"));
    strcpy (config->target_pose_channel, bot_param_get_str_or_fail(param, "navigator.targ_pose_channel"));
    strcpy (config->waypoint_channel, bot_param_get_str_or_fail(param, "manager.waypoint_channel"));
    strcpy (config->goal_channel, bot_param_get_str_or_fail(param, "manager.goal_channel"));
    strcpy (config->ardrone_cmd_channel, bot_param_get_str_or_fail(param, "manager.cmd_channel"));

    double vals[3];
    bot_param_get_double_array (param, "manager.look_behind", vals, 3);
    config->rel_look_behind.mu[0] = vals[0];
    config->rel_look_behind.mu[1] = vals[1];
    config->rel_look_behind.mu[2] = vals[2];
    config->lookahead_radius = bot_param_get_double_or_fail (param, "manager.lookahead_radius");
}

//----------------------------------------------------------------------
// Drone command callback
//----------------------------------------------------------------------
static void
quad_cmd_cb (const lcm_recv_buf_t *rbug, const char *channel,
                                const  perllcm_ardrone_cmd_t *msg, void *user)
{
    if (!state.controller)
        state.controller = (perllcm_ardrone_cmd_t*) malloc (sizeof (perllcm_ardrone_cmd_t));
    memcpy (state.controller, msg, sizeof (perllcm_ardrone_cmd_t));
}

//----------------------------------------------------------------------------------
// quadrotor pose callback
//----------------------------------------------------------------------------------
static void
quad_perllcm_position_cb (const lcm_recv_buf_t *rbug, const char *channel,
                                const  perllcm_position_t *msg, void *user)
{
    if (!state.quadrotor_pose)
        state.quadrotor_pose = (perllcm_position_t*) malloc (sizeof (perllcm_position_t));

    printf ("CALLBACK: QUADROTOR POSE\n");

    memcpy (state.quadrotor_pose, msg, sizeof (perllcm_position_t));

    state.utime = state.quadrotor_pose->utime;
}

static void
quad_senlcm_mocap_cb (const lcm_recv_buf_t *rbug, const char *channel,
                                const  senlcm_mocap_t *msg, void *user)
{
    if (!msg->valid || msg->residual > 10) return;

    if (!state.quadrotor_pose)
        state.quadrotor_pose = (perllcm_position_t*) malloc (sizeof (perllcm_position_t));

    printf ("CALLBACK: QUADROTOR POSE\n");

    memcpy (state.quadrotor_pose->xyzrph, msg->xyzrph, 6*sizeof (double));

    state.utime = msg->utime;
    state.quadrotor_pose->utime = msg->utime;
}

//----------------------------------------------------------------------------------
// target pose callback
//----------------------------------------------------------------------------------
static void
targ_perllcm_position_cb (const lcm_recv_buf_t *rbug, const char *channel,
                                const  perllcm_position_t *msg, void *user)
{
    if (!state.target_pose)
        state.target_pose = (perllcm_position_t*) malloc (sizeof (perllcm_position_t));
    
    printf ("CALLBACK: TARGET POSE\n");

    memcpy (state.target_pose, msg, sizeof (perllcm_position_t));

    state.utime = state.target_pose->utime;
}

static void
targ_senlcm_mocap_cb (const lcm_recv_buf_t *rbug, const char *channel,
                                const  senlcm_mocap_t *msg, void *user)
{
    if (!msg->valid || msg->residual > 10) return;

    if (!state.target_pose)
        state.target_pose = (perllcm_position_t*) malloc (sizeof (perllcm_position_t));

    printf ("CALLBACK: TARGET POSE\n");

    memcpy (state.target_pose->xyzrph, msg->xyzrph, 6*sizeof (double));

    state.utime = msg->utime;
    state.target_pose->utime = msg->utime;
}

double
dist2d (perllcm_position_t *a, perllcm_position_t *b)
{
    return sqrt (pow (a->xyzrph[0]-b->xyzrph[0],2) + pow (a->xyzrph[1]-b->xyzrph[1],2));
}

double
dist3d (perllcm_position_t *a, perllcm_position_t *b)
{
    return sqrt (pow (a->xyzrph[0]-b->xyzrph[0],2) + pow (a->xyzrph[1]-b->xyzrph[1],2) + pow (a->xyzrph[2]-b->xyzrph[2],2));
}

double
normalize_angle (double angle)
{
    while (angle < -M_PI)
        angle += 2*M_PI;
    while (angle > M_PI)
        angle -= 2*M_PI;
    return angle;
}

//----------------------------------------------------------------------------------
// updates mission state and publishes goals/waypoints
//----------------------------------------------------------------------------------
void
update_mission ()
{
    int publish_waypoints = 0;
    perllcm_pose3d_t waypoint = {
        .utime = state.utime,
    };
    perllcm_pose3d_t goal = {
        .utime = state.utime,
    };
    perllcm_ardrone_cmd_t land = {
        .utime = state.utime,
        .controller = state.controller && state.controller->controller,
        .auth_follow = state.controller && state.controller->auth_follow,
        .auth_land = state.controller && state.controller->auth_land,
        //.takeoff = true,
        .emergency = true,
    };
/*     perllcm_ardrone_cmd_t hover = { */
/*         .utime = state.utime, */
/*         .hover = true, */
/*     }; */

    double cur_ang, tmp, ang1, ang2, tmpx, tmpy;

    // control flow
    switch (state.mission_state) {

        // wait for initial conditions (quadrotor state update)
        case INITIALIZATION:
            printf ("STATE: INITIALIZATION\n");
            if (state.quadrotor_pose)
                state.mission_state = IDENTIFY_TARGET; // intentionally doesn't break
            else
                break;
            
        // wait until target is found
        case IDENTIFY_TARGET:
            printf ("STATE: IDENTIFY_TARGET\n");
            // do something in the future to "explore"
            goal.mu[0] = waypoint.mu[0] = state.quadrotor_pose->xyzrph[0];
            goal.mu[1] = waypoint.mu[1] = state.quadrotor_pose->xyzrph[1];
            goal.mu[2] = waypoint.mu[2] = config.rel_look_behind.mu[2];
            goal.mu[3] = waypoint.mu[3] = state.quadrotor_pose->xyzrph[3];
            goal.mu[4] = waypoint.mu[4] = state.quadrotor_pose->xyzrph[4];
            goal.mu[5] = waypoint.mu[5] = state.quadrotor_pose->xyzrph[5] + M_PI/12;

            publish_waypoints = 1;

            if (state.target_pose && 
                    (state.utime - state.target_pose->utime) < config.target_timeout * 1e6)
                state.mission_state = LOCK_ON_TARGET; /* intentionally doesn't break */
            else
                break;

        // navigate toward target, move on at 1 meter radius
        case LOCK_ON_TARGET:
            printf ("STATE: LOCK_ON_TARGET\n");
            state.time_at_look_behind = 0;
            goal.mu[5] = atan2 (state.target_pose->xyzrph[1] - state.quadrotor_pose->xyzrph[1],
                                state.target_pose->xyzrph[0] - state.quadrotor_pose->xyzrph[0]);
            goal.mu[0] = state.target_pose->xyzrph[0] + config.main_radius * cos (goal.mu[5] + M_PI);
            goal.mu[1] = state.target_pose->xyzrph[1] + config.main_radius * sin (goal.mu[5] + M_PI);
            goal.mu[2] = state.target_pose->xyzrph[2] + config.rel_look_behind.mu[2];

            waypoint.mu[5] = goal.mu[5];
            waypoint.mu[2] = goal.mu[2];
            if (sqrt (pow (goal.mu[0]-state.quadrotor_pose->xyzrph[0],2) + pow (goal.mu[1]-state.quadrotor_pose->xyzrph[1],2)) < config.lookahead_radius) {
                waypoint.mu[0] = goal.mu[0];
                waypoint.mu[1] = goal.mu[1];
            }
            else {
                waypoint.mu[0] = state.quadrotor_pose->xyzrph[0] + config.lookahead_radius * cos (goal.mu[5]);
                waypoint.mu[1] = state.quadrotor_pose->xyzrph[1] + config.lookahead_radius * sin (goal.mu[5]);
            }

            publish_waypoints = 1;

            if (dist2d (state.quadrotor_pose, state.target_pose) < config.main_radius + config.lookahead_radius)
                state.mission_state = PREPARE_LANDING;

            break;

        // move to relative "look behind" point
        case PREPARE_LANDING:
            printf ("STATE: PREPARE_LANDING\n");
            // assign goal point
            goal.mu[0] = state.target_pose->xyzrph[0]
                + config.rel_look_behind.mu[0] * cos (state.target_pose->xyzrph[5])
                - config.rel_look_behind.mu[1] * sin (state.target_pose->xyzrph[5]);
            goal.mu[1] = state.target_pose->xyzrph[1]
                + config.rel_look_behind.mu[0] * sin (state.target_pose->xyzrph[5])
                + config.rel_look_behind.mu[1] * cos (state.target_pose->xyzrph[5]);
            goal.mu[2] = state.target_pose->xyzrph[2] + config.rel_look_behind.mu[2];
            goal.mu[5] = state.target_pose->xyzrph[5];

            // determine incremental waypoint
            cur_ang = atan2 (state.quadrotor_pose->xyzrph[1] - state.target_pose->xyzrph[1],
                            state.quadrotor_pose->xyzrph[0] - state.target_pose->xyzrph[0]);
            tmpx = state.target_pose->xyzrph[0]
                + config.rel_look_behind.mu[0] * cos (cur_ang + M_PI)
                - config.rel_look_behind.mu[1] * sin (cur_ang + M_PI);
            tmpy = state.target_pose->xyzrph[1]
                + config.rel_look_behind.mu[0] * sin (cur_ang + M_PI)
                + config.rel_look_behind.mu[1] * cos (cur_ang + M_PI);

            waypoint.mu[2] = goal.mu[2];
            if (sqrt (pow (goal.mu[0]-tmpx,2) + pow (goal.mu[1]-tmpy,2)) < config.lookahead_radius) {
                waypoint.mu[0] = goal.mu[0];
                waypoint.mu[1] = goal.mu[1];
                waypoint.mu[5] = goal.mu[5];
            }
            else {
                tmp = 2 * asin (config.lookahead_radius / 2.0 / config.main_radius);
                ang1 = cur_ang + tmp;
                ang2 = cur_ang - tmp;

                if (abs (normalize_angle (goal.mu[5] - (ang1 + M_PI))) < abs (normalize_angle (goal.mu[5] - (ang2 + M_PI))) )
                    tmp = ang1;
                else
                    tmp = ang2;

                // get nearest point on circle
                waypoint.mu[0] = state.target_pose->xyzrph[0] + config.main_radius * cos (tmp);
                waypoint.mu[1] = state.target_pose->xyzrph[1] + config.main_radius * sin (tmp);
                waypoint.mu[5] = tmp + M_PI;
            }

            publish_waypoints = 1;

            if (sqrt (pow (goal.mu[0]-state.quadrotor_pose->xyzrph[0],2) + 
                        pow (goal.mu[1]-state.quadrotor_pose->xyzrph[1],2)) < config.lookahead_radius && 
                        (state.controller && state.controller->auth_land)) {
                if (state.time_at_look_behind == 0)
                    state.time_at_look_behind = state.utime;
                else if ( (state.utime - state.time_at_look_behind) > 2e6 ) {
                    state.time_on_target = 0;
                    state.mission_state = BEGIN_DESCENT;
                }
            }
            else if (dist2d (state.quadrotor_pose, state.target_pose) > 5)
                state.mission_state = LOCK_ON_TARGET;
            break;

        // begin descent onto landing pad
        case BEGIN_DESCENT:
            printf ("STATE: BEGIN_DESCENT\n");
            goal.mu[5] = state.target_pose->xyzrph[5];
            goal.mu[0] = state.target_pose->xyzrph[0] - .02*cos(goal.mu[5]);
            goal.mu[1] = state.target_pose->xyzrph[1] - .02*sin(goal.mu[5]);
            goal.mu[2] = state.target_pose->xyzrph[2] - 0.70;

            double lb[3];
            lb[0] = state.target_pose->xyzrph[0]
                + config.rel_look_behind.mu[0] * cos (state.target_pose->xyzrph[5])
                - config.rel_look_behind.mu[1] * sin (state.target_pose->xyzrph[5]);
            lb[1] = state.target_pose->xyzrph[1]
                + config.rel_look_behind.mu[0] * sin (state.target_pose->xyzrph[5])
                + config.rel_look_behind.mu[1] * cos (state.target_pose->xyzrph[5]);
            lb[2] = state.target_pose->xyzrph[2] + config.rel_look_behind.mu[2];
            //printf ("lookback point set\n");

            double phi = atan2 (lb[1] - goal.mu[1], lb[0] - goal.mu[0]);


            double dist_to_target = sqrt (pow (goal.mu[0]-state.quadrotor_pose->xyzrph[0],2) + pow (goal.mu[1]-state.quadrotor_pose->xyzrph[1],2));
            printf ("distance to target: %4.4f\n", dist_to_target);

            if (dist_to_target < config.lookahead_radius) {
                waypoint.mu[0] = goal.mu[0];
                waypoint.mu[1] = goal.mu[1];
                waypoint.mu[2] = goal.mu[2];
            }
            else {
                waypoint.mu[0] = goal.mu[0] + (dist_to_target - config.lookahead_radius) * cos (phi);
                waypoint.mu[1] = goal.mu[1] + (dist_to_target - config.lookahead_radius) * sin (phi);
                waypoint.mu[2] = goal.mu[2] + (dist_to_target - config.lookahead_radius)/config.main_radius * config.rel_look_behind.mu[2];
                //waypoint.mu[0] = goal.mu[0] + 0.38 * dist_to_target * cos (phi);
                //waypoint.mu[1] = goal.mu[1] + 0.38 * dist_to_target * sin (phi);
                //waypoint.mu[2] = goal.mu[2] + 0.38 * dist_to_target/config.main_radius * config.rel_look_behind.mu[2];

            }

            waypoint.mu[5] = goal.mu[5];

            publish_waypoints = 1;



            bool within_volume = (sqrt (pow (goal.mu[0]-state.quadrotor_pose->xyzrph[0],2) + 
                        pow (goal.mu[1]-state.quadrotor_pose->xyzrph[1],2)) < 0.25 && 
                        fabs (goal.mu[2]-state.quadrotor_pose->xyzrph[2]) < .15);

            //if (!(state.controller && state.controller->auth_land)) {
                //state.mission_state = INITIALIZATION;
            //}
            /*if (within_volume && state.time_on_target == 0){
                state.time_on_target = state.utime;
                perllcm_ardrone_cmd_t_publish (state.lcm, config.ardrone_cmd_channel, &hover);
            }
            else if (within_volume && (state.utime-state.time_on_target) > 6e5){
                state.mission_state = FINAL_DESCENT;
            }*/
            if (within_volume) {
                state.mission_state = FINAL_DESCENT;
            }
            //else if (!within_volume && (state.utime - state.time_on_target) < 1e6){
                //state.time_on_target = state.utime;
                //printf ("stay in begin descent 2\n");
            //}
            else if (dist2d (state.quadrotor_pose, state.target_pose) > 5){
                state.mission_state = LOCK_ON_TARGET;
            }

            break;

        // go straight down onto target
        case FINAL_DESCENT:
            printf ("STATE: FINAL_DESCENT\n");
            goal.mu[5] = waypoint.mu[5] = state.target_pose->xyzrph[5];
            goal.mu[0] = waypoint.mu[0] = state.target_pose->xyzrph[0] - .02*cos(goal.mu[5]);
            goal.mu[1] = waypoint.mu[1] = state.target_pose->xyzrph[1] - .02*sin(goal.mu[5]);
            goal.mu[2] = waypoint.mu[2] = state.target_pose->xyzrph[2] - 0.00;

            publish_waypoints = 1;

            bool landing_volume = (sqrt (pow (goal.mu[0]-state.quadrotor_pose->xyzrph[0],2) + 
                        pow (goal.mu[1]-state.quadrotor_pose->xyzrph[1],2)) < 0.15 && 
                        fabs (goal.mu[2]-state.quadrotor_pose->xyzrph[2]) < .15);

            if (landing_volume){
                state.mission_state = EXECUTE_LANDING;
            }
            else if (dist2d (state.quadrotor_pose, state.target_pose) > 5){
                state.mission_state = LOCK_ON_TARGET;
            }

            break;

        // currently just above landing pad, we can now cut motors
        case EXECUTE_LANDING:
            printf ("STATE: EXECUTE_LANDING\n");
            goal.mu[0] = state.target_pose->xyzrph[0];
            goal.mu[1] = state.target_pose->xyzrph[1];
            goal.mu[2] = state.target_pose->xyzrph[2] - 0.0;
            goal.mu[5] = state.target_pose->xyzrph[5];

            waypoint.mu[0] = state.target_pose->xyzrph[0];
            waypoint.mu[1] = state.target_pose->xyzrph[1];
            waypoint.mu[2] = state.target_pose->xyzrph[2] - 0.0;
            waypoint.mu[5] = state.target_pose->xyzrph[5];

            publish_waypoints = 1;
            
            perllcm_ardrone_cmd_t_publish (state.lcm, config.ardrone_cmd_channel, &land);
            state.mission_state = MISSION_COMPLETE;

            break;

        case MISSION_COMPLETE:
            printf ("STATE: MISSION_COMPLETE\n");
            //perllcm_ardrone_cmd_t_publish (state.lcm, config.ardrone_cmd_channel, &land);
            state.done = 1;
            break;

        default:
            break;
    }



    /* publish waypoints, only allow 50 hz publishing */
    if (publish_waypoints && ((timestamp_now () - state.last_publish) > 20000)) {

        // lcmgl stuff.. for debug
        bot_lcmgl_t *lcmgl = bot_lcmgl_init (state.lcm, "DEBUG_MM");
        lcmglPushMatrix();
        lcmglRotated(180, 1, 0, 0);
        lcmglPointSize(10);

        // target
        if (state.target_pose) {
            lcmglPushMatrix ();
            lcmglTranslated (state.target_pose->xyzrph[0], state.target_pose->xyzrph[1], state.target_pose->xyzrph[2]);
            lcmglRotated (state.target_pose->xyzrph[5]*180/M_PI, 0, 0, 1);

            lcmglColor3f(0.0, 0.0, 0.0);
            lcmglBegin(LCMGL_QUADS);
            double l = .5, w = .4;
            lcmglVertex3d (l/2, w/2, 0);
            lcmglVertex3d (-l/2, w/2, 0);
            lcmglVertex3d (-l/2, -w/2, 0);
            lcmglVertex3d (l/2, -w/2, 0);
            lcmglEnd();

            lcmglPopMatrix ();
        }

        waypoint.mu[5] = normalize_angle (waypoint.mu[5]);
        goal.mu[5] = normalize_angle (goal.mu[5]);
        perllcm_pose3d_t_publish (state.lcm, config.waypoint_channel, &waypoint);
        perllcm_pose3d_t_publish (state.lcm, config.goal_channel, &goal);

        // goal heading line
        lcmglPushMatrix ();
        lcmglTranslated (goal.mu[0], goal.mu[1], goal.mu[2]);
        lcmglRotated (goal.mu[5] * 180/M_PI, 0, 0, 1);
        lcmglColor3f (0.0, 0.0, 0.0);
        lcmglPointSize(5);
        lcmglBegin (LCMGL_LINES);
        lcmglVertex3d (0.0, 0.0, 0.0);
        lcmglVertex3d (0.1, 0.0, 0.0);
        lcmglEnd ();
        lcmglPopMatrix();

        // goal position
        lcmglColor3f (0.0, 1.0, 1.0);
        lcmglPointSize(10);
        lcmglBegin (LCMGL_POINTS);
        lcmglVertex3d (goal.mu[0], goal.mu[1], goal.mu[2]);
        lcmglEnd ();

        // waypoint heading line
        lcmglPushMatrix ();
        lcmglTranslated (waypoint.mu[0], waypoint.mu[1], waypoint.mu[2]);
        lcmglRotated (waypoint.mu[5] * 180/M_PI, 0, 0, 1);
        lcmglColor3f (0.0, 0.0, 0.0);
        lcmglPointSize(5);
        lcmglBegin (LCMGL_LINES);
        lcmglVertex3d (0.0, 0.0, 0.0);
        lcmglVertex3d (0.1, 0.0, 0.0);
        lcmglEnd ();
        lcmglPopMatrix();

        // waypoint position
        lcmglColor3f (1.0, 0.0, 1.0);
        lcmglPointSize(10);
        lcmglBegin (LCMGL_POINTS);
        lcmglVertex3d (waypoint.mu[0], waypoint.mu[1], waypoint.mu[2]);
        lcmglEnd ();


        lcmglPopMatrix();
        bot_lcmgl_switch_buffer(lcmgl);
        bot_lcmgl_destroy(lcmgl);

        state.last_publish = timestamp_now ();
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

    // load in config file option
    manager_load_cfg (&config);
    config.target_timeout = 10;
    //config.rel_look_behind.mu[0] = -1.00;
    //config.rel_look_behind.mu[2] = -0.80;
    config.main_radius = sqrt (pow (config.rel_look_behind.mu[0],2) + pow (config.rel_look_behind.mu[1],2));
    //config.lookahead_radius = 0.6;
    
    // read in the command line options
    getopt_t *gopt = getopt_create ();
    
    getopt_add_description (gopt, "Mission Manager for Drone");
    getopt_add_bool (gopt, 'v', "nav", 0, "Run using navigator");
    getopt_add_bool (gopt, 'D', "daemon", 0, "Run as system daemon");
    getopt_add_bool (gopt, 'h', "help",   0, "Display Help");

    if (!getopt_parse (gopt, argc, argv, 1)) {
        getopt_do_usage (gopt,"");
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt,"");
        exit (EXIT_SUCCESS);;
    }
    
    //start as daemon if asked
    if (getopt_get_bool (gopt, "daemon")) {
        daemon_fork ();
        state.is_daemon = 1;
    }
    else
        state.is_daemon = 0;
    
    // init lcm
    state.lcm = lcm_create (NULL);
    if (!state.lcm) {
        ERROR ("lcm_create() failed");
        exit (EXIT_FAILURE);
    }

    if (getopt_get_bool (gopt, "nav"))
        state.use_nav = 1;

    perllcm_ardrone_cmd_t_subscribe (state.lcm, config.ardrone_cmd_channel, &quad_cmd_cb, NULL);

    if (state.use_nav) {
        // subscribe to quadrotor pose
        perllcm_position_t_subscribe (state.lcm, config.quadrotor_pose_channel, &quad_perllcm_position_cb, NULL);

        // subscribe to target pose
        perllcm_position_t_subscribe (state.lcm, config.target_pose_channel, &targ_perllcm_position_cb, NULL);
    }
    else {
        // subscribe to quadrotor pose
        senlcm_mocap_t_subscribe (state.lcm, "MOCAP_POSE_ARDRONE", &quad_senlcm_mocap_cb, NULL);

        // subscribe to target pose
        senlcm_mocap_t_subscribe (state.lcm, "MOCAP_POSE_TARGET", &targ_senlcm_mocap_cb, NULL);
    }
                                       

    // main loop
    while (!state.done) {
        lcm_handle (state.lcm);
        update_mission ();
    }



    if (state.quadrotor_pose)
        perllcm_position_t_destroy (state.quadrotor_pose);
             
    if (state.target_pose)
        perllcm_position_t_destroy (state.target_pose);

    if (state.controller)
        perllcm_ardrone_cmd_t_destroy (state.controller);

    printf ("\nDone.\n");
    exit (EXIT_SUCCESS);
}


