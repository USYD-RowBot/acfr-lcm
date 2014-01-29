#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <queue>

#include <lcm/lcm.h>
#include "perls-lcmtypes/senlcm_mocap_t.h"
#include "perls-lcmtypes/perllcm_pose3d_t.h"
#include "perls-lcmtypes/perllcm_tag_detection_collection_t.h"
#include "perls-lcmtypes/perllcm_ardrone_cmd_t.h"
#include "perls-lcmtypes/perllcm_ardrone_state_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"

#include <bot_lcmgl_client/lcmgl.h>

#include "navigator.h"

using namespace std;

struct config_t
{
    bool track_target;
    bool use_mocap;
    bool use_state;
    bool use_control_input;

    char quadrotor_mocap_channel[256];
    char april_pose_channel[256];
    char ardrone_state_channel[256];
    char hb_channel[256];
    char image_sync_channel[256];
    char control_input_channel[256];
    
    char quadrotor_position_channel[256];
    char target_position_channel[256];
};

struct state_t
{
    int done;
    int is_daemon;
    lcm_t *lcm;

    queue<observation*> message_queue;

    config_t config;
};

// Init state and config structures
state_t state = {0};

// Thread synchronization
pthread_mutex_t MUTEX_STATE = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t COND_NEW_DATA = PTHREAD_COND_INITIALIZER;


//----------------------------------------------------------------------
// Loads the required info from the .cfg file into the state
//----------------------------------------------------------------------
void
navigator_load_cfg (config_t *config)
{
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        ERROR ("Could not create configuration parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
        exit (EXIT_FAILURE);
    }

    config->track_target = bot_param_get_int_or_fail (param, "navigator.est.track_target");
    config->use_mocap = bot_param_get_int_or_fail (param, "navigator.est.use_mocap");
    config->use_state = bot_param_get_int_or_fail (param, "navigator.est.use_state");
    config->use_control_input = bot_param_get_int_or_fail (param, "navigator.est.use_control_input");
    strcpy (config->quadrotor_position_channel, bot_param_get_str_or_fail (param, "navigator.quad_pose_channel"));
    strcpy (config->target_position_channel, bot_param_get_str_or_fail (param, "navigator.targ_pose_channel"));
    strcpy (config->hb_channel, bot_param_get_str_or_fail (param, "navigator.heartbeat_channel"));
    strcpy (config->quadrotor_mocap_channel, bot_param_get_str_or_fail (param, "navigator.quad_mocap_channel"));
    strcpy (config->april_pose_channel, bot_param_get_str_or_fail (param, "apriltags.tag_detection_channel"));
    strcpy (config->ardrone_state_channel, bot_param_get_str_or_fail (param, "ardrone-driver.state_channel"));
    strcpy (config->image_sync_channel, bot_param_get_str_or_fail (param, "ardrone-driver.sync_channel"));
    strcpy (config->control_input_channel, bot_param_get_str_or_fail (param, "ardrone-driver.drive_channel"));
    
}

//----------------------------------------------------------------------------------
// quadrotor pose callback
//----------------------------------------------------------------------------------
static void
quad_senlcm_mocap_cb (const lcm_recv_buf_t *rbug, const char *channel,
                                const  senlcm_mocap_t *msg, void *user)
{
    if (!msg->valid || msg->residual > 10 || msg->residual < 0.1)
        return;

    senlcm_mocap_t *mocap_msg = senlcm_mocap_t_copy (msg);

    pthread_mutex_lock (&MUTEX_STATE);
        state.message_queue.push (new mocap_obs (mocap_msg));
        pthread_cond_signal (&COND_NEW_DATA);
    pthread_mutex_unlock (&MUTEX_STATE);
}

//----------------------------------------------------------------------------------
// april pose callback
//----------------------------------------------------------------------------------
static void
perllcm_tag_cb (const lcm_recv_buf_t *rbug, const char *channel,
                const  perllcm_tag_detection_collection_t *msg, void *user)
{
    perllcm_tag_detection_collection_t *april_msg = perllcm_tag_detection_collection_t_copy (msg);

    pthread_mutex_lock (&MUTEX_STATE);
        state.message_queue.push (new april_obs (april_msg));
        pthread_cond_signal (&COND_NEW_DATA);
    pthread_mutex_unlock (&MUTEX_STATE);
}

//----------------------------------------------------------------------------------
// ardrone state callback
//----------------------------------------------------------------------------------
static void
perllcm_ardrone_state_cb (const lcm_recv_buf_t *rbug, const char *channel,
                                const  perllcm_ardrone_state_t *msg, void *user)
{
    perllcm_ardrone_state_t *state_msg = perllcm_ardrone_state_t_copy (msg);

    pthread_mutex_lock (&MUTEX_STATE);
        state.message_queue.push (new state_obs (state_msg));
        pthread_cond_signal (&COND_NEW_DATA);
    pthread_mutex_unlock (&MUTEX_STATE);
}

//----------------------------------------------------------------------------------
// heartbeat callback for publishing
//----------------------------------------------------------------------------------
static void
perllcm_heartbeat_cb (const lcm_recv_buf_t *rbug, const char *channel,
                                const  perllcm_heartbeat_t *msg, void *user)
{
    pthread_mutex_lock (&MUTEX_STATE);
        state.message_queue.push (new time_obs (msg->utime));
        pthread_cond_signal (&COND_NEW_DATA);
    pthread_mutex_unlock (&MUTEX_STATE);
}

//----------------------------------------------------------------------------------
// control input callback
//----------------------------------------------------------------------------------
static void
perllcm_ardrone_drive_cb (const lcm_recv_buf_t *rbug, const char *channel,
                                const  perllcm_ardrone_drive_t *msg, void *user)
{
    perllcm_ardrone_drive_t *control_msg = perllcm_ardrone_drive_t_copy (msg);

    pthread_mutex_lock (&MUTEX_STATE);
        state.message_queue.push (new control_obs (control_msg));
        pthread_cond_signal (&COND_NEW_DATA);
    pthread_mutex_unlock (&MUTEX_STATE);
}

//----------------------------------------------------------------------------------
// image sync callback
//----------------------------------------------------------------------------------
static void
bot_core_sync_cb (const lcm_recv_buf_t *rbug, const char *channel,
                                const  bot_core_image_sync_t *msg, void *user)
{
    bot_core_image_sync_t *sync_msg = bot_core_image_sync_t_copy (msg);

    pthread_mutex_lock (&MUTEX_STATE);
        state.message_queue.push (new sync_obs (sync_msg));
        pthread_cond_signal (&COND_NEW_DATA);
    pthread_mutex_unlock (&MUTEX_STATE);
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
// Data Processing Thread
//----------------------------------------------------------------------------------
void *
process_thread (void *input)
{
    state_t *st = (state_t *) input;
    lcm_t *pub_lcm = lcm_create (NULL);

    bool done = false;
    bool publish_flag = false;
    observation *cur_msg;
    double quad_to_cam[] = {0.2002, 0.0161, 0.0167, 0.7832, -0.0052, 1.5975};
    int64_t last_pubish = 0;

    navigator *nav = new navigator (quad_to_cam, st->config.track_target);

    while (!done) {
        publish_flag = true;

        pthread_mutex_lock (&MUTEX_STATE);
            /* wait for mutex */

            while (st->message_queue.empty() && !st->done) {
                pthread_cond_wait (&COND_NEW_DATA, &MUTEX_STATE);
            }

            done = st->done;
            if (!done) {
                cur_msg = st->message_queue.front();
                st->message_queue.pop();
            }
        pthread_mutex_unlock (&MUTEX_STATE);

        if (done) break;


        /* not done and new message came in */
        switch (cur_msg->get_type ()) {
            case TIME_OBS:
                nav->add_time_observation (((time_obs*) cur_msg)->obs);
                publish_flag = true;
                delete cur_msg;
                break;
            case APRIL_OBS:
                nav->add_april_observation (((april_obs*) cur_msg)->obs);
                delete ((april_obs*) cur_msg);
                break;
            case MOCAP_OBS:
                nav->add_mocap_observation (((mocap_obs*) cur_msg)->obs);
                delete ((mocap_obs*) cur_msg);
                break;
            case STATE_OBS:
                nav->add_state_observation (((state_obs*) cur_msg)->obs);
                delete ((state_obs*) cur_msg);
                break;
            case SYNC_OBS:
                nav->add_sync_observation (((sync_obs*) cur_msg)->obs);
                delete ((sync_obs*) cur_msg);
                break;
            case CONTROL_OBS:
                nav->set_control_input (((control_obs*) cur_msg)->obs);
                delete ((control_obs*) cur_msg);
                break;
            case BASE_OBS:
                delete cur_msg;
                break;
        }

        /* get and publish data, but only allow 50 hz publishing */
        if (publish_flag && ((timestamp_now () - last_pubish) > 2e4) ) {
            perllcm_position_t *quad = nav->get_quadrotor_state ();
            perllcm_position_t *targ = nav->get_target_state ();

            if (quad)
                perllcm_position_t_publish (pub_lcm, st->config.quadrotor_position_channel, quad);
            if (targ)
                perllcm_position_t_publish (pub_lcm, st->config.target_position_channel, targ);

            if (quad) perllcm_position_t_destroy (quad);
            if (targ) perllcm_position_t_destroy (targ);

            last_pubish = timestamp_now ();
        }
    }

    delete nav;
    lcm_destroy (pub_lcm);
    printf ("\nThread closing.\n");

    return NULL;
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
    struct sigaction act;
    act.sa_sigaction = my_signal_handler;
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);

    // load in config file option
    navigator_load_cfg (&state.config);
    
    // read in the command line options
    getopt_t *gopt = getopt_create ();
    
    getopt_add_description (gopt, "Navigator for Drone");
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
    
    // init lcm
    state.lcm = lcm_create (NULL);
    if (!state.lcm) {
        ERROR ("lcm_create() failed");
        exit (EXIT_FAILURE);
    }



    /* subscribe to quadrotor pose */
    if (state.config.use_mocap)
        senlcm_mocap_t_subscribe (state.lcm, state.config.quadrotor_mocap_channel, &quad_senlcm_mocap_cb, NULL);

    /* subscribe to april pose and image sync observations */
    if (state.config.track_target) {
        perllcm_tag_detection_collection_t_subscribe (state.lcm, state.config.april_pose_channel, &perllcm_tag_cb, NULL);
        bot_core_image_sync_t_subscribe (state.lcm, state.config.image_sync_channel, &bot_core_sync_cb, NULL);
    }

    /* subscribe to heartbeat for publishing */
    perllcm_heartbeat_t_subscribe (state.lcm, state.config.hb_channel, &perllcm_heartbeat_cb, NULL);

    /* subscribe to ardrone state */
    if (state.config.use_state)
        perllcm_ardrone_state_t_subscribe (state.lcm, state.config.ardrone_state_channel, &perllcm_ardrone_state_cb, NULL);

    /* subscribe to ardrone drive */
    if (state.config.use_control_input)
        perllcm_ardrone_drive_t_subscribe (state.lcm, state.config.control_input_channel, &perllcm_ardrone_drive_cb, NULL);


    /* create data processing thread */
    pthread_t *pt = new pthread_t;
    pthread_create (pt, NULL, &process_thread, &state);



    /* main loop */
    bool done = false;
    while (!done) {
        lcm_handle (state.lcm);

        pthread_yield ();
        pthread_mutex_lock (&MUTEX_STATE);
            done = state.done;
        pthread_mutex_unlock (&MUTEX_STATE);
    }


    /* cleanup */
    pthread_mutex_lock (&MUTEX_STATE);
        while (!state.message_queue.empty()) {
            observation *m = state.message_queue.front();

            /* not done and new message came in */
            switch (m->get_type ()) {
                case TIME_OBS:
                    delete m;
                    break;
                case APRIL_OBS:
                    delete ((april_obs*) m);
                    break;
                case MOCAP_OBS:
                    delete ((mocap_obs*) m);
                    break;
                case STATE_OBS:
                    delete ((state_obs*) m);
                    break;
                case SYNC_OBS:
                    delete ((sync_obs*) m);
                    break;
                case CONTROL_OBS:
                    delete ((control_obs*) m);
                    break;
                case BASE_OBS:
                    delete m;
                    break;
            }

            state.message_queue.pop();
        }
    pthread_mutex_unlock (&MUTEX_STATE);

    printf ("\nDone.\n");
    exit (EXIT_SUCCESS);
}

