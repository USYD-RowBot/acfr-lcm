/*
    Sirius monitor

    listens to sensors and if they have stoppped working
    or are not putting out the correct data then we send
    a mission abort command

    Christian Lees
    ACFR
    3/2/12
*/


#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <lcm/lcm.h>

#include "perls-common/timestamp.h"
#include "perls-common/lcm_util.h"

#include <bot_param/param_client.h>
#include "perls-lcmtypes/acfrlcm_auv_control_response_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd5_t.h"
#include "perls-lcmtypes/senlcm_parosci_t.h"
#include "perls-lcmtypes/senlcm_gpsd3_t.h"
#include "perls-lcmtypes/acfrlcm_auv_mission_command_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

#ifndef BOT_CONF_DIR
#define DEFAULT_BOT_CONF_PATH "../config/master.cfg"
#else
#define DEFAULT_BOT_CONF_PATH BOT_CONF_DIR "/master.cfg"
#endif

// Timeout variables
#define RDI_TIMEOUT 10
#define DEPTH_TIMEOUT 10
#define GPS_TIMEOUT 10
#define NUM_BTV_LOCK 10

typedef struct
{
    lcm_t *lcm;

    int rdi_count;
    int depth_count;
    int gps_count;

    int bottom_lock;
    int bottom_lock_count;
    double depth;
    double max_depth;

    int abort;

    acfrlcm_auv_control_response_t cr;
} state_t;

void
heartbeat_callback(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *user)
{
    state_t *state = (state_t *)user;

    int abort = 0;
    // check to see if any of the timers have gone over
    if((state->rdi_count > RDI_TIMEOUT) || (state->depth_count > DEPTH_TIMEOUT) || (state->gps_count > GPS_TIMEOUT))
        abort = 1;

    if((state->cr.depth_mode == ACFRLCM_AUV_CONTROL_RESPONSE_T_MODE_ALT) && !state->bottom_lock)
        abort = 1;

    if (state->depth > state->max_depth)
    {
        printf("ABORT: Triggered on max_depth of %f with depth of %f\n", state->max_depth, state->depth);
        abort = 1;
    }

    if(abort)
    {
        // send an abort
        printf("Sending an abort, counts RDI: %d, DEPTH: %d, GPS: %d, BL: %d\n",
               state->rdi_count, state->depth_count, state->gps_count, state->bottom_lock);
        acfrlcm_auv_mission_command_t mc;
        memset(&mc, 0, sizeof(acfrlcm_auv_mission_command_t));
        mc.message = ACFRLCM_AUV_MISSION_COMMAND_T_ABORT;
        mc.source = ACFRLCM_AUV_MISSION_COMMAND_T_NETWORK;
        char str = 0;
        mc.str = &str;
        mc.utime = timestamp_now();
        acfrlcm_auv_mission_command_t_publish(state->lcm, "MISSION_COMMAND", &mc);
    }

    printf("Counts RDI: %d, DEPTH: %d, GPS: %d, BL: %d depth: %f\n",
           state->rdi_count, state->depth_count, state->gps_count, state->bottom_lock, state->depth);
    fflush(NULL);

    // increment the counters
    state->rdi_count++;
    state->depth_count++;
    if(state->depth < 2.0)
        state->gps_count++;
    else
        state->gps_count = 0.0;

    return abort;
}

static void
control_response_callback(const lcm_recv_buf_t *rbuf, const char *channel, const acfrlcm_auv_control_response_t *cr, void *user)
{
    state_t *state = (state_t *)user;
    memcpy(&state->cr, cr, sizeof(acfrlcm_auv_control_response_t));
}

static void
command_ack_callback(const lcm_recv_buf_t *rbuf, const char *channel, const acfrlcm_auv_mission_command_t *ca, void *user)
{
    state_t *state = (state_t *)user;
    // listen for the abort ack
    if(ca->message == ACFRLCM_AUV_MISSION_COMMAND_T_ABORT)
        state->abort = 0;
}

void parosci_callback(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_parosci_t *parosci, void *user)
{
    state_t *state = (state_t *)user;
    state->depth_count = 0;
    state->depth = parosci->depth;
}

void gps_callback(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_gpsd3_t *gps, void *user)
{
    state_t *state = (state_t *)user;
    state->gps_count = 0;
}

// we detect a lack of bottom lock if the btv_status flag is not 0 for more then NUM_BTV_LOCK pings
void rdi_callback(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_rdi_pd5_t *rdi, void *user)
{
    state_t *state = (state_t *)user;
    state->rdi_count = 0;

    if(rdi->pd4.btv_status == 0)
    {
        state->bottom_lock = 1;
        state->bottom_lock_count = 0;
    }
    else
    {
        if(state->bottom_lock_count > NUM_BTV_LOCK)
            state->bottom_lock = 0;
        else
            state->bottom_lock = 1;

        state->bottom_lock_count++;
    }
}

void load_config(state_t *state, char *name)
{
    // read the config file
    BotParam *cfg;
    char rootkey[64];
    char key[64];

    char *path = getenv ("BOT_CONF_PATH");
    if (!path)
        path = DEFAULT_BOT_CONF_PATH;
    cfg = bot_param_new_from_file(path);
    if(cfg == NULL)
    {
        printf("cound not open config file\n");
        return 0;
    }

    sprintf(rootkey, "acfr.%s", name);

    // read the parameters from the config file
    sprintf(key, "%s.max_depth", rootkey);
    state->max_depth = bot_param_get_double_or_fail(cfg, key);
    printf("%s: setting max_depth of %f\n", rootkey, state->max_depth);
}

int program_exit;
void signal_handler(int sig_num)
{
    // do a safe exit
    program_exit = 1;
}

int main(int argc, char **argv)
{

    program_exit = 0;
    signal(SIGINT, signal_handler);

    state_t state;

    // set it all to 0
    memset(&state, 0, sizeof(state_t));

    // load config parameters
    load_config(&state, basename(argv[0]));

    // start lcm
    state.lcm = lcm_create(NULL);

    // subscribe

    acfrlcm_auv_mission_command_t_subscribe(state.lcm, "COMMAND_ACK", &command_ack_callback, &state);
    acfrlcm_auv_control_response_t_subscribe(state.lcm, "CONTROL_RESPONSE", &control_response_callback, &state);
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_callback, &state);
    senlcm_parosci_t_subscribe(state.lcm, "PAROSCI", &parosci_callback, &state);
    senlcm_rdi_pd5_t_subscribe(state.lcm, "RDI", &rdi_callback, &state);
    senlcm_gpsd3_t_subscribe(state.lcm, "GPSD_CLIENT", &gps_callback, &state);

    // process
    struct timeval tv;
    while(!program_exit)
    {
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        lcmu_handle_timeout(state.lcm, &tv);
    }

    return 1;
}
