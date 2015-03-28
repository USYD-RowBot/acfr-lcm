/*
     Munge GPSD output and Evologics AHRS for SHIP STATUS

    Lachlan Toohey
    ACFR
    25-3-15
*/

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <libgen.h>
#include <pthread.h>
#include <sys/select.h>

#include <bot_param/param_client.h>

#include "perls-common/timestamp.h"
#include "perls-lcmtypes/senlcm_gpsd3_t.h"
#include "perls-lcmtypes/senlcm_ahrs_t.h"
#include "perls-lcmtypes/acfrlcm_ship_status_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

/*#ifndef BOT_CONF_DIR
#define DEFAULT_BOT_CONF_PATH "../config/master.cfg"
#else
#define DEFAULT_BOT_CONF_PATH BOT_CONF_DIR "/master.cfg"
#endif*/

typedef struct
{
    pthread_mutex_t data_lock;
    acfrlcm_ship_status_t status;

    int64_t ahrs_utime;
    int64_t gpsd_utime;

    int64_t last_utime;

    char *output_channel;

    lcm_t *lcm;
} state_t;


void
ahrs_callback(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_ahrs_t *ahrs, void *u)
{
    state_t *state = (state_t *)u;

    //pthread_mutex_lock(&state->data_lock);
    state->ahrs_utime = ahrs->utime;
    state->status.roll = ahrs->roll;
    state->status.pitch = ahrs->pitch;
    state->status.heading = ahrs->heading;
    //pthread_mutex_unlock(&state->data_lock);
}

void
gpsd_callback(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_gpsd3_t *gps, void *u)
{
    state_t *state = (state_t *)u;

    //pthread_mutex_lock(&state->data_lock);
    state->gpsd_utime = gps->utime;
    state->status.latitude = gps->fix.latitude;
    state->status.longitude = gps->fix.longitude;
    //pthread_mutex_unlock(&state->data_lock);
}

void
heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;

    //pthread_mutex_lock(&state->data_lock);
    if (state->gpsd_utime > state->last_utime || state->ahrs_utime > state->last_utime)
    {
        state->status.utime = timestamp_now();
        //state->status.utime = max(state->gps_utime, state->ahrs_utime);
        state->last_utime = state->status.utime;

        acfrlcm_ship_status_t_publish(state->lcm, state->output_channel, &state->status);

    }
    //pthread_mutex_unlock(&state->data_lock);
}

int program_exit;
void
signal_handler(int sigNum)
{
    // do a safe exit
    program_exit = 1;
}

int
main(int argc, char **argv)
{
    state_t state;

    // install the signal handler
	program_exit = 0;
    signal(SIGINT, signal_handler);

    // read the config file
	char rootkey[64];
	char key[64];

    state.lcm = lcm_create(NULL);

    BotParam *cfg = NULL;
    cfg = bot_param_new_from_server (state.lcm, 1);
    if(cfg == NULL)
        return 0;


    sprintf(rootkey, "sensors.%s", basename(argv[0]));

	char *ahrs_target;
	sprintf(key, "%s.ahrs_channel", rootkey);
	ahrs_target = bot_param_get_str_or_fail(cfg, key);

	char *gpsd_target;
	sprintf(key, "%s.gpsd_channel", rootkey);
	gpsd_target = bot_param_get_str_or_fail(cfg, key);

	char *heartbeat_target;
	sprintf(key, "%s.heartbeat_channel", rootkey);
	heartbeat_target = bot_param_get_str_or_fail(cfg, key);

	sprintf(key, "%s.output_channel", rootkey);
	state.output_channel = bot_param_get_str_or_fail(cfg, key);

	sprintf(key, "%s.ship_name", rootkey);
	state.status.name = bot_param_get_str_or_fail(cfg, key);

	sprintf(key, "%s.ship_id", rootkey);
	state.status.ship_id = bot_param_get_int_or_fail(cfg, key);

	// lets start LCM
    senlcm_gpsd3_t_subscribe(state.lcm, gpsd_target, &gpsd_callback, &state);
    senlcm_ahrs_t_subscribe(state.lcm, ahrs_target, &ahrs_callback, &state);
    perllcm_heartbeat_t_subscribe(state.lcm, heartbeat_target, &heartbeat_handler, &state);

        //pthread_mutex_init(&state->data_lock);
	int lcm_fd = lcm_get_fileno(state.lcm);
	fd_set rfds;

	// listen to LCM, serial has been delegated
	while(!program_exit)
	{
	    FD_ZERO(&rfds);
        FD_SET(lcm_fd, &rfds);

		struct timeval tv;
		tv.tv_sec = 1;
		tv.tv_usec = 0;

	    int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
            lcm_handle(state.lcm);
	}

	return 0;
}
