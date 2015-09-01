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
#include <small/Pose3D.hh>

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
    acfrlcm_ship_status_t status;

    int64_t ahrs_utime;
    int64_t gpsd_utime;

    char *output_channel;

    lcm_t *lcm;

    SMALL::Pose3D attitude_offset;
    SMALL::Pose3D gps_offset;
} state_t;


void
ahrs_callback(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_ahrs_t *ahrs, void *u)
{
    state_t *state = (state_t *)u;

    SMALL::Pose3D raw_attitude, transformed_attitude;
    raw_attitude.setRollPitchYawRad(ahrs->roll, ahrs->pitch, ahrs->heading);
    
    transformed_attitude = state->attitude_offset.compose(raw_attitude);

    state->ahrs_utime = ahrs->utime;
    transformed_attitude.getRollPitchYawRad(state->status.roll, state->status.pitch, state->status.heading);
}

void
gpsd_callback(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_gpsd3_t *gps, void *u)
{
    //printf("Received GPSD data with timestamp %f\n", gps->utime/1e6);
    state_t *state = (state_t *)u;

    state->gpsd_utime = gps->utime;
    state->status.latitude = gps->fix.latitude;
    state->status.longitude = gps->fix.longitude;
}

void
heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;

    // check if the gps and ahrs data have not gone stale.  This should be
    // configurable.
    if (hb->utime - state->gpsd_utime < 2e6 && hb->utime - state->ahrs_utime < 2e6)
    {
        state->status.utime = hb->utime;
        acfrlcm_ship_status_t_publish(state->lcm, state->output_channel, &state->status);

    } else {
        printf("Stale ship_status data\n");
    }
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

    // initilise the time to zero so it accepts the first data received
    state.ahrs_utime = 0;
    state.gpsd_utime = 0;

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

       // read in the attitude offsets to be applied to the ins data
        sprintf(key, "%s.attitude_offset", rootkey);
        if (bot_param_has_key(cfg, key))
        {
            double attoff[6];
            bot_param_get_double_array_or_fail(cfg, key, attoff, 6);
            state.attitude_offset.setPosition(attoff[0], attoff[1], attoff[2]);
            state.attitude_offset.setRollPitchYawRad(attoff[3], attoff[4], attoff[5]);
        }
    
        // read in the position offsets to be applied to the gps data
        sprintf(key, "%s.gps_offset", rootkey);
        if (bot_param_has_key(cfg, key))
        {
            double gpsoff[6];
            bot_param_get_double_array_or_fail(cfg, key, gpsoff, 6);
            state.gps_offset.setPosition(gpsoff[0], gpsoff[1], gpsoff[2]);
            state.gps_offset.setPosition(gpsoff[3], gpsoff[4], gpsoff[5]);
        }

	// lets start LCM
        senlcm_gpsd3_t_subscribe(state.lcm, gpsd_target, &gpsd_callback, &state);
        senlcm_ahrs_t_subscribe(state.lcm, ahrs_target, &ahrs_callback, &state);
        perllcm_heartbeat_t_subscribe(state.lcm, heartbeat_target, &heartbeat_handler, &state);

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
