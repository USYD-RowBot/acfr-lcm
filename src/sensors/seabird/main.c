/* 	Seabird LCM module
	uses code lifted from the original vehicle code, SIO_seabird.c
	
	30/9/10
	Christian Lees
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <bot_param/param_client.h>

#include "perls-common/lcm_util.h"
#include "perls-common/units.h"
#include "perls-common/timestamp.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-lcmtypes/senlcm_seabird_t.h"
#include "perls-lcmtypes/acfrlcm_auv_acfr_nav_t.h"
#include "perls-lcmtypes/acfrlcm_auv_relay_t.h"

#define SEABIRD_AUTOSAMPLE_CMD   "AUTOSAMPLE=Y\r\n"
#define SEABIRD_AUTORUN_CMD   "AUTORUN=Y\r\n"
#define SEABIRD_GO_CMD   "GO\r\n"
#define SEABIRD_START_CMD   "START\r\n"
#define SEABIRD_STOP_CMD     "STOP\r\n" /* sample & send one measurement  */
#define SEABIRD_NAVG_CMD     "NAVG=1\r\n" /* number of depth values to average @16Hz*/
#define SEABIRD_OUTPUTFORMAT_CMD "OUTPUTFORMAT=3\r\n" /* output in meters of salt water */
#define SEABIRD_DECIMALS_CMD "DECIMALS=3\r\n" /* number of decimals */
#define SEABIRD_LAT_CMD      "LATITUDE=22\r\n"

// structure that holds the data for the callbacks
typedef struct {
	double pumpOnDepth;
	double pumpOffDepth;
	double depthTestTime;
	generic_sensor_driver_t *gsd;
	int64_t checkTime;
	int pumpState;
} seabirdState;


void initSeabird(generic_sensor_driver_t *gsd) {
	gsd_write(gsd, SEABIRD_AUTOSAMPLE_CMD, strlen(SEABIRD_AUTOSAMPLE_CMD));
	usleep(1e5);
	gsd_write(gsd, SEABIRD_AUTORUN_CMD, strlen(SEABIRD_AUTORUN_CMD));
	usleep(1e5);
	gsd_write(gsd, SEABIRD_GO_CMD, strlen(SEABIRD_GO_CMD));
	usleep(1e5);
}


static void
acfr_nav_callback (const lcm_recv_buf_t *rbuf, const char *channel, const acfrlcm_auv_acfr_nav_t *nav, void *user) 
{    
    seabirdState *state = (seabirdState *)user;
	double depth = nav->depth;
	if(timestamp_now() > state->checkTime) {
		state->checkTime = timestamp_now() + (int64_t)(state->depthTestTime * 1e6);

		acfrlcm_auv_relay_t relay;
        relay.utime = timestamp_now();
        char str[] = "seabird";
        relay.channel = str;
		if(!state->pumpState) {
			// pump is currently off
			if(depth > state->pumpOnDepth) {
                printf("Turning pump on\n");
				state->pumpState = 1;
                relay.state = 1;
                acfrlcm_auv_relay_t_publish (state->gsd->lcm, "RELAY", &relay);
                tcsendbreak(state->gsd->fd, 0);
                sleep(2);
                initSeabird(state->gsd);
			}
		}
		else {
			if(depth < state->pumpOffDepth) {
				printf("Turning pump off\n");
                state->pumpState = 0;
                relay.state = 0;
                acfrlcm_auv_relay_t_publish (state->gsd->lcm, "RELAY", &relay);
			}
		}
	}
}


static int parseSeabird(char *buf, senlcm_seabird_t *ctd) { 
	if (5 == sscanf(buf,"%lf, %lf, %lf, %lf, %lf", &ctd->temperature, &ctd->conductivity, &ctd->pressure, &ctd->salinity, &ctd->speed_of_sound))
		return 1;
	else 
		return 0;
}


void relay_callback(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_relay_t *msg, void *u)
{
    seabirdState *state = (seabirdState *)u;
    
    if(!strcmp(msg->channel, "seabird"))
        if(msg->state == 1)
        {
            printf("Reprogramming the CT\n");
            tcsendbreak(state->gsd->fd, 0);
            sleep(2);
            initSeabird(state->gsd);    
        }
    
}


// Process LCM messages with callbacks, done in a thread because the rest relies on the GSD code
static void *
lcmThread (void *context) {
    generic_sensor_driver_t *gsd = (generic_sensor_driver_t *) context;
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    while (!gsd->done) {
        lcmu_handle_timeout(gsd->lcm, &tv);
    }
    
    return 0;
}

static int myopts(generic_sensor_driver_t *gsd) {
    getopt_add_description (gsd->gopt, "Seabird driver.");
    return 0;
}


int main (int argc, char *argv[]) {

    seabirdState state;
		
    state.gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_canonical(state.gsd, '\r','\n');
    gsd_launch(state.gsd);
    
    gsd_flush(state.gsd);
    gsd_reset_stats(state.gsd);

    char key[128];
    sprintf(key, "%s.pumpOnDepth", state.gsd->rootkey);
    state.pumpOnDepth = bot_param_get_double_or_fail(state.gsd->params, key);

    sprintf(key, "%s.pumpOffDepth", state.gsd->rootkey);
    state.pumpOffDepth = bot_param_get_double_or_fail(state.gsd->params, key);

    sprintf(key, "%s.depthTestTime", state.gsd->rootkey);
    state.depthTestTime = bot_param_get_double_or_fail(state.gsd->params, key);

	initSeabird(state.gsd);
    
    state.checkTime = timestamp_now();
    state.pumpState = 0;
    
	// create an LCM thread to listen so the depth sensor
    pthread_t tid;
    pthread_create(&tid, NULL, lcmThread, state.gsd);
    pthread_detach(tid);

    // we need to know the depth
    acfrlcm_auv_acfr_nav_t_subscribe (state.gsd->lcm, "ACFR_NAV", &acfr_nav_callback, &state);
    acfrlcm_auv_relay_t_subscribe(state.gsd->lcm, "RELAY", &relay_callback, &state);
    
    // loop to collect data, parse and send it on its way
    while(!state.gsd->done) {
    	char buf[256];
        int64_t timestamp;
        gsd_read (state.gsd, buf, 256, &timestamp);
        
        senlcm_seabird_t seabird;
   		seabird.utime = timestamp;
        seabird.pump = state.pumpState;
        printf("%s\n", buf);
		if(parseSeabird(buf, &seabird)) {
			senlcm_seabird_t_publish (state.gsd->lcm, state.gsd->channel, &seabird);
            gsd_update_stats (state.gsd, 1);
        }
        else
            gsd_update_stats (state.gsd, -1);
	}
}

