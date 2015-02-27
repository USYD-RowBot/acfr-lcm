/*  	Interface between UVC and LCM for controlling the camera trigger.
	
	Christian Lees
	ACFR
	20/4/11
*/

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <pthread.h>
#include "perls-common/nmea.h"
#include "perls-common/timestamp.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-lcmtypes/acfrlcm_auv_camera_trigger_t.h"
#include "perls-lcmtypes/acfrlcm_auv_camera_trigger_out_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

// command messages
#define SET_FREQ				1
#define SET_DELAY				2
#define	SET_WIDTH				3
#define SET_STATE				4
#define SET_ALL					5


typedef struct {
	generic_sensor_driver_t *gsd;
	int enabled;
} state_t;

int parseMsg(state_t *state, char *buf) {
	// check to see what the message is
	if(!nmea_validate_checksum(buf))
        return 0;
        
    char outBuf[32];
    acfrlcm_auv_camera_trigger_t ct;
    
//	memset(&ct, 0, sizeof(acfrlcm_auv_camera_trigger_t));
	ct.command = SET_STATE;
    ct.freq = 1;
    ct.pulseWidthUs = 10;
    ct.strobeDelayUs = 0;
    ct.utime = timestamp_now();

    if(!strncmp(buf, "$VSTART", 6)) {
    	// start the camera
    	state->enabled = 1;
    	ct.enabled = 1;
    	nmea_sprintf(outBuf, "$ACKVID,STA,1*");
   	    gsd_write(state->gsd, outBuf, strlen(outBuf));
   	    acfrlcm_auv_camera_trigger_t_publish(state->gsd->lcm, "CAMERA_TRIGGER", &ct);

    }
    if(!strncmp(buf, "$VSTOP", 6)) {
    	// start the camera
    	state->enabled = 0;
    	ct.enabled = 0;
    	nmea_sprintf(outBuf, "$ACKVID,STP,1*");
   	    gsd_write(state->gsd, outBuf, strlen(outBuf));
   	    acfrlcm_auv_camera_trigger_t_publish(state->gsd->lcm, "CAMERA_TRIGGER", &ct);
    }

    

  	
  	return 1;
}


void heartBeatHandler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u) {
	// need to send a message once a second to UVC to tell it the camera is
	// still alive
	
	state_t *state = (state_t *)u;
	char outBuf[64];
	
	if(state->enabled)
		nmea_sprintf(outBuf, "$STATvid,4,1*");
	else
		nmea_sprintf(outBuf, "$STATvid,0,1*");
		
	gsd_write(state->gsd, outBuf, strlen(outBuf));
	

	
}
 
static void *lcmThread (void *context) {
    generic_sensor_driver_t *gsd = (generic_sensor_driver_t *)context;
    while (!gsd->done) {
        lcm_handle(gsd->lcm);
    }
    
    return 0;
}

    
static int
myopts (generic_sensor_driver_t *gsd) {
    getopt_add_description (gsd->gopt, "UVC to LCM camera control.");
    return 0;
}

int main(int argc, char **argv) {

	state_t state;
	state.enabled = 0;
	
	state.gsd = gsd_create (argc, argv, "acfr.cam-uvc", &myopts);
    gsd_canonical (state.gsd, '\r','\n');
    gsd_launch (state.gsd);
	
	perllcm_heartbeat_t_subscribe(state.gsd->lcm, "HEARTBEAT_1HZ", &heartBeatHandler, &state);
	
	gsd_flush(state.gsd);
    gsd_reset_stats(state.gsd);
    
    pthread_t tid;
    pthread_create (&tid, NULL, lcmThread, state.gsd);
    pthread_detach (tid);
    
    while (!state.gsd->done) {
        // read stream
        char buf[1024];
        int64_t timestamp;
        int len = gsd_read(state.gsd, buf, 128, &timestamp);
        if (len > 0) {
        	parseMsg(&state, buf);
            gsd_update_stats (state.gsd, 1);
        }
    } // while
}
