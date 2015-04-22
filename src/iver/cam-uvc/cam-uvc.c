/* Interface between UVC and LCM for controlling the camera trigger.
 *	
 *	Christian Lees
 *	ACFR
 *	21/4/15
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <libgen.h>

#include <bot_param/param_client.h>

#include "acfr-common/nmea.h"
#include "acfr-common/sensor.h"
#include "perls-lcmtypes/acfrlcm_auv_camera_trigger_t.h"

typedef struct
{
	lcm_t *lcm;
	acfr_sensor_t *sensor;
	int enabled;
} state_t;


int parse_msg(state_t *state, char *buf) {
	// check to see what the message is
	if(!nmea_validate_checksum(buf))
        return 0;
        
    char outBuf[32];
    acfrlcm_auv_camera_trigger_t ct;
    
	ct.command = ACFRLCM_AUV_CAMERA_TRIGGER_T_SET_STATE;
    ct.freq = 1;
    ct.pulseWidthUs = 10;
    ct.strobeDelayUs = 0;
    ct.utime = timestamp_now();

    if(!strncmp(buf, "$VSTART", 6)) {
    	// start the camera
    	ct.enabled = 1;
		state.enabled = 1;
    	nmea_sprintf(outBuf, "$ACKVID,STA,1*");
   	    acfr_sensor_write(state->sensor, outBuf, strlen(outBuf));
   	    acfrlcm_auv_camera_trigger_t_publish(state->lcm, "CAMERA_TRIGGER", &ct);
    }

    if(!strncmp(buf, "$VSTOP", 6)) {
    	// start the camera
    	ct.enabled = 0;
		state.enabled = 0;
    	nmea_sprintf(outBuf, "$ACKVID,STP,1*");
   	    acfr_sensor_write(state->sensor, outBuf, strlen(outBuf));
   	    acfrlcm_auv_camera_trigger_t_publish(state->lcm, "CAMERA_TRIGGER", &ct);
    }
  	
  	return 1;
}

void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u) {
	// need to send a message once a second to UVC to tell it the camera is
	// still alive
	
	state_t *state = (state_t *)u;
	char outBuf[64];
	
	if(state->enabled)
		nmea_sprintf(outBuf, "$STATvid,4,1*");
	else
		nmea_sprintf(outBuf, "$STATvid,0,1*");
		
	acfr_sensor_write(state->sensor, outBuf, strlen(outBuf));
}


int program_exit;
int broken_pipe;
void
signal_handler(int sig_num)
{
   // do a safe exit
    if(sig_num == SIGPIPE)
        broken_pipe = 1;
    else
        program_exit = 1;
}

int main (int argc, char *argv[]) {
		
    // install the signal handler
    program_exit = 0;
    broken_pipe = 0;
    signal(SIGINT, signal_handler);
    //signal(SIGPIPE, signal_handler);
	
	//Initalise LCM object - specReading
	state_t state;
	state.enabled = 0;
    state.lcm = lcm_create(NULL);

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));
    
    acfr_sensor_t *sensor = acfr_sensor_create(state.lcm, rootkey);
    if(sensor == NULL)
        return 0;

	perllcm_heartbeat_t_subscribe(state.gsd->lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);

	char buf[128];

	while(!program_exit)
	{
		if(acfr_sensor_read_timeout(sensor, buf, sizeof(buf), 1) < 0)
			parse_msg(&state, buf);
	}

	return 0;
}
