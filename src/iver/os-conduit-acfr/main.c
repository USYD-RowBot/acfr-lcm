#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "perls-lcmtypes/acfrlcm_auv_acfr_nav_t.h"
#include "perls-lcmtypes/senlcm_raw_ascii_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-common/lcm_util.h"
#include "perls-common/nmea.h"
#include "perls-common/timestamp.h"
#include "perls-common/units.h"

#define DEPTH_MOVE 0.0
#define NMEA_MAXIMUM_MSG_LENGTH 256
#define RTOD UNITS_RADIAN_TO_DEGREE

typedef struct
{
    generic_sensor_driver_t *gsd;
    int dvl_timeout;
} state_t;


// OS command passthru.  Commands published on the OS_COMMAND_PASSTHRU channel have an NMEA checksum 
// appended if they do not already and are then passed to the main CPU.  This is intended 
// to facilitate backseat mission control.
static void
os_command_passthru_callback (const lcm_recv_buf_t *rbuf, const char *channel, 
                     const senlcm_raw_ascii_t *raw, void *user)
{
    state_t *state = (state_t *) user;
    char msg[NMEA_MAXIMUM_MSG_LENGTH];
    char *src = raw->msg;

    // Check for valid NMEA payload
    if (nmea_payload(src, msg) > 0) {

      //Append NMEA checksum.
      nmea_sprintf (msg, "%s", src); 
      
      gsd_write (state->gsd, msg, strlen (msg));
//      publish (state, &state->os_conduit.command_passthru, raw->utime, msg);
    }
}


static void
acfr_nav_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                     const acfrlcm_auv_acfr_nav_t *nav, void *user) {

    state_t *state = (state_t *) user;
    char msg[NMEA_MAXIMUM_MSG_LENGTH];
    double hdg_360;
    double hdg,pitch,roll;

    // Calucluate the depth at a point on the vehicle that is closer to the back
    // so the vehicle had better depth control
    double newDepth = nav->depth - DEPTH_MOVE * sin(nav->pitch);

    hdg = nav->heading;
    roll = nav->roll;
    pitch = nav->pitch;

    // Compute heading \in [0,360)
    hdg_360 = fmod(nav->heading,M_PI*2.0) * RTOD;
    if(hdg_360 < 0.0)
    	hdg_360 = hdg_360 + 360.0;

    // Generate NMEA string with checksum to spoof compass
    // this gets passed to UVC pretending to be the compass...
    nmea_sprintf (msg, "$C%.1lfP%.1lfR%.1lfT%.1lfD%.2lf*",
                  hdg_360, // deg
                  pitch * RTOD, // deg
                  roll * RTOD, // deg
                  0.0,             // celsius
                  newDepth * UNITS_METER_TO_FEET);                 // feet
    
    gsd_write (state->gsd, msg, strlen (msg));
    
    // DVL
	nmea_sprintf(msg, "$ODVL,%3.3f,%3.3f,%d*", nav->vx, nav->vy, state->dvl_timeout);
	gsd_write (state->gsd, msg, strlen (msg));

    // 2D position
	nmea_sprintf(msg, "$OPOS,%3.7f,%3.7f,%3.7f*",nav->latitude, nav->longitude, nav->vx / 0.51444);
    gsd_write (state->gsd, msg, strlen (msg));
    
    // Altitude
    double alt_feet = nav->altitude * UNITS_METER_TO_FEET * cos(pitch);
    double alt_metres = nav->altitude * cos(pitch);
    double alt_fathoms = nav->altitude * UNITS_METER_TO_FATHOM * cos(pitch);
    nmea_sprintf (msg, "$SDDBT,%.2lf,f,%.2lf,M,%.2lf,F*", 
			  alt_feet, alt_metres, 
			  alt_fathoms);
    
}

// Vehicle status query upon receipt of heartbeat.  A separate process is responsible for
// processing the reply (by subscribing to the OS_CONDUIT.RAW channel).  It would be more 
// sensible to create another process alltogether than handles the status query.  Here there should 
// just be something to forward a particular channel to the main CPU.
static void
os_vehicle_status_callback (const lcm_recv_buf_t *rbuf, const char *channel, 
                     const perllcm_heartbeat_t *heartbeat, void *user)
{
    state_t *state = (state_t *) user;
    char msg[NMEA_MAXIMUM_MSG_LENGTH];

    // Compose NMEA vehicle status query.
    nmea_sprintf (msg, "$OSD,G,C,S,P,Y,*");

    // Send query to main CPU.
    gsd_write (state->gsd, msg, strlen (msg));
    //publish (state, &state->os_conduit.vehicle_status_query, heartbeat->utime, msg);

}

// Process LCM messages with callbacks
static void *
lcm_thread (void *context)
{
    generic_sensor_driver_t *gsd = (generic_sensor_driver_t *) context;
    while (!gsd->done) {
        lcm_handle (gsd->lcm);
        gsd_update_stats (gsd, 1);
	printf("-");
    }
    
    return 0;
}


static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "Ocean-Server Remote Helm I/O driver.");
    return 0;
}

int 
main(int argc, char **argv)
{
    // generic sensor driver
    state_t state;
    state.gsd = gsd_create (argc, argv, "os-conduit-acfr", &myopts);
    gsd_canonical (state.gsd, '\r','\n');
    gsd_launch (state.gsd);
    
    char key[128];
    sprintf(key, "%s.dvl_timeout", state.gsd->rootkey);
	state.dvl_timeout = bot_param_get_int_or_fail(state.gsd->params, key);
    
    
    perllcm_heartbeat_t_subscribe (state.gsd->lcm, "HEARTBEAT_1HZ", &os_vehicle_status_callback, &state);
    acfrlcm_auv_acfr_nav_t_subscribe (state.gsd->lcm, "ACFR_NAV", &acfr_nav_callback, &state);
    senlcm_raw_ascii_t_subscribe (state.gsd->lcm, "OS_COMMAND_PASSTHRU", &os_command_passthru_callback, &state);
        
    pthread_t tid;
    pthread_create (&tid, NULL, lcm_thread, state.gsd);
    pthread_detach (tid);

    gsd_flush (state.gsd);
    gsd_reset_stats (state.gsd);
    while(!state.gsd->done) 
    {
        // read stream
        char buf[1024];
        int64_t timestamp;
        int len = gsd_read (state.gsd, buf, 128, &timestamp);
        if (len > 0) 
            gsd_update_stats (state.gsd, 1);
	
    } // while
    
    return 0;
}
