/*	Change log

	27/7/10		added routines to anable the use of the Honeywell HMR3600
				compass.  Routines are honeywell_hmr3600_callback, os_compass_callback_no_send.
				Christian Lees

	29/7/10		added a new variable to the config file to select the RPH sensor type so
				you don't have to rebuild os-conduit to use a different sensor
				Christian Lees

	28/1/11		added variable to deiable the GPS from being sent through to UVC
				Christian Lees

*/


#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <bot_param/param_client.h>

#include "perls-common/nmea.h"
#include "perls-common/units.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-common/timestamp.h"

#include "perls-math/fasttrig.h"
#include "perls-math/gsl_util.h"
#include "perls-math/so3.h"
#include "perls-math/ssc.h"

#include "perls-lcmtypes/senlcm_raw_t.h"
#include "perls-lcmtypes/senlcm_raw_ascii_t.h"
#include "perls-lcmtypes/senlcm_os_altimeter_t.h"
#include "perls-lcmtypes/senlcm_os_compass_t.h"
#include "perls-lcmtypes/senlcm_ms_gx1_t.h"
#include "perls-lcmtypes/senlcm_dstar_ssp1_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd4_t.h"
#include "perls-lcmtypes/senlcm_honeywell_hmr3600_t.h"
#include "perls-lcmtypes/senlcm_ysi_t.h"
#include "perls-lcmtypes/acfrlcm_auv_os_conduit_t.h"
#include "perls-lcmtypes/acfrlcm_auv_os_conduit_raw_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/acfrlcm_auv_acfr_nav_t.h"

#define RTOD (UNITS_RADIAN_TO_DEGREE)
#define DTOR (UNITS_RADIAN_TO_DEGREE)

#define PUBLISH_OS_CONDUIT_RAW  1

#define USE_MICROSTRAIN_RPH     0
#define USE_DSTAR_DEPTH         0
#define USE_HMR3600_RPH		0

#define RDI_ATTITUDE_COR 0

// Globals.
int rphType = false;
bool useAcfrRPH;
bool sendGPS;
bool sendOSCompass;
bool sendHMR3600;
bool sendMicrostrain;

typedef struct _state_t state_t;
struct _state_t
{
    acfrlcm_auv_os_conduit_t os_conduit;
    senlcm_dstar_ssp1_t dstar;
    generic_sensor_driver_t *gsd;
    senlcm_honeywell_hmr3600_t hmr3600;
    senlcm_ms_gx1_t ustrain;
    senlcm_os_compass_t os_compass;		// compass structure needed when we are using the HMR3600 to get depth
    senlcm_ysi_t ysi;					// needed for when we use the ysi for the depth sensor
    bool useYsi;
    bool sendDvl;
    int dvlTimeout;
    bool sendOPOS;
};



static void
os_conduit_init (acfrlcm_auv_os_conduit_t *os_conduit)
{
    memset (os_conduit, 0, sizeof (*os_conduit));
    os_conduit->dvl.value = strdup ("");
    os_conduit->altimeter.value = strdup ("");
    os_conduit->gps.value = strdup ("");
    os_conduit->compass.value = strdup ("");
    os_conduit->command_passthru.value = strdup ("");
    os_conduit->vehicle_status_query.value = strdup ("");
}

static void
publish (state_t *state, acfrlcm_auv_os_conduit_raw_t *sensor,
         const int64_t utime, const char msg[])
{
    static char channel[LCM_MAX_CHANNEL_NAME_LENGTH] = { '\0' };
    if (!channel[0]) // first time
        sprintf (channel, "%s.OUT", state->gsd->channel);

#if PUBLISH_OS_CONDUIT_RAW

    // Timestamp for structure as a whole matches latest sensor data.
    state->os_conduit.utime = utime;

    // Update ages.  For *sensor this gets overwritten below.
    state->os_conduit.dvl.age = (utime - state->os_conduit.dvl.utime);
    state->os_conduit.altimeter.age = (utime - state->os_conduit.altimeter.utime);
    state->os_conduit.gps.age = (utime - state->os_conduit.gps.utime);
    state->os_conduit.compass.age = (utime - state->os_conduit.compass.utime);
    state->os_conduit.command_passthru.age = (utime - state->os_conduit.command_passthru.utime);
    state->os_conduit.vehicle_status_query.age = (utime - state->os_conduit.vehicle_status_query.utime);

    // Overwrite status of sensor that just got updated.
    sensor->age = (utime - sensor->utime) / 1000;
    sensor->utime = utime;
    free (sensor->value);
    sensor->value = strdup (msg);

    // Publish.
    acfrlcm_auv_os_conduit_t_publish (state->gsd->lcm, channel, &state->os_conduit);
#endif
}

/* static void */
/* os_compass_callback (const lcm_recv_buf_t *rbuf, const char *channel,  */
/*                      const senlcm_raw_t *raw, void *user) */
/* { */
/*     generic_sensor_driver_t *gsd = (generic_sensor_driver_t *) user; */

/*     if (raw->dtype != SENLCM_RAW_T_DTYPE_ASCII) { */
/*         ERROR ("remote-helm cannot accept binary data"); */
/*         return; */
/*     } */
/*     gsd_write (gsd, raw->buf, strlen (raw->buf)); */
/*     publish (gsd, &os_conduit.compass, raw->utime, raw->buf); */
/* } */

static void
microstrain_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                      const senlcm_ms_gx1_t *ustrain, void *user)
{
    state_t *state = (state_t *) user;

    char msg[NMEA_MAXIMUM_MSG_LENGTH];

    // only send fresh Euler data
    if (! (ustrain->bitmask & SENLCM_MS_GX1_T_STAB_EULER))
        return;

    // store attitude used for compensated altitude calc
    state->ustrain = *ustrain;

    nmea_sprintf (msg, "$C%.1lfP%.1lfR%.1lfT%.1lfD%.2lf*",
                  ustrain->sEuler[2] * RTOD + 180, // deg
                  ustrain->sEuler[1] * RTOD * -1, // deg
                  ustrain->sEuler[0] * RTOD * -1, // deg
                  ustrain->Temperature,   // celsius
                  state->dstar.depth * UNITS_METER_TO_FEET); // feet

    if (sendMicrostrain)
    {
        gsd_write (state->gsd, msg, strlen (msg));
        publish (state, &state->os_conduit.compass, ustrain->utime, msg);
    }
}

static void
os_compass_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                     const senlcm_os_compass_t *os_compass, void *user)
{
    state_t *state = (state_t *) user;

    char msg[NMEA_MAXIMUM_MSG_LENGTH];

    double dfs_feet;
    if (USE_DSTAR_DEPTH)
        dfs_feet = state->dstar.depth * UNITS_METER_TO_FEET;
    else if(state->useYsi)
        dfs_feet = state->ysi.depth * UNITS_METER_TO_FEET;
    else
        dfs_feet = os_compass->depth * UNITS_METER_TO_FEET;

    // store attitude used for compensated altitude calc
    state->os_compass = *os_compass;

    // Generate NMEA string with checksum to spoof compass
    // this gets passed to UVC pretending to be the compass...
    nmea_sprintf (msg, "$C%.1lfP%.1lfR%.1lfT%.1lfD%.2lf*",
                  os_compass->rph[2] * RTOD, // deg
                  os_compass->rph[1] * RTOD, // deg
                  os_compass->rph[0] * RTOD, // deg
                  os_compass->T,             // celsius
                  dfs_feet);                 // feet

    if ( sendOSCompass )
    {
        gsd_write (state->gsd, msg, strlen (msg));
        publish (state, &state->os_conduit.compass, os_compass->utime, msg);
    }
}


// dummy os_compass callback that just makes a copy of the incomming data
// this is required when you use a RPH sensor other then the OS one.
// The OS compass is also responsible for the depth measurement
static void
os_compass_callback_no_send (const lcm_recv_buf_t *rbuf, const char *channel,
                             const senlcm_os_compass_t *os_compass, void *user)
{
    // just store the data structure so we can extract the depth
    state_t *state = (state_t *) user;
    memcpy(&state->os_compass, os_compass, sizeof(*os_compass));
}

static void
honeywell_hmr3600_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                            const senlcm_honeywell_hmr3600_t *hmr3600, void *user)
{
    state_t *state = (state_t *) user;
    char msg[NMEA_MAXIMUM_MSG_LENGTH];

    double dfs_feet;
    if (USE_DSTAR_DEPTH)
        dfs_feet = state->dstar.depth * UNITS_METER_TO_FEET;
    else if(state->useYsi)
        dfs_feet = state->ysi.depth * UNITS_METER_TO_FEET;
    else
        dfs_feet = state->os_compass.depth * UNITS_METER_TO_FEET;

    // store attitude used for compensated altitude calc
    state->hmr3600 = *hmr3600;

    // Generate NMEA string with checksum to spoof compass
    // this gets passed to UVC pretending to be the compass...
    nmea_sprintf (msg, "$C%.1lfP%.1lfR%.1lfT%.1lfD%.2lf*",
                  hmr3600->rph[2] * RTOD, // deg
                  hmr3600->rph[1] * RTOD, // deg
                  hmr3600->rph[0] * RTOD, // deg
                  state->os_compass.T,             // celsius
                  dfs_feet);                 // feet

    if ( sendHMR3600 )
    {
        gsd_write (state->gsd, msg, strlen (msg));
        publish (state, &state->os_conduit.compass, hmr3600->utime, msg);
    }
}

static void
os_altimeter_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                       const senlcm_os_altimeter_t *alt, void *user)
{
    state_t *state = (state_t *) user;

    char msg[NMEA_MAXIMUM_MSG_LENGTH];

    if ( ! isnan(alt->offset) )
    {
        nmea_sprintf (msg, "$SDDPT,%.lf,*", alt->offset);
    }
    else
    {
        nmea_sprintf (msg, "$SDDPT,,*");
    }
    gsd_write (state->gsd, msg, strlen(msg));

    if ( ! isnan(alt->metres) )
    {
        // Compensate altimeter for pitch angle.
        // ACFRnav should be an option for pitch source as well.
        // See notes in iverACFR.cfg
        double pitch = NAN;
        switch ( rphType )
        {
        case 0: // OS-COMPASS
        {
            pitch = state->os_compass.rph[1];
            break;
        }
        case 1: // HMR3600
        {
            pitch = state->hmr3600.rph[1];
            break;
        }
        case 2: // MS-GX1
        {
            pitch = state->ustrain.sEuler[1] * -1.0;
            break;
        }
        }
        if ( ! isnan (pitch) )
        {
            senlcm_os_altimeter_t alt_comp;
            alt_comp.feet = alt->feet*cos(pitch);
            alt_comp.metres = alt->metres*cos(pitch);
            alt_comp.fathoms = alt->fathoms*cos(pitch);
            nmea_sprintf (msg, "$SDDBT,%.2lf,f,%.2lf,M,%.2lf,F*",
                          alt_comp.feet, alt_comp.metres,
                          alt_comp.fathoms);
        }
        else
        {
            nmea_sprintf (msg, "$SDDBT,%.2lf,f,%.2lf,M,%.2lf,F*", alt->feet, alt->metres, alt->fathoms);
        }
    }
    else
    {
        nmea_sprintf (msg, "$SDDBT,,f,,M,,F*");
    }
    gsd_write (state->gsd, msg, strlen(msg));

    if ( ! isnan(alt->temperature) )
    {
        nmea_sprintf (msg, "$YXMTW,%.1lf,C*", alt->temperature);
    }
    else
    {
        nmea_sprintf (msg, "$YXMTW,,C*");
    }
}


static void
desert_star_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                      const senlcm_dstar_ssp1_t *my_dstar, void *user)
{
    state_t *state = (state_t *) user;
    memcpy (&state->dstar, my_dstar, sizeof (*my_dstar));
}

static void
gpsd_client_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                      const senlcm_raw_t *raw, void *user)
{
    state_t *state = (state_t *) user;

    if ( sendGPS )
    {
        gsd_write (state->gsd, (char *) raw->data, raw->length);
        publish (state, &state->os_conduit.gps, raw->utime, (char *) raw->data);
    }
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
    publish (state, &state->os_conduit.vehicle_status_query, heartbeat->utime, msg);

}

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
    if (nmea_payload(src, msg) > 0)
    {

        //Append NMEA checksum.
        nmea_sprintf (msg, "%s", src);

        gsd_write (state->gsd, msg, strlen (msg));
        publish (state, &state->os_conduit.command_passthru, raw->utime, msg);
    }

}

#define DEPTH_MOVE (+0.0)
static void
acfr_nav_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                   const acfrlcm_auv_acfr_nav_t *nav, void *user)
{

    state_t *state = (state_t *) user;
    char msg[NMEA_MAXIMUM_MSG_LENGTH];
    double hdg_360;
    double hdg,pitch,roll;

    // Calucluate the depth at a point on the vehicle that is closer to the back
    // so the vehicle had better depth control
    double newDepth = nav->depth - DEPTH_MOVE * sin(nav->pitch);

    // Choose attitude source.
    if ( useAcfrRPH )
    {
        hdg = nav->heading;
        roll = nav->roll;
        pitch = nav->pitch;
    }
    else
    {
        switch ( rphType )
        {
        case 0: // OS-COMPASS
        {
            hdg = state->os_compass.rph[2];
            pitch = state->os_compass.rph[1];
            roll = state->os_compass.rph[0];
            break;
        }
        case 1: // HMR3600
        {
            hdg = state->hmr3600.rph[2];
            pitch = state->hmr3600.rph[1];
            roll = state->hmr3600.rph[0];
            break;
        }
        case 2: // MS-GX1
        {
            hdg = state->ustrain.sEuler[2] + M_PI/2.0;
            pitch = state->ustrain.sEuler[1] * -1.0;
            roll = state->ustrain.sEuler[0] * -1.0;
            break;
        }
        }
    }

    // Compute heading \in [0,360)
    hdg_360 = fmod(nav->heading,M_PI*2.0) * RTOD;
    if ( hdg_360 < 0.0 )
    {
        hdg_360 = hdg_360 + 360.0;
    }

    // Generate NMEA string with checksum to spoof compass
    // this gets passed to UVC pretending to be the compass...
    nmea_sprintf (msg, "$C%.1lfP%.1lfR%.1lfT%.1lfD%.2lf*",
                  hdg_360, // deg
                  pitch * RTOD, // deg
                  roll * RTOD, // deg
                  state->os_compass.T,             // celsius
                  newDepth * UNITS_METER_TO_FEET);                 // feet

    gsd_write (state->gsd, msg, strlen (msg));

    if(state->sendDvl)
    {
        nmea_sprintf(msg, "$ODVL,%3.3f,%3.3f,%d*", nav->vx, nav->vy, state->dvlTimeout);
        gsd_write (state->gsd, msg, strlen (msg));
    }

    if(state->sendOPOS)
    {
        nmea_sprintf(msg, "$OPOS,%3.7f,%3.7f,%3.7f,0.0,0.0*",nav->latitude, nav->longitude, nav->vx / 0.51444);
        gsd_write (state->gsd, msg, strlen (msg));
    }

}

static void
ysi_callback (const lcm_recv_buf_t *rbuf, const char *channel,
              const senlcm_ysi_t *ysi, void *user)
{

    // just make a copy of the data to use elsewhere
    state_t *state = (state_t *) user;
    memcpy(&state->ysi, ysi, sizeof(senlcm_ysi_t));
}


// Process LCM messages with callbacks
static void *
lcm_thread (void *context)
{
    generic_sensor_driver_t *gsd = (generic_sensor_driver_t *) context;
    while (!gsd->done)
    {
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
main (int argc, char *argv[])
{
    state_t *state = calloc (1, sizeof (state_t));
    os_conduit_init (&state->os_conduit);

    fasttrig_init ();

    state->gsd = gsd_create (argc, argv, "remote-helm.os-conduit-acfr", &myopts);
    gsd_canonical (state->gsd, '\r','\n');
    gsd_launch (state->gsd);

    senlcm_raw_ascii_t_subscribe (state->gsd->lcm, "OS_COMMAND_PASSTHRU", &os_command_passthru_callback, state);
    perllcm_heartbeat_t_subscribe (state->gsd->lcm, "HEARTBEAT_1HZ", &os_vehicle_status_callback, state);
    senlcm_os_altimeter_t_subscribe (state->gsd->lcm, "OS_ALTIMETER", &os_altimeter_callback, state);
    senlcm_dstar_ssp1_t_subscribe (state->gsd->lcm, "DESERT_STAR", &desert_star_callback, state);
    senlcm_honeywell_hmr3600_t_subscribe (state->gsd->lcm, "HONEYWELL_HMR3600", &honeywell_hmr3600_callback, state);
    senlcm_os_compass_t_subscribe (state->gsd->lcm, "OS_COMPASS", &os_compass_callback_no_send, state);
    senlcm_ms_gx1_t_subscribe (state->gsd->lcm, "MICROSTRAIN", &microstrain_callback, state);
    acfrlcm_auv_acfr_nav_t_subscribe (state->gsd->lcm, "ACFR_NAV", &acfr_nav_callback, state);
    senlcm_raw_t_subscribe (state->gsd->lcm, "GPSD_CLIENT.RAW", &gpsd_client_callback, state);
    senlcm_ysi_t_subscribe (state->gsd->lcm, "YSI", &ysi_callback, state);

    char key[128];
    sprintf(key, "%s.useAcfrRPH", state->gsd->rootkey);
    if(bot_param_get_boolean(state->gsd->cfg, key, &useAcfrRPH) == -1)
        useAcfrRPH = false;

    char *rphTypeStr;
    sprintf(key, "%s.rph", state->gsd->rootkey);
    if(bot_param_get_str(state->gsd->cfg, key, &rphTypeStr) == -1)
        rphType = 0;
    else
    {
        if(!strcmp(rphTypeStr, "os"))
            rphType = 0;
        if(!strcmp(rphTypeStr, "hmr3600"))
            rphType = 1;
        if(!strcmp(rphTypeStr, "ms-gx1"))
            rphType = 2;
    }



    // find out if we are going to ignore the gps
    sprintf(key, "%s.sendGPS", state->gsd->rootkey);
    sendGPS = bot_param_get_boolean_or_fail(state->gsd->cfg, key);

    // Determine whether to send OS-Compass through.
    sprintf(key, "%s.sendOSCompass", state->gsd->rootkey);
    sendOSCompass = bot_param_get_boolean_or_fail(state->gsd->cfg, key);

    // Determine whether to send HMR3600 through.
    sprintf(key, "%s.sendHMR3600", state->gsd->rootkey);
    sendHMR3600 = bot_param_get_boolean_or_fail(state->gsd->cfg, key);

    // Determine whether to send Microstrain through.
    sprintf(key, "%s.sendMicrostrain", state->gsd->rootkey);
    sendMicrostrain = bot_param_get_boolean_or_fail(state->gsd->cfg, key);

    // Determine whether to send YSI compass through.
    sprintf(key, "%s.useYsi", state->gsd->rootkey);
    state->useYsi = bot_param_get_boolean_or_fail(state->gsd->cfg, key);

    // Determine whether to send DVL data to UVC.
    sprintf(key, "%s.sendDvl", state->gsd->rootkey);
    state->sendDvl = bot_param_get_boolean_or_fail(state->gsd->cfg, key);

    // Determine whether to send DVL data to UVC.
    if(state->sendDvl)
    {
        sprintf(key, "%s.dvlTimeout", state->gsd->rootkey);
        state->dvlTimeout = bot_param_get_int_or_fail(state->gsd->cfg, key);
    }

    // Determine whether to send DVL data to UVC.
    sprintf(key, "%s.sendOPOS", state->gsd->rootkey);
    state->sendOPOS = bot_param_get_boolean_or_fail(state->gsd->cfg, key);


    pthread_t tid;
    pthread_create (&tid, NULL, lcm_thread, state->gsd);
    pthread_detach (tid);

    gsd_flush (state->gsd);
    gsd_reset_stats (state->gsd);
    while (!state->gsd->done)
    {
        // read stream
        char buf[1024];
        int64_t timestamp;
        int len = gsd_read (state->gsd, buf, 128, &timestamp);
        if (len > 0)
            gsd_update_stats (state->gsd, 1);
        printf(".");
    } // while
}
