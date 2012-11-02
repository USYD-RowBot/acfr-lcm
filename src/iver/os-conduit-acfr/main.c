#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "perls-lcmtypes/acfrlcm_auv_acfr_nav_t.h"
#include "perls-lcmtypes/senlcm_raw_ascii_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/senlcm_os_power_system_t.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-lcmtypes/senlcm_uvc_ack_t.h"
#include "perls-lcmtypes/senlcm_uvc_opi_t.h"
#include "perls-lcmtypes/senlcm_uvc_osi_t.h"
#include "perls-lcmtypes/senlcm_uvc_rphtd_t.h"

#include "perls-common/lcm_util.h"
#include "perls-common/nmea.h"
#include "perls-common/timestamp.h"
#include "perls-common/units.h"

#define DEPTH_MOVE 0.0
#define NMEA_MAXIMUM_MSG_LENGTH 256
#define DTOR (UNITS_DEGREE_TO_RADIAN)
#define RTOD (UNITS_RADIAN_TO_DEGREE)
#define ERROR printf

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


// send a battery string to UVC
static void
battery_callback(const lcm_recv_buf_t *rbuf, const char *channel, 
                     const senlcm_os_power_system_t *batt, void *user)
{
    state_t *state = (state_t *)user;
    char msg[NMEA_MAXIMUM_MSG_LENGTH];
    int leak = 1;
    double voltage = 0.0;
    double capacity = 0.0;
    int batt_count = 0;
    char mode;
    
    // find out the average voltage
    for(int i=0; i<batt->num_controllers; i++)
        for(int j=0; j<batt->controller[i].num_batteries; j++)
        {
            voltage += batt->controller[i].battery[j].voltage;
            capacity += batt->controller[i].battery[j].remaining_capacity;
            batt_count++;
        }
    voltage = voltage / batt_count;
    
    if(batt->controller[0].battery_state[0] == 'D')
        mode = 'D';
    else
        mode = 'C';
    
    // Compose NMEA battery string
    nmea_sprintf (msg, "$OCEANA,%d,%d,%d,%3.1f,%3.1f,%5.1f,%c,%d*", batt->avg_charge_p, (int)capacity*1000, abs((int)batt->power), voltage, batt->current, (double)batt->minutes_tef, mode, leak);
    gsd_write (state->gsd, msg, strlen (msg));
    
    printf("%s\n", msg);
}



/* returns 1 for successfully classifying and publishing uvc to
 * backseat driver messages */
static int
publish_from_uvc (state_t *state, const char *msg, const int64_t timestamp)
{
    // $OSI -- vehicle state data
    if (0==strncmp (msg, "$OSI", 4)) {
        char hex[128] = {'\0'};
        if (!nmea_arg (msg, 1, hex))
            ERROR ("unable to extract OSI hex");

        int yt=-1, yb=-1, pl=-1, pr=-1, mot=-1;
        if (5!=sscanf (hex, "%02x%02x%02x%02x%02x", &yt, &yb, &pl, &pr, &mot))
            ERROR ("unable to parse OSI yt, yb, pl, pr, mot");

        char mode_str[128];
        if (!nmea_arg (msg, 2, mode_str))
            ERROR ("unable to parse OSI mode_str");

        int mode = -1;
        if (0==strncmp (mode_str, "N", 1))
            mode = SENLCM_UVC_OSI_T_MODE_NORMAL;
        else if (0==strncmp (mode_str, "S", 1))
            mode = SENLCM_UVC_OSI_T_MODE_STOPPED;
        else if (0==strncmp (mode_str, "P", 1))
            mode = SENLCM_UVC_OSI_T_MODE_PARK;
        else if (0==strncmp (mode_str, "Mp", 2))
            mode = SENLCM_UVC_OSI_T_MODE_MANUAL_PARK;
        else if (0==strncmp (mode_str, "M", 1))
            mode = SENLCM_UVC_OSI_T_MODE_MANUAL_OVERRIDE;
        else if (0==strncmp (mode_str, "A", 1))
            mode = SENLCM_UVC_OSI_T_MODE_SERVO;
        else if (0==strncmp (mode_str, "W", 1))
            mode = SENLCM_UVC_OSI_T_MODE_MISSION;
        else
            ERROR ("unable to parse OSI mode");

        int nextwp = 0;
        if (!nmea_argi (msg, 3, &nextwp))
            ERROR ("unable to parse OSI nextwp");

        double latitude = 0;
        if (!nmea_argf (msg, 4, &latitude))
            ERROR ("unable to parse OSI latitude");

        double longitude = 0;
        if (!nmea_argf (msg, 5, &longitude))
            ERROR ("unable to parse OSI longitude");

        double speed = 0;
        if (!nmea_argf (msg, 6, &speed))
            ERROR ("unable to parse OSI speed");

        double dist_to_nextwp = 0;
        if (!nmea_argf (msg, 7, &dist_to_nextwp))
            ERROR ("unable to parse OSI dist_to_nextwp");

        char error[128] = {'\0'};
        if (!nmea_arg (msg, 8, error))
            ERROR ("unable to parse OSI error");

        int err_state = -1;
        if (0==strncmp (error, "N", 1))
            err_state = SENLCM_UVC_OSI_T_ERR_NONE;
        else if (0==strncmp (error, "Sop", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_OVER_PITCH;
        else if (0==strncmp (error, "Stl", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_EXCEED_TIME;
        else if (0==strncmp (error, "Sle", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_LEAK;
        else if (0==strncmp (error, "Sfp", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_NO_FORWARD_PROG;
        else if (0==strncmp (error, "Sed", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_EXCEED_MAX_DEPTH;
        else if (0==strncmp (error, "Sup", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_NO_UPWARD_PROG;
        else if (0==strncmp (error, "Stf", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_TOW_FLOAT_ENGAGED;
        else if (0==strncmp (error, "Srp", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_SAFETY_RET_PATH;
        else if (0==strncmp (error, "Snd", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_DFS_UNCHANGED;
        else if (0==strncmp (error, "Snc", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_COMPASS_STOPPED;
        else if (0==strncmp (error, "Spw", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_EXCEEDED_MIN_REQ_PWR;
        else if (0==strncmp (error, "Sti", 3))
            err_state = SENLCM_UVC_OSI_T_ERR_STOP_AND_TRANSMIT_IRIDIUM;
        else
            ERROR ("unable to parse OSI err_state");

        double altimeter = 0;
        if (!nmea_argf (msg, 9, &altimeter))
            ERROR ("unable to parse OSI altimeter");

        char park[128]; double park_time = 0;
        if (!nmea_arg (msg, 10, park))
            ERROR ("unable to parse OSI park");
        else
            park_time = atoi (park+1);

        double magnetic_dec = 0;
        if (!nmea_argf (msg, 11, &magnetic_dec))
            ERROR ("unable to parse OSI magnetic_dec");

        senlcm_uvc_osi_t data = {
            .utime          = timestamp,
            .yaw_top        = yt,
            .yaw_bot        = yb,
            .pitch_left     = pl,
            .pitch_right    = pr,
            .motor          = mot, 
            .mode           = mode,
            .nextwp         = nextwp + 1,  // to match match vector map indexing
            .latitude       = DTOR * latitude,
            .longitude      = DTOR * longitude,
            .speed          = UNITS_KNOT_TO_METER_PER_SEC * speed,
            .dist_to_nextwp = dist_to_nextwp,
            .error          = err_state,
            .altimeter      = UNITS_FEET_TO_METER * altimeter,
            .park_time      = park_time,
            .magnetic_dec   = magnetic_dec,
        };
        senlcm_uvc_osi_t_publish (state->gsd->lcm, "UVC_OSI", &data); 
        return 1;
    }

    // $OPI -- vehicle power data
    else if (0==strncmp (msg,"$OPI",4)) {
        double percent = 0;
        if (!nmea_argf (msg, 1, &percent))
            ERROR ("unable to parse OPI percent");

        double remaining_cap = 0;
        if (!nmea_argf (msg, 2, &remaining_cap))
            ERROR ("unable to parse OPI remaining_cap");

        double pwr = 0;
        if (!nmea_argf (msg, 3, &pwr))
            ERROR ("unable to parse OPI pwr");

        double volts = 0;
        if (!nmea_argf (msg, 4, &volts))
            ERROR ("unable to parse OPI volts");

        double current = 0;
        if (!nmea_argf (msg, 5, &current))
            ERROR ("unable to parse OPI current");

        double time_til = 0;
        if (!nmea_argf (msg, 6, &time_til))
            ERROR ("unable to parse OPI time_til");

        char bs = '\0';
        if (!nmea_argc (msg, 7, &bs))
            ERROR ("unable to parse OPI bs");

        int batt_state = 0;
        switch (bs) {
            case 'C':
                batt_state = SENLCM_UVC_OPI_T_BS_CHARGING;
                break;
            case 'D':
                batt_state = SENLCM_UVC_OPI_T_BS_DISCHARGING;
                break;
            case 'F':
                batt_state = SENLCM_UVC_OPI_T_BS_FAULT;
                break;
            default:
                ERROR ("unable to parse OPI batt_state");
        }

        int leak = 0;
        if (!nmea_argi (msg, 8, &leak))
            ERROR ("unable to parse OPI leak");

        senlcm_uvc_opi_t data = {
            .utime         = timestamp,
            .percent       = percent,
            .remaining_cap = remaining_cap,
            .pwr           = pwr,
            .volts         = volts,
            .current       = current,
            .time_til      = time_til,	    
            .batt_state    = batt_state,
            .leak          = leak,
        };
        senlcm_uvc_opi_t_publish (state->gsd->lcm, "UVC_OPI", &data);
        return 1;
    }  

    // $C#P#R#T#D# -- uvc to host compass string
    else if (0==strncmp (msg, "$C", 2)) {
        double heading, pitch, roll, temp, depth = 0;
        if (5!=sscanf (msg, "$C%lfP%lfR%lfT%lfD%lf", &heading, &pitch, &roll, &temp, &depth)) {
            ERROR ("unable to parse UVC Compass string!");
        }
        
        senlcm_uvc_rphtd_t data = {
            .utime         = timestamp,
            .rph           = {roll*DTOR, pitch*DTOR, heading*DTOR},
            .T             = temp,
            .depth         = depth * UNITS_FEET_TO_METER,
        };
        senlcm_uvc_rphtd_t_publish (state->gsd->lcm, "UVC_RPH", &data);
        return 1;
    }

    // $ACK -- uvc ack of backseat driver cmd
    else if (0==strncmp (msg,"$ACK",4)) {
        int msg_type = 0;
        if (!nmea_argi (msg, 1, &msg_type))
            ERROR ("unable to parse ACK msg_type");

        int status = 0;
        if (!nmea_argi (msg, 2, &status))
            ERROR ("unable to parse ACK status");

        int err_num = 0;
        if (!nmea_argi (msg, 3, &err_num))
            ERROR ("unable to parse ACK err_num");

        char usrset[128] = {'\0'};
        int usrnum = 0;
        char usrval[128] = {'\0'};
        if (msg_type == SENLCM_UVC_ACK_T_ACK_ORWSET) {
            if (!nmea_arg (msg, 4, usrset))
                ERROR ("unable to parse ACK usrset");

            if (!nmea_argi (msg, 5, &usrnum))
                ERROR ("unable to parse ACK usrnum");

            if (!nmea_arg (msg,6,usrval))
                ERROR ("unable to parse ACK usrval");
        }

        senlcm_uvc_ack_t data = {
            .utime    = timestamp,
            .msg_type = msg_type,
            .status   = status,
            .err_num  = err_num,
            .usrset   = usrset,
            .usrnum   = usrnum,
            .usrval   = usrval,
        };
        senlcm_uvc_ack_t_publish (state->gsd->lcm, "UVC_ACK", &data); 
        return 1;
    }

    return 0;
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
    state.gsd = gsd_create (argc, argv, "remote-helm.os-conduit-acfr", &myopts);
    gsd_canonical (state.gsd, '\r','\n');
    gsd_launch (state.gsd);
    
    char key[128];
    sprintf(key, "%s.dvl_timeout", state.gsd->rootkey);
	state.dvl_timeout = bot_param_get_int_or_fail(state.gsd->params, key);
    
    
    perllcm_heartbeat_t_subscribe (state.gsd->lcm, "HEARTBEAT_1HZ", &os_vehicle_status_callback, &state);
    acfrlcm_auv_acfr_nav_t_subscribe (state.gsd->lcm, "ACFR_NAV", &acfr_nav_callback, &state);
    senlcm_raw_ascii_t_subscribe (state.gsd->lcm, "OS_COMMAND_PASSTHRU", &os_command_passthru_callback, &state);
    senlcm_os_power_system_t_subscribe (state.gsd->lcm, "BATTERY", &battery_callback, &state);
        
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
        if (publish_from_uvc (&state, buf, timestamp)) 
            gsd_update_stats (state.gsd, 1);
        else 
            gsd_update_stats (state.gsd, -1);
        
    } // while
    
    return 0;
}
