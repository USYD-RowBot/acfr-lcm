/* Interface between UVC and LCM for controlling the Iver and getting the vehicle state.
 *
 *	Christian Lees
 *	ACFR
 *	23/4/15
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <libgen.h>
#include <math.h>

#include <bot_param/param_client.h>

#include "acfr-common/nmea.h"
#include "acfr-common/sensor.h"
#include "acfr-common/socket.h"

#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/senlcm_uvc_ack_t.h"
#include "perls-lcmtypes/senlcm_uvc_opi_t.h"
#include "perls-lcmtypes/senlcm_uvc_osi_t.h"
#include "perls-lcmtypes/senlcm_uvc_rphtd_t.h"
#include "perls-lcmtypes/senlcm_uvc_dvl_t.h"
#include "perls-lcmtypes/senlcm_gpsd3_t.h"

#define DTOR M_PI/180
#define UNITS_KNOT_TO_METER_PER_SEC (0.514444444)
#define UNITS_FEET_TO_METER          (0.3048)

#define GPS_PORT 10066

typedef struct
{
    lcm_t *lcm;
    acfr_sensor_t *sensor;
	udp_info_t gps_udp_info;
	int osi_mode;
	char *base_path;
} state_t;

int start_lcm_logger(senlcm_uvc_osi_t *osi, state_t *state)
{
	
	// We need to detect when a mission has stopped so we can stop the logger
	if(state->osi_mode == SENLCM_UVC_OSI_T_MODE_NORMAL && osi->mode == SENLCM_UVC_OSI_T_MODE_STOPPED)
	{
		system("killall -9 lcm-logger");	
	}
	else if(state->osi_mode == SENLCM_UVC_OSI_T_MODE_STOPPED && osi->mode == SENLCM_UVC_OSI_T_MODE_NORMAL)
	{
		// Generate a file path for the mission		
		char path_name[256] = {'\0'};
		strcpy(path_name, state->base_path);
		if(path_name[strlen(path_name) - 1] != '/')
                strcat(path_name, "/");

		struct timeval tv;
		timestamp_to_timeval(osi->utime, &tv);
		struct tm tm;
		localtime_r (&tv.tv_sec, &tm);
		char mission_date_name[64];
		memset(mission_date_name, 0, sizeof(mission_date_name));
		
		char acfr_format_r[32] = "r%Y%m%d_%H%M%S_";
		strftime(mission_date_name, sizeof(mission_date_name), acfr_format_r, &tm);
		strcat(path_name, mission_date_name);
		strcat(path_name, osi->mission_name);

		char acfr_format_d[64] = "/d%Y%m%d_%H%M%S/lcm_%Y%m%d_%H%M%S";		 
		strftime(mission_date_name, sizeof(mission_date_name), acfr_format_d, &tm);
		strcat(path_name, mission_date_name);
		
		char cmd_str[256];
		sprintf(cmd_str, "mkdir -p %s\n", path_name);
		system(cmd_str);

		
		sprintf(cmd_str, "mkdir -p %s\n", path_name);
		system(cmd_str);
		sprintf(cmd_str, "lcm-logger -v -c PROSILICA_..16 -i --split-mb=64 %s &\n", path_name);
		system(cmd_str);
	}

	state->osi_mode = osi->mode;

	return 1;
}

/* returns 1 for successfully classifying and publishing uvc to
 * backseat driver messages */
static int
parse_msg(state_t *state, const char *msg, const int64_t timestamp)
{
    // $OSI -- vehicle state data
    if (0==strncmp (msg, "$OSI", 4))
    {
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

        char park[128];
        double park_time = 0;
        if (!nmea_arg (msg, 10, park))
            ERROR ("unable to parse OSI park");
        else
            park_time = atoi (park+1);

        double magnetic_dec = 0;
        if (!nmea_argf (msg, 11, &magnetic_dec))
            ERROR ("unable to parse OSI magnetic_dec");

		char mission_name[128] ={'\0'};
		if (!nmea_arg (msg, 12, mission_name))
            ERROR ("unable to parse current mission name");

		double mission_time_remaining = 0;
        if (!nmea_argf (msg, 13, &mission_time_remaining))
            ERROR ("unable to parse mission time remaining");

        senlcm_uvc_osi_t data =
        {
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
			.mission_name	= mission_name,
			.mission_time_remaining = mission_time_remaining,
        };
        senlcm_uvc_osi_t_publish (state->lcm, "UVC_OSI", &data);

		// start or stop the logger
		start_lcm_logger(&data, state);

        return 1;
    }

    // $OPI -- vehicle power data
    else if (0==strncmp (msg,"$OPI",4))
    {
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
        switch (bs)
        {
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

        senlcm_uvc_opi_t data =
        {
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
        senlcm_uvc_opi_t_publish (state->lcm, "UVC_OPI", &data);
        return 1;
    }

    // $C#P#R#T#D# -- uvc to host compass string
    else if (0==strncmp (msg, "$C", 2))
    {
        double heading, pitch, roll, temp, depth = 0;
        if (5!=sscanf (msg, "$C%lfP%lfR%lfT%lfD%lf", &heading, &pitch, &roll, &temp, &depth))
        {
            ERROR ("unable to parse UVC Compass string!");
        }

        senlcm_uvc_rphtd_t data =
        {
            .utime         = timestamp,
            .rph           = {roll*DTOR, pitch*DTOR, heading*DTOR},
            .T             = temp,
            .depth         = depth * UNITS_FEET_TO_METER,
        };
        senlcm_uvc_rphtd_t_publish (state->lcm, "UVC_RPH", &data);
        return 1;
    }
	// $DVL -- uvc to host DVL string
    else if (0==strncmp (msg, "$DVL", 4))
    {

        double vx = 0;
        if (!nmea_argf (msg, 1, &vx))
            ERROR ("unable to parse DVL vx");

        double vy = 0;
        if (!nmea_argf (msg, 2, &vy))
            ERROR ("unable to parse DVL vy");

        double vz = 0;
        if (!nmea_argf (msg, 3, &vz))
            ERROR ("unable to parse DVL vz");

        double xdist = 0;
        if (!nmea_argf (msg, 4, &xdist))
            ERROR ("unable to parse DVL xdist");

        double ydist = 0;
        if (!nmea_argf (msg, 5, &ydist))
            ERROR ("unable to parse DVL ydist");

        double dfs = 0;
        if (!nmea_argf (msg, 6, &dfs))
            ERROR ("unable to parse DVL dfs");

        double alt = 0;
        if (!nmea_argf (msg, 7, &alt))
            ERROR ("unable to parse DVL alt");

        senlcm_uvc_dvl_t data =
        {
            .utime = timestamp,
            .vx = vx,
            .vy = vy,
            .vz = vz,
            .xdist = xdist,
            .ydist = ydist,
            .dfs = dfs,
            .alt = alt
        };
        senlcm_uvc_dvl_t_publish (state->lcm, "UVC_DVL", &data);
        return 1;
    }
	// $GPRMC -- uvc to host GPS string
    else if (0==strncmp (msg, "$GPRMC", 6))
    {
		// we will be rebroadcasting the RMC message so that gpsd can be run
		send_udp(&state->gps_udp_info, msg, strlen(msg));
	}
    // $ACK -- uvc ack of backseat driver cmd
    else if (0==strncmp (msg,"$ACK",4))
    {
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
        if (msg_type == SENLCM_UVC_ACK_T_ACK_ORWSET)
        {
            if (!nmea_arg (msg, 4, usrset))
                ERROR ("unable to parse ACK usrset");

            if (!nmea_argi (msg, 5, &usrnum))
                ERROR ("unable to parse ACK usrnum");

            if (!nmea_arg (msg,6,usrval))
                ERROR ("unable to parse ACK usrval");
        }

        senlcm_uvc_ack_t data =
        {
            .utime    = timestamp,
            .msg_type = msg_type,
            .status   = status,
            .err_num  = err_num,
            .usrset   = usrset,
            .usrnum   = usrnum,
            .usrval   = usrval,
        };
        senlcm_uvc_ack_t_publish (state->lcm, "UVC_ACK", &data);
        return 1;
    }

    return 0;
}


void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    // need to send a message once a second to UVC to ask for the state information

    state_t *state = (state_t *)u;
    char msg[64];
    nmea_sprintf (msg, "$OSD,G,C,S,P,Y,D,*");

    acfr_sensor_write(state->sensor, msg, strlen(msg));
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

int main (int argc, char *argv[])
{

    // install the signal handler
    program_exit = 0;
    broken_pipe = 0;
    signal(SIGINT, signal_handler);
    //signal(SIGPIPE, signal_handler);

    //Initalise LCM object - specReading
    state_t state;
    state.lcm = lcm_create(NULL);

    char rootkey[64], key[128];
    sprintf(rootkey, "iver.%s", basename(argv[0]));

    state.sensor = acfr_sensor_create(state.lcm, rootkey);
    if(state.sensor == NULL)
        return 0;

	// get the base image path
	sprintf(key, "%s.base_path", rootkey);
	state.base_path = bot_param_get_str_or_fail(state.sensor->param, key); 
	state.osi_mode = SENLCM_UVC_OSI_T_MODE_STOPPED;

    acfr_sensor_canonical(state.sensor, '\n', '\r');
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);

	// create a UDP socket to send the GPRMC message out on
	create_udp_send(&state.gps_udp_info, "127.0.0.1", GPS_PORT);

    char buf[128];
    fd_set rfds;
    struct timeval tv;
    int lcm_fd = lcm_get_fileno(state.lcm);

    while(!program_exit)
    {
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        FD_ZERO(&rfds);
        FD_SET(state.sensor->fd, &rfds);
        FD_SET(lcm_fd, &rfds);
        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret > 0)
        {
            int64_t timestamp = timestamp_now();
            if(FD_ISSET(lcm_fd, &rfds))
                lcm_handle(state.lcm);
            if(FD_ISSET(state.sensor->fd, &rfds))
            {
                memset(buf, 0, sizeof(buf));
                int ret = acfr_sensor_read(state.sensor, buf, sizeof(buf));
                {
                    if(ret > 0)
                        parse_msg(&state, buf, timestamp);
                }
            }
        }
    }

    return 0;
}
