#include <math.h>
#include "uvc_health.hpp"

#define COMPASS_TIMEOUT	1000000
#define GPS_TIMEOUT 	1000000
#define ECOPUCK_TIMEOUT 1000000
#define NAV_TIMEOUT 	1000000
#define IMU_TIMEOUT 	1000000
#define DVL_TIMEOUT 	1000000
#define DVLBL_TIMEOUT 	10000000 // 10sec?
#define DEPTH_TIMEOUT 	1000000
#define OAS_TIMEOUT 	1000000

#define DVL_BIT 	    0b0000000000000001 //0x0001
#define DVL_BL_BIT 	    0b0000000000000010 //0x0002
#define GPS_BIT 	    0b0000000000000100 //0x0004
#define DEPTH_BIT 	    0b0000000000001000 //0x0008
#define COMPASS_BIT	    0b0000000000010000 //0x0010
#define IMU_BIT 	    0b0000000000100000 //0x0020
#define OAS_BIT 	    0b0000000001000000 //0x0040
#define NAV_BIT 	    0b0000000010000000 //0x0080
#define ECOPUCK_BIT     0b0000000100000000 //0x0100
#define ABORT_BIT       0b0000001000000000 //0x0200
#define PRESSURE_BIT    0b0010000000000000 //0x2000
#define TEMP_BIT        0b0100000000000000 //0x4000
#define LEAK_BIT        0b1000000000000000 //0x8000
 

// Handlers

void handle_uvc_opi(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::uvc_opi_t *sensor, HealthMonitor *state)
{
	state->uvc_opi = *sensor;
}

void handle_uvc_osi(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::uvc_osi_t *sensor, HealthMonitor *state)
{
	state->uvc_osi = *sensor;
}

void handle_uvc_rph(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::uvc_rphtd_t *sensor, HealthMonitor *state)
{
	state->uvc_rph = *sensor;
}

void handle_uvc_dvl(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::uvc_dvl_t *sensor, HealthMonitor *state)
{
	state->uvc_dvl = *sensor;
}

void handle_vis(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const acfrlcm::auv_vis_rawlog_t *sensor, HealthMonitor *state)
{
	state->image_count++;
}

void handle_heartbeat(const lcm::ReceiveBuffer *rbuf,
		const std::string& channel, const perllcm::heartbeat_t *hb,
		HealthMonitor *state)
{
	state->checkStatus(hb->utime);
	
}

void handle_gps(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::gpsd3_t *sensor, HealthMonitor *state)
{
	state->gps = *sensor;
}

void handle_global_state(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const acfrlcm::auv_global_planner_state_t *sensor, HealthMonitor *state)
{
	state->global_state = *sensor;
}

HealthMonitor::HealthMonitor()
{
	// initialise member variables
	gps.utime = 0;
	dvlbl_utime = 0;
	uvc_opi.utime = 0;
	uvc_osi.utime = 0;
	uvc_dvl.utime = 0;
	uvc_rph.utime = 0;
	image_count = 0;

	// Subscribe to all the sensors we need to monitor
	lcm.subscribeFunction("UVC_OPI", handle_uvc_opi, this);
	lcm.subscribeFunction("UVC_OSI", handle_uvc_osi, this);
	lcm.subscribeFunction("UVC_DVL", handle_uvc_dvl, this);
	lcm.subscribeFunction("UVC_RPH", handle_uvc_rph, this);
	lcm.subscribeFunction("GPSD_CLIENT", handle_gps, this);
	lcm.subscribeFunction("ACFR_AUV_VIS_RAWLOG", handle_vis, this);
	lcm.subscribeFunction("GLOBAL_STATE", handle_global_state, this);

	// Subscribe to the heartbeat
	lcm.subscribeFunction("HEARTBEAT_1HZ", &handle_heartbeat, this);
}


int HealthMonitor::checkStatus(int64_t hbTime)
{
	acfrlcm::auv_status_short_t status;
	memset(&status, 0, sizeof(acfrlcm::auv_status_short_t));
	status.utime = timestamp_now();

	// check the age of the sensor data
	// Its a bit different for the Iver
	if ((hbTime - uvc_rph.utime) < compass_timeout)
		status.status |= COMPASS_BIT;
	
	if ((hbTime - gps.utime) < gps_timeout)
		status.status |= GPS_BIT;
	
	
	if ((hbTime - uvc_osi.utime) < nav_timeout)
		status.status |= NAV_BIT;
	
	
	if ((hbTime - uvc_dvl.utime) < dvl_timeout)
		status.status |= DVL_BIT;
	
	// We have bottom lock when we have a valid altitude
	if (uvc_dvl.alt < 999.0) // No value is 999.99
	{
		status.status |= DVL_BL_BIT;
		dvlbl_utime = uvc_dvl.utime;
	}
		
	if(uvc_opi.leak == 1)
	{
		status.status |= LEAK_BIT;		
	}
    
    //if(global_state.state == 0)
    //    status.status |= ABORT_BIT;

	status.latitude = (float)(uvc_osi.latitude);	
	status.longitude = (float)(uvc_osi.longitude);
	status.altitude = (unsigned char)(uvc_dvl.alt * 10.0);
	status.depth = (short)(uvc_rph.depth * 10.0);
	status.roll = (char)((uvc_rph.rph[0] * RTOD) * 10.0);
	status.pitch = (char)((uvc_rph.rph[1] * RTOD) * 10.0);
	status.heading = (short)((uvc_rph.rph[2] * RTOD) * 10.0);
	status.charge = (char)uvc_opi.percent;
	status.waypoint = (char)uvc_osi.nextwp;
	status.img_count = image_count;
	status.target_id = target_id;

	lcm.publish(channel_name, &status);

	return 1;
}

int HealthMonitor::loadConfig(char *program_name)
{
	BotParam *param = NULL;
	param = bot_param_new_from_server(lcm.getUnderlyingLCM(), 1);
	if (param == NULL)
		return 0;

	char rootkey[64];
	char key[128];
	
	sprintf(rootkey, "iver.%s", program_name);

	sprintf(key, "%s.channel", rootkey);
	channel_name = bot_param_get_str_or_fail(param, key); 

	sprintf(key, "%s.target_id", rootkey);
	target_id = bot_param_get_int_or_fail(param, key); 

	return 1;
}		


int program_exit;
void signal_handler(int sigNum)
{
	// do a safe exit
	program_exit = 1;
}

int main(int argc, char **argv)
{
	program_exit = 0;

	HealthMonitor state;

	if (!state.loadConfig(basename(argv[0])))
	{
		std::cerr << "Failed loading config" << std::endl;
		return 0;
	}

	// Loop
	int fd = state.lcm.getFileno();
	fd_set rfds;
	while (!program_exit)
	{
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		struct timeval timeout;
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;
		int ret = select(fd + 1, &rfds, NULL, NULL, &timeout);
		if (ret > 0)
			state.lcm.handle();
	}

	return 0;
}
