#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include "health_monitor.hpp"

#define COMPASS_TIMEOUT	1000000
#define GPS_TIMEOUT 	1000000
#define ECOPUCK_TIMEOUT 1000000
#define NAV_TIMEOUT 	1000000
#define IMU_TIMEOUT 	1000000
#define DVL_TIMEOUT 	1000000
#define DVLBL_TIMEOUT 	10000000 // 10sec?
#define DEPTH_TIMEOUT 	1000000
#define OAS_TIMEOUT 	1000000
#define BF_TAIL_TIMEOUT 10000000

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


using namespace std;

// Handlers

void handle_tcm(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::tcm_t *sensor, HealthMonitor *state)
{
	state->compass = *sensor;
}

void handle_imu(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::kvh1750_t *sensor, HealthMonitor *state)
{
	state->imu = *sensor;
}

void handle_gps(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::gpsd3_t *sensor, HealthMonitor *state)
{
	state->gps = *sensor;
}

void handle_ecopuck(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::ecopuck_t *sensor, HealthMonitor *state)
{
	state->ecopuck = *sensor;
}

void handle_micron(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::micron_ping_t *sensor, HealthMonitor *state)
{
	state->oas = *sensor;
}

void handle_rdi(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::rdi_pd5_t *sensor, HealthMonitor *state)
{
	state->dvl = *sensor;
}

void handle_parosci(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::parosci_t *sensor, HealthMonitor *state)
{
	state->parosci = *sensor;
}

void handle_ysi(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::ysi_t *sensor, HealthMonitor *state)
{
	state->ysi = *sensor;
}

void handle_nav(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const acfrlcm::auv_acfr_nav_t *sensor, HealthMonitor *state)
{
	state->nav = *sensor;
}

void handle_bf_tail(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const acfrlcm::auv_bluefin_tail_status_t *sensor, HealthMonitor *state)
{
	state->bf_status = *sensor;
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
	state->checkAbortConditions();
}

void handle_leak(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::leak_t *sensor, HealthMonitor *state)
{
	state->leak = *sensor;
}

void handle_global_state(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const acfrlcm::auv_global_planner_state_t *sensor, HealthMonitor *state)
{
	state->global_state = *sensor;
}

void handle_battery(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
		const senlcm::os_power_system_t *sensor, HealthMonitor *state)
{
	state->battery = *sensor;
}

void handle_path_response(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
        const acfrlcm::auv_path_response_t *sensor, HealthMonitor *state)
{
        state->path_response = *sensor;
}

HealthMonitor::HealthMonitor()
{
	// initialise member variables
	compass.utime = 0;
	gps.utime = 0;
	ecopuck.utime = 0;
	nav.utime = 0;
	imu.utime = 0;
	dvl.utime = 0;
	dvlbl_utime = 0;
	parosci.utime = 0;
	parosci.depth = 0;
	ysi.utime = 0;
	ysi.depth = 0;
	oas.utime = 0;
	bf_status.utime = 0;
	image_count = 0;
    

	abort_on_no_compass = false;
	abort_on_no_gps = false;
	abort_on_no_ecopuck = false;
	abort_on_no_nav = false;
	abort_on_no_imu = false;
	abort_on_no_dvl = false;
	abort_on_no_depth = false;
	abort_on_no_oas = false;

	compass_timeout = COMPASS_TIMEOUT;
	gps_timeout = GPS_TIMEOUT;
	ecopuck_timeout = ECOPUCK_TIMEOUT;
	nav_timeout = NAV_TIMEOUT;
	imu_timeout = IMU_TIMEOUT;
	dvl_timeout = DVL_TIMEOUT;
	dvlbl_timeout = DVLBL_TIMEOUT;
	depth_timeout = DEPTH_TIMEOUT;
	oas_timeout = OAS_TIMEOUT;
	bf_tail_timeout = BF_TAIL_TIMEOUT;
}


int HealthMonitor::subscribeChannels()
{
	// Subscribe to all the sensors we need to monitor
	lcm.subscribeFunction(vehicle_name+".TCM", handle_tcm, this);
	lcm.subscribeFunction(vehicle_name+".KVH1750", handle_imu, this);
	lcm.subscribeFunction(vehicle_name+".GPSD_CLIENT", handle_gps, this);
	lcm.subscribeFunction(vehicle_name+".ECOPUCK", handle_ecopuck, this);
	lcm.subscribeFunction(vehicle_name+".MICRON", handle_micron, this);
	lcm.subscribeFunction(vehicle_name+".RDI", handle_rdi, this);
	lcm.subscribeFunction(vehicle_name+".PAROSCI", handle_parosci, this);
	lcm.subscribeFunction(vehicle_name+".YSI", handle_ysi, this);
	lcm.subscribeFunction(vehicle_name+".ACFR_NAV", handle_nav, this);
	lcm.subscribeFunction(vehicle_name+".ACFR_AUV_VIS_RAWLOG", handle_vis, this);
	lcm.subscribeFunction(vehicle_name+".LEAK", handle_leak, this);
	lcm.subscribeFunction(vehicle_name+".GLOBAL_STATE", handle_global_state, this);
	lcm.subscribeFunction(vehicle_name+".BATTERY", handle_battery, this);
	lcm.subscribeFunction(vehicle_name+".BLUEFIN_STATUS", handle_bf_tail, this);

    // Subscribe to path response to report waypoint progress in *.AUVSTAT
    lcm.subscribeFunction(vehicle_name+".PATH_RESPONSE", handle_path_response, this);

	// Subscribe to the heartbeat
	lcm.subscribeFunction("HEARTBEAT_1HZ", &handle_heartbeat, this);
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
	double tmp_double;
	sprintf(rootkey, "acfr.%s", program_name);

	sprintf(key, "%s.min_x", rootkey);
	min_x = botu_param_get_double_or_default(param, key, LONG_MIN);
	sprintf(key, "%s.max_x", rootkey);
	max_x = botu_param_get_double_or_default(param, key, LONG_MAX);
	sprintf(key, "%s.min_y", rootkey);
	min_y = botu_param_get_double_or_default(param, key, LONG_MIN);
	sprintf(key, "%s.max_y", rootkey);
	max_y = botu_param_get_double_or_default(param, key, LONG_MAX);
	std::cout << "Operation bounding box is: \n";
	print_bounding_box();
	if( fabs(min_x - LONG_MIN) > 1e-3 ||
			fabs(max_x - LONG_MAX) > 1e-3 ||
			fabs(min_y - LONG_MIN) > 1e-3 ||
			fabs(max_y - LONG_MAX) > 1e-3 ) {
		abort_on_out_of_bound = true;
		std::cout << "We will abort when out ot bounds!!\n\n\n";
	}
	else {
		abort_on_out_of_bound = false;
		std::cout << "We will NOT abort when out ot bounds!!\n\n\n";
	}

	sprintf(key, "%s.max_depth", rootkey);
	max_depth = bot_param_get_double_or_fail(param, key);
	std::cout << "Set max depth to: " << max_depth << std::endl;

	sprintf(key, "%s.min_alt", rootkey);
	min_alt = bot_param_get_double_or_fail(param, key);
	std::cout << "Set min alt to: " << min_alt << std::endl;

	sprintf(key, "%s.max_pitch", rootkey);
	max_pitch = bot_param_get_double_or_fail(param, key);
	std::cout << "Set max pitch to: " << max_pitch << std::endl;

	sprintf(key, "%s.abort_on_no_compass", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		abort_on_no_compass = tmp_double;
	sprintf(key, "%s.compass_timeout", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		compass_timeout = tmp_double;

	sprintf(key, "%s.abort_on_no_gps", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		abort_on_no_gps = tmp_double;
	sprintf(key, "%s.gps_timeout", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		gps_timeout = tmp_double;

	sprintf(key, "%s.abort_on_no_ecopuck", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		abort_on_no_ecopuck = tmp_double;
	sprintf(key, "%s.ecopuck_timeout", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		ecopuck_timeout = tmp_double;

	sprintf(key, "%s.abort_on_no_nav", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		abort_on_no_nav = tmp_double;
	sprintf(key, "%s.nav_timeout", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		nav_timeout = tmp_double;

	sprintf(key, "%s.abort_on_no_imu", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		abort_on_no_imu = tmp_double;
	sprintf(key, "%s.imu_timeout", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		imu_timeout = tmp_double;

	sprintf(key, "%s.abort_on_no_dvl", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		abort_on_no_dvl = tmp_double;
	sprintf(key, "%s.dvl_timeout", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		dvl_timeout = tmp_double;

	sprintf(key, "%s.abort_on_no_depth", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		abort_on_no_depth = tmp_double;
	sprintf(key, "%s.depth_timeout", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		depth_timeout = tmp_double;

	sprintf(key, "%s.abort_on_no_oas", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		abort_on_no_oas = tmp_double;
	sprintf(key, "%s.oas_timeout", rootkey);
	if (0 == bot_param_get_double(param, key, &tmp_double))
		oas_timeout = tmp_double;

        //sprintf(key, "%s.vehicle_name", rootkey);
        //vehicle_name = bot_param_get_str_or_fail(param, key);
 
	return 1;
}

int HealthMonitor::checkStatus(int64_t hbTime)
{
	acfrlcm::auv_status_short_t status;
	memset(&status, 0, sizeof(acfrlcm::auv_status_short_t));
	status.utime = timestamp_now();
	status.status = 0xFFFF;
    float heading_temp = 0; // temp value to process heading before copy to char
    float pitch_temp = 0; // temp value to process heading before copy to char
    float roll_temp = 0; // temp value to process heading before copy to char

	// check the age of the sensor data
	if ((hbTime - compass.utime) < compass_timeout)
		status.status &= ~COMPASS_BIT;
	else if (abort_on_no_compass == true)
		sendAbortMessage("COMPASS dead");

	if ((hbTime - gps.utime) < gps_timeout)
		status.status &= ~GPS_BIT;
	else if (abort_on_no_gps == true)
		sendAbortMessage("GPS dead");

	if ((hbTime - ecopuck.utime) < ecopuck_timeout)
		status.status &= ~ECOPUCK_BIT;
	else if (abort_on_no_ecopuck == true)
		sendAbortMessage("ECOPUCK dead");

	if ((hbTime - nav.utime) < nav_timeout)
		status.status &= ~NAV_BIT;
	else if (abort_on_no_nav == true)
		sendAbortMessage("NAV dead");

	if ((hbTime - imu.utime) < imu_timeout)
		status.status &= ~IMU_BIT;
	else if (abort_on_no_imu == true)
		sendAbortMessage("IMU dead");

	if ((hbTime - dvl.utime) < dvl_timeout)
		status.status &= ~DVL_BIT;
	else if (abort_on_no_dvl == true)
		sendAbortMessage("DVL dead");

	if (dvl.pd4.btv_status == 0) // 0 is good
	{
		status.status &= ~DVL_BL_BIT;
		dvlbl_utime = dvl.utime;
	}
	else if (((hbTime - dvlbl_utime) >= dvlbl_timeout)
			&& (abort_on_no_dvl == true))
	{
		printf("last dvlbl at %ld. This is older than %ld. ABORTING\n\n",
				dvlbl_utime, dvlbl_timeout);
		sendAbortMessage("DVL lost bottom lock");
	}

	if ((hbTime - parosci.utime) < depth_timeout
			|| (hbTime - ysi.utime) < depth_timeout)
		status.status &= ~DEPTH_BIT;
	else if (abort_on_no_depth == true)
		sendAbortMessage("DEPTH dead");

	if ((hbTime - oas.utime) < oas_timeout)
		status.status &= ~OAS_BIT;
	else if (abort_on_no_oas == true)
		sendAbortMessage("OAS dead");
		
	if(leak.leak == 1)
	{
		status.status &= ~LEAK_BIT;
		sendAbortMessage("Leak");
	}
    
        if(global_state.state != 0)
            status.status &= ~ABORT_BIT;

	status.latitude = (float)nav.latitude*180/M_PI;	
	status.longitude = (float)nav.longitude*180/M_PI;
    //status.altitude = (unsigned char)(nav.altitude * 10.0);
    if (nav.altitude < 12.5)
        status.altitude = (char)(nav.altitude * 10.0);
    else if (nav.altitude > 125)
        status.altitude = (char)(-125);
    else
        status.altitude = (char)(-nav.altitude);

    status.depth = (short)(nav.depth * 10.0);
	//status.roll = (char)(nav.roll * 10.0 * 180 / M_PI); //DONE below - undefined for values outside +/- 12.8 degrees
    roll_temp = (nav.roll * 180 / M_PI);
    if (roll_temp > 60) // 60 max degrees reported
        status.roll = 127; // over roll indicator
    else if (roll_temp < -60)
        status.roll = -127; // under roll indicator
    else // abs(roll_temp) <= 60 
        status.roll = (char)(roll_temp * 2); // sent in quarter degree units

    //status.pitch = (char)(nav.pitch * 10.0 * 180 / M_PI); // DONE below - undefined for values outside +/- 12.8 degrees
    pitch_temp = (nav.pitch * 180 / M_PI); 
    if (pitch_temp > 60) // 60 max degrees reported
        status.pitch = 127; // over pitch indicator
    else if (pitch_temp < -60)
        status.pitch = -127; // under pitch indicator
    else // abs(pitch_temp) <= 60
        status.pitch = (char)(pitch_temp * 2); // sent in quarter degree units

    //status.heading = (short)(fmod(nav.heading *  180 / M_PI, 360)/2); //DONE - this is a char, not a short, needs to be fixed /2 is not a solution 180 > 128
    heading_temp = fmod(nav.heading *  180 / M_PI, 360); // rad to deg, limit range to -360..+360
    while (heading_temp >= 180) // limit to -180..180
        heading_temp = (heading_temp - 360); 
    while (heading_temp <= -180) // limit to -180..180 opposite case
        heading_temp = (heading_temp + 360);
    status.heading = (char)(heading_temp/2); // and halve to fit into signed char (sent as 2 degree incements)
	status.img_count = image_count;
    status.charge = (char)(battery.avg_charge_p);
    status.vel = (char)(nav.vx * 10.0);
    status.waypoint = path_response.goal_id;

	lcm.publish(vehicle_name+".AUVSTAT", &status);

	return 1;
}

int HealthMonitor::sendAbortMessage(const char *msg)
{
	acfrlcm::auv_global_planner_t abortMsg;
	abortMsg.command = acfrlcm::auv_global_planner_t::ABORT;
	abortMsg.str = msg;
	lcm.publish("TASK_PLANNER_COMMAND."+vehicle_name, &abortMsg);

	return 1;
}

int HealthMonitor::checkAbortConditions()
{
	// we haven't initialised the nav yet, can't check abort
	if (nav.utime == 0)
	{
		return 0;
	}
	// for now check depth against nav solution.  This could also consider
	// the raw sensor measurements but would have to account for tare.
	if (nav.depth > max_depth)
	{
		std::cerr << "ABORT: Exceeded max depth" << std::endl;
		sendAbortMessage("MAX_DEPTH exceeded");
	}
	if (fabs(nav.pitch) > max_pitch)
	{
		std::cerr << "ABORT: Exceeded max pitch: " << nav.pitch << std::endl;
		sendAbortMessage("MAX_PITCH exceeded");
	}
	if (nav.altitude > 0.0 && nav.altitude < min_alt)
	{
		std::cerr << "ABORT: Exceeded min altitude:" << nav.altitude
				<< std::endl;
		sendAbortMessage("MIN_ALT exceeded");
	}
	if ( abort_on_out_of_bound &&
			(nav.x < min_x || nav.x > max_x || nav.y < min_y || nav.y > max_y) ) {
		std::cerr << "ABORT: Exceeded bounding box limit: x/y = " <<
				nav.x << "/" << nav.y << std::endl;
		print_bounding_box();
		sendAbortMessage("BOUNDS exceeded");
	}
	if (bf_status.voltage <= 15)
	{
		std::cerr << "ABORT: Tail Undervoltage :" << bf_status.voltage << std::endl;
		sendAbortMessage("Tail Blackout risk");
	}

	return 0;
}

void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, HealthMonitor &state)
{
    int opt;

    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            state.setVehicleName((char*)optarg);
            break;
         }
    }
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
        parse_args(argc, argv, state);
        state.subscribeChannels();

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
