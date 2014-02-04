#include "health_monitor.hpp"


#define COMPASS_TIMEOUT 1000000
#define GPS_TIMEOUT 1000000
#define ECOPUCK_TIMEOUT 1000000
#define NAV_TIMEOUT 1000000
#define IMU_TIMEOUT 1000000
#define DVL_TIMEOUT 1000000
#define DEPTH_TIMEOUT 1000000
#define OAS_TIMEOUT 1000000

#define COMPASS_BIT 0x0010
#define GPS_BIT 0x0004
#define ECOPUCK_BIT 0x0100
#define NAV_BIT 0x0080
#define IMU_BIT 0x0020
#define DVL_BIT 0x0001
#define DVL_BL_BIT 0x0002
#define DEPTH_BIT 0x0008
#define OAS_BIT 0x0040


// Handlers

void handle_tcm(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const senlcm::tcm_t *sensor, HealthMonitor *state)
{
    state->compass = *sensor;
}

void handle_imu(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const senlcm::kvh1750_t *sensor, HealthMonitor *state)
{
    state->imu = *sensor;
}

void handle_gps(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const senlcm::gpsd3_t *sensor, HealthMonitor *state)
{
    state->gps = *sensor;
}

void handle_ecopuck(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const senlcm::ecopuck_t *sensor, HealthMonitor *state)
{
    state->ecopuck = *sensor;
}

void handle_micron(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const senlcm::micron_ping_t *sensor, HealthMonitor *state)
{
    state->oas = *sensor;
}

void handle_rdi(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const senlcm::rdi_pd5_t *sensor, HealthMonitor *state)
{
    state->dvl = *sensor;
}

void handle_parosci(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const senlcm::parosci_t *sensor, HealthMonitor *state)
{
    state->parosci = *sensor;
}

void handle_ysi(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const senlcm::ysi_t *sensor, HealthMonitor *state)
{
    state->ysi = *sensor;
}

void handle_nav(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const acfrlcm::auv_acfr_nav_t *sensor, HealthMonitor *state)
{
    state->nav = *sensor;
}

void handle_heartbeat(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const perllcm::heartbeat_t *hb, HealthMonitor *state)
{
    state->checkStatus(hb->utime);
    state->checkAbortConditions();
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
    parosci.utime = 0;
    parosci.depth = 0;
    ysi.utime = 0;
    ysi.depth = 0;
    oas.utime = 0;
    
    compass_timeout = COMPASS_TIMEOUT;
    gps_timeout = GPS_TIMEOUT;
    ecopuck_timeout = ECOPUCK_TIMEOUT;
    nav_timeout = NAV_TIMEOUT;
    imu_timeout = IMU_TIMEOUT;
    dvl_timeout = DVL_TIMEOUT;
    depth_timeout = DEPTH_TIMEOUT;
    oas_timeout = OAS_TIMEOUT;

    //lcm = NULL; 
    // Subscribe to all the sensors we need to monitor
    lcm.subscribeFunction("TCM", handle_tcm, this);
    lcm.subscribeFunction("KVH1550", handle_imu, this); 
    lcm.subscribeFunction("GPSD_CLIENT", handle_gps, this); 
    lcm.subscribeFunction("ECOPUCK", handle_ecopuck, this); 
    lcm.subscribeFunction("MICRON", handle_micron, this); 
    lcm.subscribeFunction("RDI", handle_rdi, this);  
    lcm.subscribeFunction("PAROSCI", handle_parosci, this); 
    lcm.subscribeFunction("YSI", handle_ysi, this); 
    lcm.subscribeFunction("ACFR_NAV", handle_nav, this); 
    
    // Subscribe to the heartbeat
    lcm.subscribeFunction("HEARTBEAT_1HZ", &handle_heartbeat, this); 
}

int HealthMonitor::loadConfig(char *program_name)
{
        BotParam *param = NULL;
        param = bot_param_new_from_server(lcm.getUnderlyingLCM(), 1);
        if (param == NULL
        )
                return 0;

        char rootkey[64];
        char key[128];
        sprintf(rootkey, "acfr.%s", program_name);

        sprintf(key, "%s.max_depth", rootkey);
        max_depth = bot_param_get_double_or_fail(param, key);
	std::cout << "Set max depth to: " << max_depth << std::endl;

	return 1;
}

int HealthMonitor::checkStatus(int64_t hbTime)
{
    acfrlcm::auv_status_t status;
    memset(&status, 0, sizeof(acfrlcm::auv_status_t));
    status.utime = timestamp_now();
    
    // check the age of the sensor data
    if((hbTime - compass.utime) < compass_timeout)
        status.status |= COMPASS_BIT;
    
    if((hbTime - gps.utime) < gps_timeout)
        status.status |= GPS_BIT;

    if((hbTime - ecopuck.utime) < ecopuck_timeout)
        status.status |= ECOPUCK_BIT;

    if((hbTime - nav.utime) < nav_timeout)
        status.status |= NAV_BIT;

    if((hbTime - imu.utime) < imu_timeout)
        status.status |= IMU_BIT;

    if((hbTime - dvl.utime) < dvl_timeout)
        status.status |= DVL_BIT;

    if(dvl.pd4.btv_status != 0)
        status.status |= DVL_BIT;

    if((hbTime - parosci.utime) < depth_timeout || (hbTime - ysi.utime) < depth_timeout)
        status.status |= DEPTH_BIT;

    if((hbTime - oas.utime) < oas_timeout)
        status.status |= OAS_BIT;


    lcm.publish("AUV_HEALTH", &status);

    return 1;
}

int HealthMonitor::checkAbortConditions()
{
    // for now check depth against nav solution.  This could also consider
    // the raw sensor measurements but would have to account for tare.
    if (nav.depth > max_depth)
    {
	std::cout << "ABORT: Exceeded max depth" << std::endl;
	acfrlcm::auv_global_planner_t abortMsg;
	abortMsg.command = acfrlcm::auv_global_planner_t::ABORT;
	abortMsg.str = "MAX_DEPTH exceeded";
	lcm.publish("TASK_PLANNER_COMMAND", &abortMsg);
    }
    return 0;
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

    if (!state.loadConfig(basename(argv[0]))) {
	std::cerr << "Failed loading config" << std::endl;
	return 0;
    }
    
    // Loop
    int fd = state.lcm.getFileno();
    fd_set rfds;
    while (!program_exit) {
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
