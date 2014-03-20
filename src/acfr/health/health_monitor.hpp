#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <climits>

#include "perls-common/bot_util.h"
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"

#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_global_planner_t.hpp"
#include "perls-lcmtypes++/senlcm/tcm_t.hpp"
#include "perls-lcmtypes++/senlcm/kvh1750_t.hpp"
#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"
#include "perls-lcmtypes++/senlcm/ecopuck_t.hpp"
#include "perls-lcmtypes++/senlcm/micron_ping_t.hpp"
#include "perls-lcmtypes++/senlcm/rdi_pd5_t.hpp"
#include "perls-lcmtypes++/senlcm/parosci_t.hpp"
#include "perls-lcmtypes++/senlcm/ysi_t.hpp"

#include "perls-lcmtypes++/acfrlcm/auv_status_t.hpp"



class HealthMonitor
{
public:
    HealthMonitor();

    int loadConfig(char *programName);
    int checkStatus(int64_t hbTime);
    int sendAbortMessage(const char *);
    int checkAbortConditions();
	
    void print_bounding_box(void) {
	std::cout << "min x/y = " << min_x << " / " << min_y << std::endl
		<< "max x/y = " << max_x << " / " << max_y << std::endl;
    }
    senlcm::tcm_t compass;
    senlcm::gpsd3_t gps;
    senlcm::ecopuck_t ecopuck;
    acfrlcm::auv_acfr_nav_t nav;
    senlcm::kvh1750_t imu;
    senlcm::rdi_pd5_t dvl;
    senlcm::parosci_t parosci;
    senlcm::ysi_t ysi;
    senlcm::micron_ping_t oas;

    //lcm_t *lcm;
    lcm::LCM lcm;

private:

    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double min_depth; // not used
    double max_depth;
    double min_alt;
    double max_alt; // not used
    double max_pitch;

    bool abort_on_out_of_bound;
    bool abort_on_no_compass;
    bool abort_on_no_gps;
    bool abort_on_no_ecopuck;
    bool abort_on_no_nav;
    bool abort_on_no_imu;
    bool abort_on_no_dvl;
    bool abort_on_no_depth;
    bool abort_on_no_oas;

    int64_t compass_timeout;
    int64_t gps_timeout;
    int64_t ecopuck_timeout;
    int64_t nav_timeout;
    int64_t imu_timeout;
    int64_t dvl_timeout;
    int64_t dvlbl_timeout;
    int64_t depth_timeout;
    int64_t oas_timeout;

    int64_t dvlbl_utime;
};

