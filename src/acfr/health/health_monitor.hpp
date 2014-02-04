#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <stdio.h>
#include <signal.h>
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
    int checkAbortConditions();

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
    double max_depth;

    int64_t compass_timeout;
    int64_t gps_timeout;
    int64_t ecopuck_timeout;
    int64_t nav_timeout;
    int64_t imu_timeout;
    int64_t dvl_timeout;
    int64_t dvlbl_timeout;
    int64_t depth_timeout;
    int64_t oas_timeout;
};

