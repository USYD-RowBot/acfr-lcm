#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <climits>
#include <math.h>
#include <bot_param/param_client.h>
#include "acfr-common/timestamp.h"

#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_global_planner_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_vis_rawlog_t.hpp"
#include "perls-lcmtypes++/senlcm/uvc_opi_t.hpp"
#include "perls-lcmtypes++/senlcm/uvc_osi_t.hpp"
#include "perls-lcmtypes++/senlcm/uvc_rphtd_t.hpp"
#include "perls-lcmtypes++/senlcm/uvc_dvl_t.hpp"
#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"

#include "perls-lcmtypes++/acfrlcm/auv_status_short_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_global_planner_state_t.hpp"

#define RTOD (180.0/M_PI)


class HealthMonitor
{
public:
    HealthMonitor();

    int checkStatus(int64_t hbTime);
	int loadConfig(char *program_name);
    
    senlcm::uvc_opi_t uvc_opi;
    senlcm::uvc_osi_t uvc_osi;
    senlcm::uvc_rphtd_t uvc_rph;
    senlcm::uvc_dvl_t uvc_dvl;
	senlcm::gpsd3_t gps;
    acfrlcm::auv_global_planner_state_t global_state;

    lcm::LCM lcm;
    int32_t image_count;
	char *channel_name;
	int target_id;

private:
    
    int64_t gps_timeout;
	int64_t compass_timeout;
	int64_t nav_timeout;
    int64_t osi_timeout;
    int64_t opi_timeout;
    int64_t dvl_timeout;
    int64_t dvlbl_timeout;
    int64_t rph_timeout;    
    int64_t dvlbl_utime;


	
    

};

