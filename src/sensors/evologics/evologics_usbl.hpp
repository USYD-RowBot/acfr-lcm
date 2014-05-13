#include <lcm/lcm-cpp.hpp>
#include <small/Pose3D.hh>
#include <iostream>
#include <signal.h>
#include <string>
#include <libgen.h>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <bot_param/param_client.h>
#include <proj_api.h>
#include "evologics.h"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/senlcm/novatel_t.hpp"
#include "perls-lcmtypes++/senlcm/usbl_fix_t.hpp"
#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"
#include "perls-lcmtypes++/senlcm/evologics_usbl_t.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_global_planner_t.hpp"

using namespace std;
using namespace senlcm;
using namespace perllcm;
using namespace acfrlcm;

#define DTOR M_PI/180.0
#define RTOD 180.0/M_PI
#define MAX_BUF_LEN 1024

// Attitude source
typedef enum 
{
    ATT_NOVATEL,
    ATT_EVOLOGICS
} attitude_source_t; 

// GPS source
typedef enum 
{
    GPS_NOVATEL,
    GPS_GPSD
} gps_source_t; 


class Evologics_Usbl
{
    public:
        Evologics_Usbl();
        lcm::LCM *lcm;
        int load_config(char *program_name);
        int init();
        int process();
        int calc_position();
        int ping_targets();
        int task_command(const auv_global_planner_t *task);
        
        // data holders
        gpsd3_t gpsd;
        novatel_t novatel;
        
        
        
    private:
        // usbl tcp config
        char *ip;
        char *inet_port;
        
        // targets
        int targets[8];
    
        el_state_t state;
        attitude_source_t attitude_source;
        gps_source_t gps_source;
        
        
    
        // pose of the usbl to ins
        SMALL::Pose3D usbl_ins_pose;
        
        //pose of the ins to ship center
        SMALL::Pose3D ins_ship_pose;
        
        // Proj4 lat lon projection
        projPJ pj_latlong;
        
        int ping_period;
        int ping_counter;        
};
         
