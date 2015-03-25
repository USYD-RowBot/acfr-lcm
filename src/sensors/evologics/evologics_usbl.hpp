#include <lcm/lcm-cpp.hpp>
#include <small/Pose3D.hh>
#include <iostream>
#include <deque>
#include <signal.h>
#include <string>
#include <libgen.h>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <bot_param/param_client.h>
#include <proj_api.h>
#include "perls-common/timestamp.h"
#include "perls-common/serial.h"
#include "perls-lcmtypes++/senlcm/novatel_t.hpp"
#include "perls-lcmtypes++/senlcm/usbl_fix_t.hpp"
#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"
#include "perls-lcmtypes++/senlcm/evologics_usbl_t.hpp"
#include "perls-lcmtypes++/senlcm/evologics_command_t.hpp"
#include "perls-lcmtypes++/senlcm/ahrs_t.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_global_planner_t.hpp"

using namespace std;
using namespace senlcm;
using namespace perllcm;
using namespace acfrlcm;

#define DTOR M_PI/180.0
#define RTOD 180.0/M_PI
#define MAX_BUF_LEN 1024
#define AHRS_PORT "10000"

#define MAX_TARGETS 8

// Attitude source
typedef enum 
{
    ATT_NOVATEL = 0,
    ATT_EVOLOGICS_AHRS,
    ATT_EVOLOGICS_COMPENSATED,
    ATT_AUV_STATUS
} attitude_source_t; 

// GPS source
typedef enum 
{
    GPS_NOVATEL = 0,
    GPS_GPSD,
    GPS_AUV_STATUS
} gps_source_t; 

    

class Evologics_Usbl
{
    public:
        Evologics_Usbl();
        ~Evologics_Usbl();
        lcm::LCM *lcm;
        int load_config(char *program_name);
        int init();
        int process();
        int process_usblfix(const std::string& channel, const evologics_usbl_t *evo);
//        int ping_targets();
//        int get_target_channel(const char *target_name);
//        int get_target_name(int target_channel, char *target_name);
//        int parse_ahrs_message(char *buf);
        int on_lcm_data(const lcm::ReceiveBuffer* rbuf, const std::string& channel, bool use_pbm = false);
        
        // data holders
        gpsd3_t gpsd;
        deque<novatel_t *> novatelq;
        novatel_t novatel;
        ahrs_t ahrs;
        
        bool send_fixes;

    private:
        bool has_ahrs;
        attitude_source_t attitude_source;
        gps_source_t gps_source;
            
        // pose of the usbl to ins
        SMALL::Pose3D usbl_ins_pose;
        
        //pose of the ins to ship center
        SMALL::Pose3D ins_ship_pose;
        
        // Proj4 lat lon projection
        projPJ pj_latlong;
        
//        int usbl_send_counter[MAX_TARGETS];    
//        int usbl_send[MAX_TARGETS]; 
        
};
         
