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
#include "perls-lcmtypes++/acfrlcm/ship_status_t.hpp"
#include "perls-lcmtypes++/senlcm/usbl_fix_t.hpp"
#include "perls-lcmtypes++/senlcm/novatel_t.hpp"
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
        int on_lcm_data(const lcm::ReceiveBuffer* rbuf, const std::string& channel, bool use_pbm = false);
        
        // data holders
        deque<ship_status_t *> ship_statusq;
        ship_status_t ship_status;
        
        bool send_fixes;

    private:
        // pose of the usbl to ins
        SMALL::Pose3D usbl_ins_pose;
        
        //pose of the ins to ship center
        SMALL::Pose3D ins_ship_pose;
        
        // Proj4 lat lon projection
        projPJ pj_latlong;

        char *ship_status_channel_str;
};
         
