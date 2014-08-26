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
#include <sys/un.h>
#include <vector>
#include <bot_param/param_client.h>
#include <proj_api.h>
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/senlcm/novatel_t.hpp"
#include "perls-lcmtypes++/senlcm/usbl_fix_t.hpp"
#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/senlcm/rt3202_t.hpp"

#ifndef LQUSBL_HPP

using namespace std;
using namespace senlcm;
using namespace perllcm;
//using namespace acfrlcm;

#define DTOR M_PI/180.0
#define RTOD 180.0/M_PI
#define MAX_BUF_LEN 1024
#define AHRS_PORT "10000"

// Attitude source
typedef enum 
{
    ATT_NOVATEL,
    ATT_RT3202
} attitude_source_t; 

// GPS source
typedef enum 
{
    GPS_NOVATEL,
    GPS_GPSD,
    GPS_RT3202
} gps_source_t; 


class Lq_Usbl
{
    public:
        Lq_Usbl();
        lcm::LCM *lcm;
        int load_config(char *program_name);
        int init();
        int process();
        
         // data holders
        gpsd3_t gpsd;
        novatel_t novatel;
        rt3202_t rt3202;


    private:
        int open_lq_socket(char *name);
        int lq_fd;
        int calc_position(double x, double y, double z, int id, int64_t timestamp);
        int parse_tp2(char *d, int64_t timestamp);

        char *pipename;
        
        attitude_source_t attitude_source;
        gps_source_t gps_source;

        // pose of the usbl to ins
        SMALL::Pose3D usbl_ins_pose;
        
        //pose of the ins to ship center
        SMALL::Pose3D ins_ship_pose;
        
        // Proj4 lat lon projection
        projPJ pj_latlong;
};

#endif
