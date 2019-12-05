#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <libgen.h>
#include <fstream>
#include <bot_param/param_client.h>
#include "libseabedcommon/seabed_interface.hpp"
#include "libflounder/auv_seabed_slam_models.hpp"
#include "handlers.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "acfr-common/timestamp.h"

extern "C"
{
#include "magfield.h"
}

using namespace std;
using namespace lcm;
using namespace libplankton;
using namespace libflounder;
using namespace perllcm;
using namespace acfrlcm;

#ifndef ACFR_NAV_HPP
#define ACFR_NAV_HPP

// Attitude source
typedef enum 
{
    ATT_TCM,
    ATT_RDI,
    ATT_OS,
    ATT_UVC	
} attitude_source_t;  


// Velocity source
typedef enum 
{
    VEL_RDI,
    VEL_UVC	
} velocity_source_t;  


// Depth source
typedef enum 
{
    DEPTH_YSI,
    DEPTH_PAROSCI,
    DEPTH_SEABIRD,
    DEPTH_OS,
    DEPTH_OS_COMPASS
} depth_source_t;

typedef enum 
{
    NAV,
    RAW
} nav_mode_t;

class state_c
{
    public:
        lcm::LCM *lcm;
        Seabed_Interface *slam;
        
        // used to be able to output data in the old format
        nav_mode_t mode;    
        ofstream raw_out;
        
        // persistant variables
        double altitude;
        double depth;
        double oas_utime;
        double oas_altitude;
        double fwd_obs_dist;
        double oas_transformed_alt;
        double sea_floor_est;
        int lowRateCount;

        bool bottomLock;
	bool broken_iver_alt;
	int64_t uvc_osi_prev_utime;
	double uvc_prev_alt;
	double iver_vz;
};

class acfr_nav
{
    public:
        acfr_nav();
        ~acfr_nav();
        int load_config(char *);
        int initialise();
        
        state_c *state;
        
        // the main loop function
        int process();
 
    private:
        void calculate_mag();

        
        // config parameters
        attitude_source_t attitude_source;
        velocity_source_t velocity_source;
        depth_source_t depth_source;
        char *slam_config_filename;
	char *evologics_channel;
                
};

#endif
