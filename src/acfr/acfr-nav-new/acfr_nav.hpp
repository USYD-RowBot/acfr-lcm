#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <libgen.h>
#include <fstream>
#include <bot_param/param_client.h>
#include "seabed_interface.hpp"
#include "auv_seabed_slam_models.hpp"
#include "handlers.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"

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
        double oas_altitude;
        double fwd_obs_dist;
        int lowRateCount;

        bool bottomLock;
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
