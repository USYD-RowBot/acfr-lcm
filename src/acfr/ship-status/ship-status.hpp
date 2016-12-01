#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <signal.h>

#include "acfr-common/timestamp.h"
#include "acfr-common/units.h"

// LCM types we will be listening to
#include "perls-lcmtypes++/senlcm/novatel_t.hpp"
#include "perls-lcmtypes++/senlcm/ahrs_t.hpp"
#include "perls-lcmtypes++/senlcm/rt3202_t.hpp"
#include "perls-lcmtypes++/senlcm/posmv_t.hpp"
#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/ship_status_t.hpp"

#ifndef _SHIP_STATUS_HPP
#define _SHIP_STATUS_HPP

 

using namespace senlcm;
using namespace perllcm;
using namespace acfrlcm;
using namespace std;

typedef enum {  
        GPS_NOVATEL,
        GPS_GPSD,
        GPS_RT3202,
        GPS_STATIC,
        GPS_POSMV 
    } gps_source_t;
        
typedef enum {  
        ATT_NOVATEL,
        ATT_RT3202,
        ATT_AHRS,
        ATT_POSMV 
    } att_source_t;
    

class Ship_Status
{
    public:
        Ship_Status(char *root_key);
        ~Ship_Status();
        int process();
        
        int send_status();
        
        // structures to hold sensors
        novatel_t novatel;
        rt3202_t rt3202;
        gpsd3_t gpsd3;
        ahrs_t ahrs;
        posmv_t posmv;
        double mag_dec;
        double static_lat;
        double static_lon;
        
        lcm::LCM *lcm;
        
        // What we are using
        gps_source_t gps_source;
        att_source_t att_source;
        
        int ship_id;
        char *ship_name;
};


#endif // _SHIP_STATUS_HPP
