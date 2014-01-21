#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <libgen.h>
#include <fstream>
#include <libplankton/auv_geometry.hpp>
#include <bot_param/param_client.h>
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_path_command_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_control_t.hpp"

#ifndef LOCAL_PLANNER_HPP
#define LOCAL_PLANNER_HPP

using namespace libplankton;
using namespace acfrlcm;
using namespace perllcm;
using namespace std;


class local_planner
{
    public:
        local_planner();
        ~local_planner();
        int process();
        int load_config(char *program_name);
        Vector transform_to_earth(Vector point);
        Vector transform_to_body(Vector point);

        // data storage, these have to be public as they are accessed by the callbacks
        auv_acfr_nav_t nav;
        auv_path_command_t path_command;
        
        // config variables
        double look_ahead_velocity_scale;
        double turning_radius;
        double max_dist_from_line;
        double max_angle_from_line;
        double velocity_change_distance;
                
        lcm::LCM lcm;        
        
        
};        
        



#endif
