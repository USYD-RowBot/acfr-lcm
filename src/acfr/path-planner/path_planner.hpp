#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <libgen.h>
#include <fstream>
#include <bot_param/param_client.h>
#include "DubinsPath.h"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_path_command_t.hpp"

#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

using namespace acfrlcm;
using namespace perllcm;
using namespace std;


class path_planner
{
    public:
        path_planner();
        ~path_planner();
        int process();
        int load_config(char *program_name);

        // data storage, these have to be public as they are accessed by the callbacks
        auv_acfr_nav_t nav;
        
        // LCM
        lcm::LCM lcm;        
        
        // the path object
        DubinsPath dp;
        
        // config variable
        double drop_distance;
        double drop_angle;
        double turn_radius;
        double at_goal_distance;
        
        // path variables
        int depth_mode;
        double velocity;
        vector<Pose3D> path;
        int done;
        vector<Pose3D>::iterator start_point, end_point;
};        


#endif
