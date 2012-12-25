#include <lcm/lcm-cpp.hpp>
#include <string>
#include <iostream>
#include "mission_control.hpp"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_mission_command_t.hpp"

#include "waypoint.hpp"

#ifndef MAIN_CONTROL_HPP
#define MAIN_CONTROL_HPP

using namespace std;
using namespace acfrlcm;



// main FSM states
typedef enum
{
    main_fsm_idle,
    main_fsm_run,
    main_fsm_abort,
    main_fsm_goto_hold,
} main_control_state;


// main mission messages
typedef enum
{
    main_run,
    main_goto,
    main_error,
    main_abort,
    main_done,
    main_stop,
    main_none
} main_msg_t;

class main_control
{  
    public:
        main_control(lcm::LCM *_lcm);
        ~main_control();
        main_control_state get_current_state();
        string  get_current_state_string	();
        int update_nav(const auv_acfr_nav_t *nav);
        int message(const auv_mission_command_t *mc);
        int message(main_msg_t message);
        
    private:        
        int clock();
        lcm::LCM *lcm;
        int set_goto_location(goal_point *gp);

        main_control_state current_state;
        main_control_state next_state;
        
        main_msg_t main_message;
        
        location current_location;
        
        // variables for a goto
        goal_point goto_location;
        int64_t goto_start_time;
        
        // The mission controller and other stuff
        mission_control *mc;
        string filename;
};    

#endif
