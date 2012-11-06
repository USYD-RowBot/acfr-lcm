#include <lcm/lcm-cpp.hpp>
#include <string>
#include <iostream>
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_mission_command_t.hpp"

#ifndef MAIN_CONTROL_HPP
#define MAIN_CONTROL_HPP

using namespace std;
using namespace acfrlcm;

// location class
class location
{
    public:
        double x;
        double y;
        double z;
        int64_t time;
};

// goal class, inherites waypoint and adds a time out and velocity
class goal_point : public location
{
    public:
        double timeout;
        double xy_vel;
        double z_vel;
};

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
        
    private:        
        int clock();
        lcm::LCM *lcm;
        int set_goto_location(goal_point *gp);

        pthread_mutex_t state_lock;
        main_control_state current_state;
        main_control_state next_state;
        
        pthread_mutex_t main_message_lock;
        main_msg_t main_message;
        
        pthread_mutex_t current_location_lock;
        location current_location;
        
        // variables for a goto
        pthread_mutex_t goto_location_lock;
        goal_point goto_location;
        int64_t goto_start_time;
};    

#endif
