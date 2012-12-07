#include <lcm/lcm-cpp.hpp>
#include "mission.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_path_response_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_path_command_t.hpp"
#include "perls-common/timestamp.h"

#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

// mission FSM states
typedef enum
{
    mission_fsm_idle,
    mission_fsm_travel,
    mission_fsm_abort,
    mission_fsm_done,
    mission_fsm_fault
} mission_control_state;
    

typedef enum
{
    mission_run,
    mission_abort,
    mission_stop,
} mission_message_t;

class mission_control
{
    public:
        mission_control(lcm::LCM *_lcm);
        ~mission_control();
        mission_control_state get_current_state();
        string  get_current_state_string();
        lcm::LCM *lcm;
        bool are_we_there_yet;
        double distance_to_goal;

    private:
        int clock();

        pthread_mutex_t mission_message_lock;        
        mission_message_t mission_message;
        
        pthread_mutex_t state_lock;
        mission_control_state current_state;
        mission_control_state next_state;
        
        // mission stuff
        string filename;
        mission mis;
        list<goal_point>::iterator point_one, point_two;
        int send_leg();
        int send_commands(list<mission_command> &commands);
        double leg_start_time;
        
        
        
};

#endif
