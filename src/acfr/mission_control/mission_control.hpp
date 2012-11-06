#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

// mission FSM states
typedef enum
{
    mission_fsm_idle,
    mission_fsm_heading,
    mission_fsm_goto,
    mission_fsm_line,
    mission_fsm_goal_reached,
    mission_fsm_abort,
    mission_fsm_done,
    mission_fsm_fault
} mission_control_state;
    

typedef enum
{
    mission_goto,
    mission_line,
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

    private:
        int clock();
        
        pthread_mutex_t state_lock;
        mission_control_state current_state;
        mission_control_state next_state;
        
};

#endif
