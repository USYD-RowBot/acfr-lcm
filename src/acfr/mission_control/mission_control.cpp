#include "mission_control.hpp"

mission_control::mission_control(lcm::LCM *_lcm)
{
    lcm = _lcm;
    pthread_mutex_init(&state_lock);
    
    // subscribe to the relevant LCM messages
    
}

mission_control::~mission_control()
{
    pthread_mutex_destroy(&state_lock);
}

string mission_control::get_current_state_string()
{
    char *mission_control_state_str[] = {"idle", "heading", "goto", "line", "goal reached", "abort", "done", "fault"};
    return mission_control_state_str[current_state];
}

mission_control_state mission_control::get_current_state()
{
    return current_state;
}

int mission_control::clock()
{
    main_msg_t new_message;

    switch(current_state)
    {
        
