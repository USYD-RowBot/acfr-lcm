#include "mission_control.hpp"

void on_leg_response(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const acfrlcm::auv_path_response_t *pr, mission_control* mc)
{
    mc->are_we_there_yet = pr->are_we_there_yet;
    mc->distance_to_goal = pr->distance;
    
    // clock the FSM
    clock();
}

// Process an external message
void mission_control::message(mission_message_t message)
{
    mission_message = message;

    // clock the FSM
    clock();
}


mission_control::mission_control(lcm::LCM *_lcm)
{
    lcm = _lcm;
    
    // subscribe to the relevant LCM messages
    lcm->subscribeFunction("LEG_RESPONSE", on_leg_response, this);
}

mission_control::~mission_control()
{
}

string mission_control::get_current_state_string()
{
    string mission_control_state_str[] = {"idle", "travel", "abort", "done", "fault"};
    return mission_control_state_str[current_state];
}

mission_control_state mission_control::get_current_state()
{
    return current_state;
}

void mission_control::set_filename(string _filename)
{
    filename = _filename;
}

int mission_control::clock()
{
    //main_msg_t new_message;

    switch(current_state)
    {
            
        // We only leave the idle state by going to the travel state
        case mission_fsm_idle:
            if(mission_message == mission_run)
            {
                // load a mission
                if(!mis.load(filename))

                {
                    cerr << "Could not load mission file " << filename << endl;
                    next_state = mission_fsm_idle;    
                }
                
                // set the start point
                point_one = mis.goal_points.begin();
                next_state = mission_fsm_travel;
            }
            else
                next_state = mission_fsm_idle;
                
            break;
            
        case mission_fsm_travel:
            if(mission_message == mission_abort)
                next_state = mission_fsm_abort;
            else if(mission_message == mission_stop) 
                next_state = mission_fsm_idle;
            else
            {
                // check to see if we have reached our destination or we have hit the timeout, 
                // if so we can feed the next leg to the path planner
                if(are_we_there_yet || ((timestamp_now() - leg_start_time) > (*point_one).timeout))
                {
                    // load the next point and send it along, that is if we are not
                    // at the end of the list
                    list<goal_point>::iterator i = point_two;
                    i++;
                    if(i == mis.goal_points.end())
                        next_state = mission_fsm_done;
                    else
                    {
                        point_one++;
                        send_leg();
                        next_state = mission_fsm_travel;
                    }
                }
            }
            break;
            
        case mission_fsm_abort:
            // We end up here by an external abort message
            // we will signal the main control layer to go to abort
            // then go into idle
            next_state = mission_fsm_idle;
            break;
        case mission_fsm_done:
            // the mission is complete, go back to idle
            // send a done message up a layer to the main controller
            next_state = mission_fsm_idle;
            break;
        case mission_fsm_fault:
            // with any luck we will never end up here, the state machine will need an external
            // signal to be able to leave this state, to be implemented
            next_state = mission_fsm_fault;
            break;
    }
    
    if(next_state != current_state)      
    {  
        cout << "Current state: " << get_current_state_string() << "  ";
        current_state = next_state;
        cout << "New state: " << get_current_state_string() << endl;
    }
    
    return 0;
}    
        
int mission_control::send_leg()
{
    // we need to check that both the points we have are goal points
    // if they are not then we need to send the commands in the command
    // point and increment the indexes appropriately
    
    while((*point_one).type == COMMAND)
    {
        send_commands((*point_one).commands);
        point_one++;
    }
    
    // point two is the next point along
    point_two = point_one;
    point_two++;
    
    // check to make sure it is not a command point
    // it it is step over it and we will execute them next time round
    while((*point_two).type == COMMAND)
    {
        point_two++;
    }

    // execute the commands inside point one
    send_commands((*point_one).commands);

    // Put together the LEG message for the path planner
    acfrlcm::auv_path_command_t pc;
    pc.utime = timestamp_now();
    pc.wpt_one[0] = (*point_one).loc.getX();
    pc.wpt_one[1] = (*point_one).loc.getY();
    pc.wpt_one[2] = (*point_one).loc.getZ();
    pc.wpt_one[3] = (*point_one).loc.getRollRad();
    pc.wpt_one[4] = (*point_one).loc.getPitchRad();
    pc.wpt_one[5] = (*point_one).loc.getYawRad();
    pc.wpt_one[6] = (*point_one).vel[0];

    pc.wpt_two[0] = (*point_two).loc.getX();
    pc.wpt_two[1] = (*point_two).loc.getY();
    pc.wpt_two[2] = (*point_two).loc.getZ();
    pc.wpt_two[3] = (*point_two).loc.getRollRad();
    pc.wpt_two[4] = (*point_two).loc.getPitchRad();
    pc.wpt_two[5] = (*point_two).loc.getYawRad();
    pc.wpt_two[6] = (*point_two).vel[0];

    pc.command_type = acfrlcm::auv_path_command_t::DOUBLE;

    pc.depth_mode = (*point_one).depth_mode;

    // Send the message
    lcm->publish("LEG_COMMAND", &pc);
    leg_start_time = timestamp_now();

    return 1;
}
 
 // Send commands to instruments
int mission_control::send_commands(list<mission_command> &commands)
{
    return 1;
}
 

