#include "main_control.hpp"


// LCM handlers
void on_acfr_nav(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const acfrlcm::auv_acfr_nav_t *nav, main_control* mc)
{
    mc->update_nav(nav);
}

void on_mission_command(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const acfrlcm::auv_mission_command_t *message, main_control* mc)
{
    mc->message(message);
}



main_control::main_control(lcm::LCM *_lcm)
{
    pthread_mutex_init(&state_lock, NULL);
    pthread_mutex_init(&current_location_lock, NULL);
    pthread_mutex_init(&main_message_lock, NULL);
    pthread_mutex_init(&goto_location_lock, NULL);
    
    current_state = main_fsm_idle;
    next_state = main_fsm_idle;
    lcm = _lcm;
    
    // install the LCM handlers we need
    lcm->subscribeFunction("ACFR_NAV", on_acfr_nav, this);
    lcm->subscribeFunction("MISSION_COMMAND", on_mission_command, this);
}

main_control::~main_control()
{
    pthread_mutex_destroy(&state_lock);
    pthread_mutex_destroy(&current_location_lock);
    pthread_mutex_destroy(&main_message_lock);
    pthread_mutex_destroy(&goto_location_lock);
}

string main_control::get_current_state_string()
{
    string main_state_str[] = {"idle", "run", "abort", "goto_hold"};
    return main_state_str[current_state];
}

main_control_state main_control::get_current_state()
{
    return current_state;
}

int main_control::update_nav(const auv_acfr_nav_t *nav)
{
    pthread_mutex_lock(&current_location_lock);
    current_location.x = nav->x;
    current_location.y = nav->y;
    current_location.z = nav->depth;
    current_location.time = nav->utime;    
    pthread_mutex_unlock(&current_location_lock);
    
    clock();
    
    return 1;
}

int main_control::message(const auv_mission_command_t *mc)
{
    goal_point gp;
    main_msg_t new_message;

    switch(mc->message)
    {
        case auv_mission_command_t::ABORT:
            new_message = main_abort;
            break;
        case auv_mission_command_t::GOTO:
            gp.x = mc->x;
            gp.y = mc->y;
            gp.z = mc->z;
            gp.xy_vel = mc->xy_vel;
            gp.z_vel = mc->z_vel;
	        gp.timeout = (int64_t)(mc->timeout * 1e6);
            new_message = main_goto;
            break;
        //case ACFRLCM_AUV_MISSION_COMMAND_T_PAUSE:
        //    state->main_msg = main_pause;
        //    break;
        //case ACFRLCM_AUV_MISSION_COMMAND_T_RESUME:
        //    state->main_msg = main_goto;
        //    break;
        case auv_mission_command_t::RUN:
            new_message = main_run;
            break;
        case auv_mission_command_t::LOAD:
            // to be implemented later
            break;
        case auv_mission_command_t::STOP:
            new_message = main_stop;
            break;
    }    
    
    
    // send a mission command ack message, which just happens to be the same data type as the command
    // this is ued by the LQ_Modem driver as well as things like the gui and teleop

    auv_mission_command_t mr;
    memcpy(&mr, mc, sizeof(auv_mission_command_t));
    mr.utime = timestamp_now();
    char str = 0;
    mr.str = &str;
//    acfrlcm_auv_mission_command_t_publish(state->lcm, "MISSION_ACK", &mr);


    pthread_mutex_lock(&main_message_lock);
    main_message = new_message;        
    pthread_mutex_unlock(&main_message_lock);
    
    if(main_message == main_goto)
        set_goto_location(&gp);

    
    clock();
    
    return 1;
}

int main_control::set_goto_location(goal_point *gp)
{
    pthread_mutex_lock(&goto_location_lock);
    memcpy(&goto_location, gp, sizeof(goto_location));
    pthread_mutex_unlock(&goto_location_lock);
    
    return 1;
}

int main_control::clock()
{
 
    switch(current_state) 
    {

        /***********************************************************************************
         *      In idle state the vehicle sits with the motors off and waiting for a command.
         *              NOTE: The idle state does NOT issue a motor stop!
         **********************************************************************************/
        case main_fsm_idle:
            // If an error is found of an abort is issued manually => ABORT
            if( main_error == main_message || main_abort == main_message ) 
    			next_state = main_fsm_abort;
            // If a new way point is issued => GOTO & HOLD
            else if( main_goto == main_message )             
                // goto position is set in goto_callback
                // next state GOTO HOLD
                next_state = main_fsm_goto_hold;
            
            // If the start script is issued by a RUN command
            else if( main_run == main_message )
                next_state = main_fsm_run;            
                
            // Stay here
            else 
                next_state = main_fsm_idle;
            break;

        /***********************************************************************************
         *      In this state we are driving the vehicle to the next way point.
         *              If we reach the way point we keep it here.
         *              If an error occur or we receive abort we => ABORT.
         *              If we receive a manual stop or a timeout occur => IDLE
         **********************************************************************************/

        case main_fsm_goto_hold:
            // If an error occurs or a manual abort
            if( main_error == main_message || main_abort == main_message )
                next_state = main_fsm_abort;
            
            // If manual stop or timeout
/*            else if( main_stop == main_message || (timestamp_now() - state->goto_start_time > state->goto_location.timeout )) 
            {
                // Stop the motors
                stop_motors(state);

                // next state IDLE
                next_state = main_fsm_idle;
            }
            // We are waiting for the way point to be reached by the lower lever controller
            //              once reached. The controller should keep position.
            //      If a new way point is issued this is passed on to the controller instead.
            //              We are currently setting the way point at each iteration. We can perform
            //              tests so if current way point is same as previous it is not reset (this
            //              depends on the implementation of the low level controller and what that
            //              is happy with).
            else 
            {
                // issue goto command to controller
                send_goto( state );

                // We have reached our goal and need to keep this position
                if( state->goal_complete ) {
                        // ignore for now but we can command low level control to hold position
                    }

                    // stay here
                    next_state = main_fsm_goto_hold;

            }
*/            
            next_state = main_fsm_goto_hold;
            break;

            /***********************************************************************************
             *      In this state we are waiting for the perl script to finish. But if an error
             *              occur we can abort or stop and go to a new way point
             **********************************************************************************/
            case main_fsm_run:

                // If an error occurs or a manual abort
                if( main_error == main_message || main_abort == main_message ) 
                    // next state ABORT
                    next_state = main_fsm_abort;

                // If the perl script is done or a manual stop
                else if( main_done == main_message || main_stop == main_message ) 
                {
                    // stop motors
//                    stop_motors(state);

                    // set next state IDLE
                    next_state = main_fsm_idle;
                    // script will be aborted on next status message
                }

                // If a new way point is given
                else if( main_goto == main_message ) 
                    // next state GOTO HOLD
                    next_state = main_fsm_goto_hold;

                // Stay here
                else 
                            next_state = main_fsm_run;
                
                break;

 			/***********************************************************************************
             *      Set the way point to surface at current X,Y => GOTO HOLD
             **********************************************************************************/
            case main_fsm_abort:
                        // Set way point to current X,Y but at the SURFACE
                        // Set next state GOTO HOLD
                        next_state = main_fsm_goto_hold;
                        break;

            /***********************************************************************************
             *      This should NOT happen
             **********************************************************************************/
            default:
                printf( "ERROR: default state should not happen :( \n" );
                next_state = main_fsm_abort;
                break;
    };

    /***********************************************************************************
     *      Change state
     **********************************************************************************/
    pthread_mutex_lock(&state_lock);
    if(next_state != current_state)      
    {  
        cout << "Current state: " << (char *)get_current_state() << "  ";
        current_state = next_state;
        cout << "New state: " << (char *)get_current_state() << endl;
    }
    pthread_mutex_unlock(&state_lock);
    
    return 0;
}



