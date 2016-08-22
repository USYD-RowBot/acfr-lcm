#include "task_planner.hpp"

int mainExit;

// LCM handlers
void onTaskPlannerResponse(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const acfrlcm::auv_global_planner_response_t *response, TaskPlanner* tp) {
    tp->taskPlannerResponse(response);
}

void onTaskCommand(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const acfrlcm::auv_global_planner_t *message, TaskPlanner* tp)
{
    tp->taskCommand(message);
}

void onAcfrNav(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const acfrlcm::auv_acfr_nav_t *nav, TaskPlanner* tp)
{
    tp->updateNav(nav);
}


TaskPlanner::TaskPlanner()
{
    
    currentState =taskPlannerFsmIdle;
    nextState = taskPlannerFsmIdle;
    
    // install the LCM handlers we need
    lcm.subscribeFunction("TASK_PLANNER_RESPONSE", onTaskPlannerResponse, this);
    lcm.subscribeFunction("AUV_COMMAND", onTaskCommand, this);
    lcm.subscribeFunction("ACFR_NAV.*", onAcfrNav, this);
    
    inAbort = false;
}

TaskPlanner::~TaskPlanner()
{
}

string TaskPlanner::getCurrentStateString()
{
    string mainStateStr[] = {"idle", "run", "abort", "goto_hold"};
    return mainStateStr[currentState];
}

taskPlannerStateT TaskPlanner::getCurrentState()
{
    return currentState;
}

int TaskPlanner::taskCommand(const acfrlcm::auv_global_planner_t *m) {
    // We have received an external message, lets decode it and
    // decide what to do
    
    if((m->command == acfrlcm::auv_global_planner_t::LOAD) ||
        (m->command == acfrlcm::auv_global_planner_t::PAUSE) ||
        (m->command == acfrlcm::auv_global_planner_t::RESUME) ||
        (m->command == acfrlcm::auv_global_planner_t::GOTO) ||
        (m->command == acfrlcm::auv_global_planner_t::LEG) ||
        (m->command == acfrlcm::auv_global_planner_t::GRID) ||
        (m->command == acfrlcm::auv_global_planner_t::SPIRAL) ||
        (m->command == acfrlcm::auv_global_planner_t::ZAMBONIE)) {
        
        memcpy(&tpc, m, sizeof(acfrlcm::auv_global_planner_t));
        taskPlannerMessage = taskPlannerRun;
    }
    else if(m->command == acfrlcm::auv_global_planner_t::ABORT)
        taskPlannerMessage = taskPlannerAbort;
    else if(m->command == acfrlcm::auv_global_planner_t::STOP)
        taskPlannerMessage = taskPlannerStop;
    else
        taskPlannerMessage = taskPlannerNone;
    
    clock();
    return 1;
}

int TaskPlanner::taskPlannerResponse(const acfrlcm::auv_global_planner_response_t *response) {

    clock();
    return 1;
}

int TaskPlanner::updateNav(const acfrlcm::auv_acfr_nav_t *nav) {
    navX = nav->x;
    navY = nav->y;
    navZ = nav->depth;
    navH = nav->heading;
    return 1;
}

int TaskPlanner::sendAbort() {
// Generate a spiral abort path to the surface
    acfrlcm::auv_global_planner_t ac;
    memset(&ac, 0, sizeof(acfrlcm::auv_global_planner_t));
    ac.utime = timestamp_now();
    ac.command = acfrlcm::auv_global_planner_t::SPIRAL;
    ac.point1_x = navX + (sin(M_PI/2 - navH) * 5);
    ac.point1_y = navY + (cos(M_PI/2 - navH) * 5);
    ac.point1_z = navZ;
    ac.var_d[0] = 5;
    ac.var_d[1] = -navZ;
    lcm.publish("TASK_PLANNER_COMMAND", &tpc);
    
    // set the in abort flag;
    inAbort = true;
    return 1;
}

int TaskPlanner::clock()
{
 
    switch(currentState) 
    {

        /***********************************************************************************
         *      In idle state the vehicle sits with the motors off and waiting for a command.
         *              NOTE: The idle state does NOT issue a motor stop!
         **********************************************************************************/
        case taskPlannerFsmIdle:
            // If an error is found of an abort is issued manually => ABORT
            if((taskPlannerMessage == taskPlannerError) || (taskPlannerMessage == taskPlannerAbort)) 
    			nextState = taskPlannerFsmAbort;
            // If a new way point is issued => GOTO & HOLD
            // If the start script is issued by a RUN command
            
            else if(taskPlannerMessage == taskPlannerRun)
            {
                // we will have check the command when it arrived at this layer
                // so all we do now is send it down a layer to the global planner
                lcm.publish("TASK_PLANNER_COMMAND", &tpc);
                nextState = taskPlannerFsmRun;            
            }    
            // Stay here
            else 
                nextState = taskPlannerFsmIdle;
            break;

            /***********************************************************************************
             *      In this state we are waiting for the perl script to finish. But if an error
             *              occur we can abort or stop and go to a new way point
             **********************************************************************************/
            case taskPlannerFsmRun:

                // If an error occurs or a manual abort
                if((taskPlannerMessage == taskPlannerError) || (taskPlannerMessage == taskPlannerAbort)) 
    			    nextState = taskPlannerFsmAbort;
            
                
                else if((taskPlannerMessage == taskPlannerDone) || (taskPlannerMessage == taskPlannerStop)) {
                    // set next state IDLE
                    nextState = taskPlannerFsmIdle;
                }
                else 
                    nextState = taskPlannerFsmRun;
                
                break;

 			/***********************************************************************************
             *      Set the way point to surface at current X,Y => GOTO HOLD
             **********************************************************************************/
            case taskPlannerFsmAbort:
                if(!inAbort)
                    sendAbort();
                 
                nextState = taskPlannerFsmAbort;
                break;

            /***********************************************************************************
             *      This should NOT happen
             **********************************************************************************/
            case taskPlannerFsmFault:
                break;
            default:
                printf( "ERROR: default state should not happen :( \n" );
                nextState = taskPlannerFsmAbort;
                break;
    };

    /***********************************************************************************
     *      Change state
     **********************************************************************************/
    if(nextState != currentState)      
    {  
        cout << "Current state: " << getCurrentStateString() << "  ";
        currentState = nextState;
        cout << "New state: " << getCurrentStateString() << endl;
    }
    
    return 0;
}

int TaskPlanner::process()
{
    int fd = lcm.getFileno();
    fd_set rfds;
    while(!mainExit)
    {
        FD_ZERO (&rfds);
        FD_SET (fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int ret = select (fd + 1, &rfds, NULL, NULL, &timeout);
        if(ret > 0)
            lcm.handle();
    }
    
    return 1;
}


 
void signalHandler(int sig) {
    mainExit = 1;
} 
 
int main(int argc, char **argv) {
    // install the signal handler
    mainExit = 0;
    signal(SIGINT, signalHandler);

    // create a global planner object and let it do its thing
    TaskPlanner *tp = new TaskPlanner;
    
    
    tp->process();
    
    delete tp;
    
    return 0;
}

