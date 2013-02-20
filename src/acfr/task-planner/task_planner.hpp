#include <lcm/lcm-cpp.hpp>
#include <string>
#include <iostream>
#include <math.h>
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_global_planner_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_global_planner_response_t.hpp"



#ifndef TaskPlanner_HPP
#define TaskPlanner_HPP

using namespace std;
using namespace acfrlcm;



// main FSM states
typedef enum
{
    taskPlannerFsmIdle,
    taskPlannerFsmRun,
    taskPlannerFsmAbort,
    taskPlannerFsmFault
} taskPlannerStateT;


// main mission messages
typedef enum
{
    taskPlannerRun,
    taskPlannerAbort,
    taskPlannerStop,
    taskPlannerError,
    taskPlannerDone,
    taskPlannerNone
} taskPlannerMessageT;

class TaskPlanner
{  
    public:
        TaskPlanner();
        ~TaskPlanner();
        int process();
        int taskPlannerResponse(const acfrlcm::auv_global_planner_response_t *message);
        int updateNav(const acfrlcm::auv_acfr_nav_t *nav);
        int taskCommand(const acfrlcm::auv_global_planner_t *message);
        
    private:      
        
          
        
        int sendAbort();
        
        taskPlannerStateT currentState;
        taskPlannerStateT nextState;
        
        taskPlannerMessageT taskPlannerMessage;
        acfrlcm::auv_global_planner_t tpc;
        int clock();
        lcm::LCM lcm;
        taskPlannerStateT getCurrentState();
        string  getCurrentStateString();
        
        // for doing aborts we need to know out current x and y locations
        double navX, navY, navZ, navH;
        
        // Misc flags
        bool inAbort;
        
};    

#endif
