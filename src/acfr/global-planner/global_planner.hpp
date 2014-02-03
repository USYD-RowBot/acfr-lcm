#include <signal.h>

#include <lcm/lcm-cpp.hpp>
#include "perls-lcmtypes++/acfrlcm/auv_path_response_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_path_command_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_global_planner_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_global_planner_state_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_camera_trigger_t.hpp"
#include "perls-common/timestamp.h"

#include "mission.hpp"
#include "mission_command.hpp"

#ifndef GlobalPlanner_H
#define GlobalPlanner_H

// mission FSM states
typedef enum
{
    globalPlannerFsmIdle,
    globalPlannerFsmRun,
    globalPlannerFsmAbort,
    globalPlannerFsmPause,
    globalPlannerFsmDone,
    globalPlannerFsmFault
} GlobalPlannerStateT;
    

typedef enum
{
    globalPlannerRun,
    globalPlannerAbort,
    globalPlannerStop,
    globalPlannerPause,
    globalPlannerResume,
    globalPlannerIdle
} globalPlannerMessageT;

class GlobalPlanner
{
    public:
        GlobalPlanner();
        ~GlobalPlanner();
        GlobalPlannerStateT getCurrentState();
        string  getCurrentStateString();
        
        bool areWeThereYet;
        double distanceToGoal;
        void set_filename(string filename);

        int clock();
        int process();
        
        Mission mis;
        globalPlannerMessageT globalPlannerMessage;
    private:
        
        lcm::LCM lcm;
        
        
        GlobalPlannerStateT currentState;
        GlobalPlannerStateT nextState;
        
        // mission stuff
        list<waypoint>::iterator currPoint;
        int sendLeg();
        int sendCommands(list<MissionCommand> &commands);
        int64_t legStartTime;        

	// command settings
	acfrlcm::auv_camera_trigger_t cameraTriggerMsg;
};

#endif
