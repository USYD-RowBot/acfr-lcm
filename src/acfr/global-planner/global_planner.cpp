#include <unistd.h>
#include "global_planner.hpp"

using namespace std;

// Globals are bad mmmmk
int mainExit;
string vehicle_name = "DEFAULT";

void onPathResponse(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
		const acfrlcm::auv_path_response_t *pr, GlobalPlanner* gp)
{
	gp->areWeThereYet = pr->are_we_there_yet;
	gp->distanceToGoal = pr->distance;

	// clock the FSM
	gp->clock();
}

void onGlobalPlannerCommand(const lcm::ReceiveBuffer* rbuf,
		const std::string& channel, const acfrlcm::auv_global_planner_t *gm,
		GlobalPlanner* gp)
{
	cout << "\nReceived new command\n" << endl;
	fstream * fs = NULL;
	// we just got a message from the task planner
	switch (gm->command)
	{
	case acfrlcm::auv_global_planner_t::LOAD:
		// Check if file exists
		fs = new fstream(gm->str.c_str(), ios::in);
		// load a mission file and run said mission
		if (!fs || !fs->good() ) {
			cerr << "Invalid mission file" << gm->str
					<< ". Stopping execution" << endl;
			gp->globalPlannerMessage = globalPlannerStop;
		}
		else if( !gp->loadNewMissionFile(gm->str) )
		{
			cerr << "Could not load mission file " << gm->str
					<< ". Stopping execution" << endl;
			gp->globalPlannerMessage = globalPlannerStop;
		}
		else
		{
			cout << "\tLoaded new mission" << endl;
			// set the start point
			//if( gp->getCurrentState() == globalPlannerFsmRun ) {
			//	gp->globalPlannerMessage = globalPlannerStop;
			//	gp->clock();
			//}
			gp->globalPlannerMessage = globalPlannerRun;
			gp->mis.dumpMatlab("matlab_plot.m");
		}
		break;

	case acfrlcm::auv_global_planner_t::RESUME:
		// Resume the mission
		gp->globalPlannerMessage = globalPlannerResume;
		break;

	case acfrlcm::auv_global_planner_t::PAUSE:
		// Pause the current mission
		gp->globalPlannerMessage = globalPlannerPause;
		break;

	case acfrlcm::auv_global_planner_t::ABORT:
		// Abort the curent mission
		gp->globalPlannerMessage = globalPlannerAbort;
		break;

	case acfrlcm::auv_global_planner_t::STOP:
		// Stop the currently running mission
		gp->globalPlannerMessage = globalPlannerStop;
		break;

	case acfrlcm::auv_global_planner_t::RESET:
		// Stop the currently running mission
		gp->globalPlannerMessage = globalPlannerReset;
		break;

	case acfrlcm::auv_global_planner_t::SKIP:
		// Skip the current waypoint
		gp->skipWaypoint = true;
		break;

	// We have received a task command. Parse the xml mission string 
	case acfrlcm::auv_global_planner_t::GOTO:
	case acfrlcm::auv_global_planner_t::LEG:
	case acfrlcm::auv_global_planner_t::GRID:
	case acfrlcm::auv_global_planner_t::SPIRAL:
	case acfrlcm::auv_global_planner_t::ZAMBONIE:
		if( !gp->loadNewMissionString(gm->str) )
		{
			cerr << "Could not load mission file " << gm->str
					<< ". Stopping execution" << endl;
			gp->globalPlannerMessage = globalPlannerStop;
		}
		else
		{
			cout << "\tLoaded new mission received as a task command" << endl;
			//if( gp->getCurrentState() == globalPlannerFsmRun ) {
			//	gp->globalPlannerMessage = globalPlannerStop;
			//	gp->clock();
			//}
			gp->globalPlannerMessage = globalPlannerRun;
		}	
                break;
		
	}

	gp->clock();
}

GlobalPlanner::GlobalPlanner() :
		skipWaypoint(false), areWeThereYet(false), distanceToGoal(-1),
		globalPlannerMessage(globalPlannerIdle)
{

	// subscribe to the relevant LCM messages
	lcm.subscribeFunction("TASK_PLANNER_COMMAND."+vehicle_name, onGlobalPlannerCommand, this);
	lcm.subscribeFunction("PATH_RESPONSE."+vehicle_name, onPathResponse, this);

	// set default values
	cameraTriggerMsg.command = acfrlcm::auv_camera_trigger_t::SET_STATE;
	cameraTriggerMsg.freq = 1;
	cameraTriggerMsg.pulseWidthUs = -1;
	cameraTriggerMsg.strobeDelayUs = 1;

	cout << endl << endl << endl << "GlobalPlanner started" << endl;
}

GlobalPlanner::~GlobalPlanner()
{
}

string GlobalPlanner::getCurrentStateString()
{
	string GlobalPlannerStateStr[] =
	{ "idle", "run", "abort", "pause", "done", "fault" };
	return GlobalPlannerStateStr[currentState];
}

GlobalPlannerStateT GlobalPlanner::getCurrentState()
{
	return currentState;
}

int GlobalPlanner::clock()
{
	switch (currentState)
	{

	// We only leave the idle state by going to the run state
	case globalPlannerFsmIdle:
		if (globalPlannerMessage == globalPlannerRun)
		{
			// set the start point
			//currPoint = mis.waypoints.begin();
			nextState = globalPlannerFsmRun;
			cout << "globalplannerfsmidle" << endl;
			//sendLeg();
		}
		else
			nextState = globalPlannerFsmIdle;

		break;

	case globalPlannerFsmRun:
		if (globalPlannerMessage == globalPlannerAbort)
			nextState = globalPlannerFsmAbort;
		else if (globalPlannerMessage == globalPlannerStop)
			nextState = globalPlannerFsmIdle;
		else if (globalPlannerMessage == globalPlannerPause) {
			cout << "Paused..." << endl;
			nextState = globalPlannerFsmPause;
		}
		else
		{
			// check to see if we have reached our destination or we have hit the timeout,
			// if so we can feed the next leg to the path planner
			if ( areWeThereYet || skipWaypoint ||
				 ((timestamp_now() - legStartTime) > (*currPoint).timeout * 1e6) )
			{
				if (areWeThereYet)
				{
					cout << timestamp_now() << " We are there!" << endl;
				}
				if ((timestamp_now() - legStartTime) > (*currPoint).timeout * 1e6)
				{
					cout << "currPoint timed out. timeout="
							<< (*currPoint).timeout * 1e6 << ", curr="
							<< (timestamp_now() - legStartTime) << endl;
				}
				if( skipWaypoint ) {
					cout << "Skipping waypoint at requested" << endl;
					skipWaypoint = false;
				}

				// load the next point and send it along, that is if we are not
				// at the end of the list
				currPoint++;
				if (currPoint == mis.waypoints.end())
					nextState = globalPlannerFsmDone;
				else
				{
					sendLeg();
					cout << "globalplannerfsmrun" << endl;
					nextState = globalPlannerFsmRun;
				}
			}
		}
		break;

	case globalPlannerFsmPause:
		if (globalPlannerMessage == globalPlannerAbort) {
			nextState = globalPlannerFsmAbort;
		}
		else if (globalPlannerMessage == globalPlannerStop) {
			nextState = globalPlannerFsmIdle;
		}
		else if (globalPlannerMessage == globalPlannerResume) {
			nextState = globalPlannerFsmRun;
		}
		else
			nextState = globalPlannerFsmPause;
		break;

	case globalPlannerFsmAbort:
		// We end up here by an external abort message
		// we will signal the main control layer to go to abort
		// then go into idle
		nextState = globalPlannerFsmAbort;
		if( globalPlannerMessage == globalPlannerReset ) {
			nextState = globalPlannerFsmIdle;
		}
		break;

	case globalPlannerFsmDone:
		// the mission is complete, go back to idle
		// TODO: send a done message up a layer to the main controller
		nextState = globalPlannerFsmIdle;
		break;

	case globalPlannerFsmFault:
		// with any luck we will never end up here, the state machine will need an external
		// signal to be able to leave this state, to be implemented
		nextState = globalPlannerFsmFault;
		break;
	default:
		nextState = globalPlannerFsmAbort;
		break;
	}

	if (nextState != currentState)
	{
		cout << timestamp_now() << " Current state: " << getCurrentStateString()
				<< "  ";
		currentState = nextState;
		cout << " New state: " << getCurrentStateString() << endl;

		// broadcast the state change
		acfrlcm::auv_global_planner_state_t gpState;
		gpState.utime = timestamp_now();
		switch (nextState)
		{
		case globalPlannerFsmAbort:
			gpState.state = acfrlcm::auv_global_planner_state_t::ABORT;
			break;
		case globalPlannerFsmIdle:
			gpState.state = acfrlcm::auv_global_planner_state_t::IDLE;
			break;
		case globalPlannerFsmRun:
			gpState.state = acfrlcm::auv_global_planner_state_t::RUN;
			break;
		case globalPlannerFsmDone:
			gpState.state = acfrlcm::auv_global_planner_state_t::DONE;
			break;
		case globalPlannerFsmPause:
			gpState.state = acfrlcm::auv_global_planner_state_t::PAUSE;
			break;
		case globalPlannerFsmFault:
			gpState.state = acfrlcm::auv_global_planner_state_t::FAULT;
			break;
		}

		// Send the global state change message
		cout << "Publishing new global state: " << (int) (gpState.state)
				<< endl;
		lcm.publish("GLOBAL_STATE."+vehicle_name, &gpState);
	}
	globalPlannerMessage = globalPlannerIdle;
	return 0;
}

bool GlobalPlanner::loadNewMissionFile(string filename)
{
	bool ret;
	if ((ret = mis.load(filename)) == true)
	{
		currPoint = mis.waypoints.begin();
        	sendLeg();
	}
	return ret;
}

bool GlobalPlanner::loadNewMissionString(string mission_string)
{	bool ret;
	if ((ret = mis.parseMissionString(mission_string)) == true)
	{
		currPoint = mis.waypoints.begin();
        	sendLeg();
	}
	return ret;
}


int GlobalPlanner::sendLeg()
{
	// we need to check that both the points we have are goal points
	// if they are not then we need to send the commands in the command
	// point and increment the indexes appropriately
/*
	while ((*currPoint).goalType == COMMAND)
	{
		sendCommands((*currPoint).commands);
		currPoint++;
	}
*/
	// execute the commands inside the point
	sendCommands((*currPoint).commands);

	// reset the flag
	areWeThereYet = 0;

	// Account for next waypoint yaw for this waypoint (take average between the
	// 	desired yaw and the following leg angle)
#if 0
	list<waypoint>::iterator ii = currPoint;
	ii++;
	double waypoint_yaw, yaw1, yaw2;
	if (ii == mis.waypoints.end())
	{
		waypoint_yaw = (*currPoint).pose.getYawRad();
		cout << "Compare yaw commands (final)"
		<< (*currPoint).pose.getYawRad() * 180 / M_PI << endl;
	}
	else
	{
		yaw1 = (*currPoint).pose.getYawRad();
		if (yaw1 > 2 * M_PI
		)
		yaw1 = yaw1 - 2 * M_PI;
		else if (yaw1 < 0)
		yaw1 = yaw1 + 2 * M_PI;

		yaw2 = atan2((*ii).pose.getY() - (*currPoint).pose.getY(),
				(*ii).pose.getX() - (*currPoint).pose.getX()) + 2 * M_PI;
		if (yaw2 > 2 * M_PI
		)
		yaw2 = yaw2 - 2 * M_PI;
		else if (yaw2 < 0)
		yaw2 = yaw2 + 2 * M_PI;

		if (yaw2 - yaw1 > M_PI
		)
		yaw2 = yaw2 - 2 * M_PI;
		else if (yaw1 - yaw2 > M_PI
		)
		yaw1 = yaw1 - 2 * M_PI;

		waypoint_yaw = (yaw1 + yaw2) / 2;
		if (waypoint_yaw > 2 * M_PI
		)
		waypoint_yaw = waypoint_yaw - 2 * M_PI;
		else if (waypoint_yaw < 0)
		waypoint_yaw = waypoint_yaw + 2 * M_PI;

		//cout << "Compare yaw commands (lookahead)" << (*currPoint).pose.getYawRad()*180/M_PI << " " << yaw2*180/M_PI << " " << waypoint_yaw*180/M_PI << endl;

	}
#else
	double waypoint_yaw = (*currPoint).pose.getYawRad();
#endif
	// Put together the Waypoint message for the path planner
	acfrlcm::auv_path_command_t pc;
	pc.utime = timestamp_now();
	pc.goal_id = (*currPoint).id;
	pc.waypoint[0] = (*currPoint).pose.getX();
	pc.waypoint[1] = (*currPoint).pose.getY();
	pc.waypoint[2] = (*currPoint).pose.getZ();
	pc.waypoint[3] = (*currPoint).pose.getRollRad();
	pc.waypoint[4] = (*currPoint).pose.getPitchRad();
	pc.waypoint[5] = waypoint_yaw;
	pc.waypoint[6] = (*currPoint).velocity[0];

	pc.depth_mode = (*currPoint).depthMode;

	cout << timestamp_now()
			<< fixed << setprecision(2)
			<< " Sending way point "
			<< (*currPoint).id << " of " << mis.waypoints.size()
			<< "(" << (*currPoint).id*100/mis.waypoints.size() << "%) : "
			<< (*currPoint).pose.getX() << ", " << (*currPoint).pose.getY()
			<< ", " << (*currPoint).pose.getZ() << endl;

	// Send the message
	lcm.publish("PATH_COMMAND."+vehicle_name, &pc);
	legStartTime = timestamp_now();

	return 1;
}

// Send commands to instruments
int GlobalPlanner::sendCommands(list<MissionCommand> &commands)
{
	for (std::list<MissionCommand>::iterator itr = commands.begin();
			itr != commands.end(); itr++)
			{ 
		switch (itr->device)
		{
		case CAMERA:
            memset(&cameraTriggerMsg, 0, sizeof(cameraTriggerMsg));
			cameraTriggerMsg.utime = timestamp_now();
			if (itr->command == CAMERA_FREQ)
			{
				cameraTriggerMsg.command =
						acfrlcm::auv_camera_trigger_t::SET_FREQ;
				cameraTriggerMsg.freq = itr->valueDouble;
			}
			else if (itr->command == CAMERA_WIDTH)
			{
				cameraTriggerMsg.command =
						acfrlcm::auv_camera_trigger_t::SET_WIDTH;
				cameraTriggerMsg.pulseWidthUs = itr->valueDouble;
			}
			else if (itr->command == CAMERA_START)
			{
				cameraTriggerMsg.command =
						acfrlcm::auv_camera_trigger_t::SET_STATE;
				cameraTriggerMsg.enabled = 1;
                cameraTriggerMsg.freq = 1.0;
                cameraTriggerMsg.pulseWidthUs = 10.0;
			}
			else if (itr->command == CAMERA_STOP)
			{
				cameraTriggerMsg.command =
						acfrlcm::auv_camera_trigger_t::SET_STATE;
				cameraTriggerMsg.enabled = 0;
			}
			// Send the trigger command 
			std::cout << "Sending camera trigger: state:"
					<< (int) (cameraTriggerMsg.enabled) << " f:"
					<< (int) (cameraTriggerMsg.freq) << " w:"
					<< (int) (cameraTriggerMsg.pulseWidthUs) << endl;
			lcm.publish("CAMERA_TRIGGER."+vehicle_name, &cameraTriggerMsg);
			break;
		case DVL:
		    memset(&rdiCommandMsg, 0, sizeof(rdiCommandMsg));
			rdiCommandMsg.utime = timestamp_now();
			string cmd;
			if (itr->command == DVL_RANGE)
			{
				rdiCommandMsg.command =
						senlcm::rdi_control_t::RANGE;
				rdiCommandMsg.d = itr->valueDouble;
				cmd = "Range";
			}
			if (itr->command == DVL_PD5)
		    {
				rdiCommandMsg.command =
						senlcm::rdi_control_t::PD5_COUNT;
				rdiCommandMsg.i = itr->valueInt;
				cmd = "PD5 count";
			}
			if (itr->command == DVL_PD0)
			{
				rdiCommandMsg.command =
						senlcm::rdi_control_t::PD0_COUNT;
				rdiCommandMsg.i = itr->valueInt;
				cmd = "PD0 count";
			}
			
			std::cout << "Sending RDI DVL command: "
					<< cmd << " I:"
					<< (int) (rdiCommandMsg.d) << " F:"
					<< (int) (rdiCommandMsg.i) << endl;
            lcm.publish("RDI_CONTROL."+vehicle_name, &rdiCommandMsg);
			break;
		}
	}
	return 1;
}

int GlobalPlanner::process()
{
	int fd = lcm.getFileno();
	fd_set rfds;
	while (!mainExit)
	{
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		struct timeval timeout;
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;
		int ret = select(fd + 1, &rfds, NULL, NULL, &timeout);
		if (ret > 0)
			lcm.handle();
	}

	return 1;
}

void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void
parse_args (int argc, char **argv)
{
    int opt;

    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            vehicle_name = (char*)optarg;
            break;
         }
    }
}

void signalHandler(int sig)
{
	mainExit = 1;
}

int main(int argc, char **argv)
{
        parse_args(argc, argv);
	// install the signal handler
	mainExit = 0;
	signal(SIGINT, signalHandler);

	// create a global planner object and let it do its thing
	GlobalPlanner * gp = new GlobalPlanner();
	gp->process();

	delete gp;
	return 0;
}
