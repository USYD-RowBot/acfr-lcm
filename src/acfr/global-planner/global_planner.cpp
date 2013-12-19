#include "global_planner.hpp"

// Globals are bad mmmmk
int mainExit;

void onPathResponse(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
		const acfrlcm::auv_path_response_t *pr, GlobalPlanner* gp) {
	gp->areWeThereYet = pr->are_we_there_yet;
	gp->distanceToGoal = pr->distance;

	// clock the FSM
	gp->clock();
}

void onGlobalPlannerCommand(const lcm::ReceiveBuffer* rbuf,
		const std::string& channel, const acfrlcm::auv_global_planner_t *gm,
		GlobalPlanner* gp) {
	// we just got a message from the task planner
	switch (gm->command) {
	case acfrlcm::auv_global_planner_t::LOAD:
		// load a mission file and run said mission
		if (!gp->mis.load(gm->str)) {
			cerr << "Could not load mission file " << gm->str << endl;
			gp->globalPlannerMessage = globalPlannerStop;
		} else
			gp->globalPlannerMessage = globalPlannerRun;
		gp->mis.dumpMatlab(
                "matlab_plot.m");
		break;

	case acfrlcm::auv_global_planner_t::RESUME:
		// Resume the mission
		break;

	case acfrlcm::auv_global_planner_t::PAUSE:
		// Pause the current mission
		break;

	case acfrlcm::auv_global_planner_t::ABORT:
		// Abort the curent mission
		gp->globalPlannerMessage = globalPlannerAbort;
		break;

	case acfrlcm::auv_global_planner_t::STOP:
		// Stop the currently running mission
		gp->globalPlannerMessage = globalPlannerStop;
		break;

	case acfrlcm::auv_global_planner_t::GOTO:
		break;
	case acfrlcm::auv_global_planner_t::LEG:
		break;
	case acfrlcm::auv_global_planner_t::GRID:
		break;
	case acfrlcm::auv_global_planner_t::SPIRAL:
		break;
	case acfrlcm::auv_global_planner_t::ZAMBONIE:
		break;
	}

	gp->clock();
}

GlobalPlanner::GlobalPlanner() :
		globalPlannerMessage(globalPlannerIdle) {

	// subscribe to the relevant LCM messages
	lcm.subscribeFunction("TASK_PLANNER_COMMAND", onGlobalPlannerCommand, this);
	lcm.subscribeFunction("PATH_RESPONSE", onPathResponse, this);
}

GlobalPlanner::~GlobalPlanner() {
}

string GlobalPlanner::getCurrentStateString() {
	string GlobalPlannerStateStr[] = { "idle", "run", "abort", "pause", "done",
			"fault" };
	return GlobalPlannerStateStr[currentState];
}

GlobalPlannerStateT GlobalPlanner::getCurrentState() {
	return currentState;
}

int GlobalPlanner::clock() {
	switch (currentState) {

	// We only leave the idle state by going to the run state
	case globalPlannerFsmIdle:
		if (globalPlannerMessage == globalPlannerRun) {
			// set the start point
			currPoint = mis.waypoints.begin();
			nextState = globalPlannerFsmRun;
			cout << "globalplannerfsmidle" << endl;
			sendLeg();
		} else
			nextState = globalPlannerFsmIdle;

		break;

	case globalPlannerFsmRun:
		if (globalPlannerMessage == globalPlannerAbort)
			nextState = globalPlannerFsmAbort;
		else if (globalPlannerMessage == globalPlannerStop)
			nextState = globalPlannerFsmIdle;
		else if (globalPlannerMessage == globalPlannerPause)
			nextState = globalPlannerFsmPause;
		else {
			// check to see if we have reached our destination or we have hit the timeout,
			// if so we can feed the next leg to the path planner
			if ((areWeThereYet) // || distanceToGoal < 1.0)
					|| ((timestamp_now() - legStartTime)
							> (*currPoint).timeout * 1e6)) {
				if( areWeThereYet ) {
					cout << timestamp_now() << " We are there!" << endl;
				}
				if ((timestamp_now() - legStartTime)
						> (*currPoint).timeout * 1e6) {
					cout << "currPoint timed out. timeout="
							<< (*currPoint).timeout * 1e6 << ", curr="
							<< (timestamp_now() - legStartTime) << endl;
				}

				// load the next point and send it along, that is if we are not
				// at the end of the list
				list<waypoint>::iterator i = currPoint;
				i++;
				if (i == mis.waypoints.end())
					nextState = globalPlannerFsmDone;
				else {
					currPoint++;
					sendLeg();
					cout << "globalplannerfsmrun" << endl;
					nextState = globalPlannerFsmRun;
				}
			}
		}
		break;

	case globalPlannerFsmPause:
		if (globalPlannerMessage == globalPlannerAbort)
			nextState = globalPlannerFsmAbort;
		else if (globalPlannerMessage == globalPlannerStop)
			nextState = globalPlannerFsmIdle;
		else if (globalPlannerMessage == globalPlannerResume)
			nextState = globalPlannerFsmRun;
		else
			nextState = globalPlannerFsmPause;

	case globalPlannerFsmAbort:
		// We end up here by an external abort message
		// we will signal the main control layer to go to abort
		// then go into idle
		nextState = globalPlannerFsmAbort;
		break;

	case globalPlannerFsmDone:
		// the mission is complete, go back to idle
		// send a done message up a layer to the main controller
		nextState = globalPlannerFsmIdle;
		break;

	case globalPlannerFsmFault:
		// with any luck we will never end up here, the state machine will need an external
		// signal to be able to leave this state, to be implemented
		nextState = globalPlannerFsmFault;
		break;
	}

	if (nextState != currentState) {
		cout << timestamp_now() << " Current state: " << getCurrentStateString() << "  ";
		currentState = nextState;
		cout <<  " New state: " << getCurrentStateString() << endl;
	}
	globalPlannerMessage = globalPlannerIdle;
	return 0;
}

int GlobalPlanner::sendLeg() {
	// we need to check that both the points we have are goal points
	// if they are not then we need to send the commands in the command
	// point and increment the indexes appropriately

	while ((*currPoint).goalType == COMMAND) {
		sendCommands((*currPoint).commands);
		currPoint++;
	}

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
	if (ii == mis.waypoints.end()) {
		waypoint_yaw = (*currPoint).pose.getYawRad();
		cout << "Compare yaw commands (final)"
				<< (*currPoint).pose.getYawRad() * 180 / M_PI << endl;
	} else {
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
	// Put together the LEG message for the path planner
	acfrlcm::auv_path_command_t pc;
	pc.utime = timestamp_now();
	pc.waypoint[0] = (*currPoint).pose.getX();
	pc.waypoint[1] = (*currPoint).pose.getY();
	pc.waypoint[2] = (*currPoint).pose.getZ();
	pc.waypoint[3] = (*currPoint).pose.getRollRad();
	pc.waypoint[4] = (*currPoint).pose.getPitchRad();
	pc.waypoint[5] = waypoint_yaw;
	pc.waypoint[6] = (*currPoint).velocity[0];

	pc.depth_mode = (*currPoint).depthMode;

	cout << timestamp_now() << " Sending way point " << (*currPoint).id << ": "
			<< (*currPoint).pose.getX() << ", " << (*currPoint).pose.getY()
			<< ", " << (*currPoint).pose.getZ() << endl;

	// Send the message
	lcm.publish("PATH_COMMAND", &pc);
	legStartTime = timestamp_now();

	return 1;
}

// Send commands to instruments
int GlobalPlanner::sendCommands(list<MissionCommand> &commands) {
	return 1;
}

int GlobalPlanner::process() {
	int fd = lcm.getFileno();
	fd_set rfds;
	while (!mainExit) {
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

void signalHandler(int sig) {
	mainExit = 1;
}

int main(int argc, char **argv) {
	// install the signal handler
	mainExit = 0;
	signal(SIGINT, signalHandler);

	// create a global planner object and let it do its thing
	GlobalPlanner *gp = new GlobalPlanner;

	gp->process();

	delete gp;

	return 0;
}
