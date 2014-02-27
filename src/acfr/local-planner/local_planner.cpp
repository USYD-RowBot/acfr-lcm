#include "local_planner.hpp"

// get the index to the minimum value in an array
int getMin(double array[], int size)
{
	int index = 0;
	for (int i = 0; i < size; i++)
	{
		if (array[i] < array[index])
		{
			index = i;
		}
	}

	return index;

}

// Exit handler, I swear this is the only global
int mainExit;
void signalHandler(int sig)
{
	mainExit = 1;
}

// Nav callback
void onNavLCM(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
		const acfrlcm::auv_acfr_nav_t *nav, LocalPlanner *lp)
{
	lp->onNav(nav);
}

// Path command callback
void onPathCommandLCM(const lcm::ReceiveBuffer* rbuf,
		const std::string& channel, const acfrlcm::auv_path_command_t *pc,
		LocalPlanner *lp)
{
	lp->onPathCommand(pc);
}

// Global State command callback
void onGlobalStateLCM(const lcm::ReceiveBuffer* rbuf,
		const std::string& channel,
		const acfrlcm::auv_global_planner_state_t *gpState, LocalPlanner *lp)
{
	lp->onGlobalState(gpState);
}

// heartbeat callback
void recalculate(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
		const perllcm::heartbeat_t *heartbeat, LocalPlanner *lp)
{
	// We haven't reached our goal but don't have any more waypoints.
	// 	Let's recalculate a feasible path.
	// Addition: Make the replanning stage occur at a set period
	//        if( (lp->waypoints.size() == 0) && (lp->getDestReached() == false) && (lp->getNewDest() == true))// ||  (( timestamp_now() - lp->getReplanTime() )/1000000 > 5) ) // if we want faster replanning, or a different number than the heartbeat, this callback needs to be made faster and this variable can be used
	static long count = 0;
	//	double timeSinceReplan = (timestamp_now() - lp->getReplanTime()) / 1000000.;

	if ((lp->getDestReached() == false && lp->getNewDest() == true) &&
	//((lp->waypoints.size() == 0) || (timeSinceReplan > lp->getReplanInterval()))
			(lp->getWaypointTimePassedSec() > lp->getWaypointTimeout()))
	{

		if (lp->getWaypointTimePassedSec() > lp->getWaypointTimeout())
			cout << "Way point timed out: " << lp->getWaypointTimePassedSec()
					<< " > " << lp->getWaypointTimeout() << endl;

		cout << endl << "------------------------" << endl
				<< "RECALCULATING A NEW PATH" << endl
				<< "------------------------" << endl << endl;

		lp->calculateWaypoints();
		lp->resetWaypointTime(timestamp_now());
		lp->resetReplanTime(timestamp_now());
	}
	else if (++count % 10 == 0)
	{
		//		cout << "Time since replanning:" << timeSinceReplan << endl << endl;
	}

	lp->sendResponse();
}

/** *************************************************************************
 *
 * LocalPlanner
 *
 */
LocalPlanner::LocalPlanner() :
		startVel(0), destVel(0), newDest(false), destReached(false),
		depthMode(0), diveMode(0), destID(0), turningRadius(1.0), maxPitch(0),
		lookaheadVelScale(0), maxDistFromLine(0), maxAngleFromLine(0),
		velChangeDist(1.0), defaultLegVel(1.0), waypointTimeout(1e3),
		forwardBound(0.5), sideBound(2.0), distToDestBound(2.0),
		maxAngleWaypointChange(0.0), radiusIncrease(1.0), maxAngle(300),
		wpDropDist(4.0), wpDropAngle(M_PI / 18.), replanInterval(1.5),
		waypointTime(timestamp_now()), replanTime(timestamp_now())
{

	// sunscribe to the required LCM messages
	lcm.subscribeFunction("ACFR_NAV", onNavLCM, this);
	lcm.subscribeFunction("PATH_COMMAND", onPathCommandLCM, this);
	lcm.subscribeFunction("GLOBAL_STATE", onGlobalStateLCM, this);
	lcm.subscribeFunction("HEARTBEAT_5HZ", recalculate, this);

	gpState.state = acfrlcm::auv_global_planner_state_t::IDLE;

	//    pthread_mutex_init(&currPoseLock, NULL);
	//    pthread_mutex_init(&destPoseLock, NULL);
	//    pthread_mutex_init(&waypointsLock, NULL);
	fp.open("/tmp/log_waypoint.txt", ios::out);
	fp_wp.open("/tmp/log_waypoint_now.txt", ios::out);
}

LocalPlanner::~LocalPlanner()
{

	if (fp.is_open())
		fp.close();
	if (fp_wp.is_open())
		fp_wp.close();
	//    pthread_mutex_destroy(&currPoseLock);
	//    pthread_mutex_destroy(&destPoseLock);
	//    pthread_mutex_destroy(&waypointsLock);
}
/**
 * Copy current nav solution and process the waypoints
 */
int LocalPlanner::onNav(const acfrlcm::auv_acfr_nav_t *nav)
{

	currPose.setPosition(nav->x, nav->y, nav->depth);
	currAltitude = nav->altitude;

	// Instead of heading, we make the current pose "heading" actually the slip
	// 	angle (bearing), for control
#if 0
	currPose.setRollPitchYawRad( nav->roll, nav->pitch, nav->heading );
#else
	double bearing = atan2(
			+nav->vy * cos(nav->heading) + nav->vx * sin(nav->heading),
			-nav->vy * sin(nav->heading) + nav->vx * cos(nav->heading));
	while (bearing < -M_PI)
		bearing += 2 * M_PI;
	while (bearing > M_PI)
		bearing -= 2 * M_PI;
	currPose.setRollPitchYawRad(nav->roll, nav->pitch, bearing);
#endif
	currVel = nav->vx, nav->vy, nav->vz;

	// for now only process waypoints or dive commands if we are in
	// RUN mode.  This will need to be modified to take into account
	// an active PAUSE mode.
	if (gpState.state == acfrlcm::auv_global_planner_state_t::RUN)
	{
		processWaypoints();
	}

	return 1;
}

/**
 * We have received a new path command.
 * Let's calcualte a new path.
 */
int LocalPlanner::onPathCommand(const acfrlcm::auv_path_command_t *pc)
{
	bool status = false;
	// Reset destination pose
	destPose.setIdentity();

	// Set destination pose and velocity
	destPose.setPosition(pc->waypoint[0], pc->waypoint[1], pc->waypoint[2]);
	destPose.setRollPitchYawRad(pc->waypoint[3], pc->waypoint[4],
			pc->waypoint[5]);
	destVel = pc->waypoint[6];
	depthMode = pc->depth_mode;
	destID = pc->goal_id;

	cout << timestamp_now() << " Got a new DEST point " << endl << "\t"
			<< "DestPose=" << destPose.getX() << "," << destPose.getY() << ","
			<< destPose.getZ() << " < " << destPose.getYawRad() / M_PI * 180
			<< endl;

	setDestReached(false);
	setNewDest(true);
	if ((status = calculateWaypoints()) == false)
	{
		// TODO: We must do something more clever here!!!!!
		cerr
				<< endl
				<< "----------------------------------------"
				<< "Can't calcualte a feasible path. Let's cruise and see what happens"
				<< "----------------------------------------" << endl << endl;
	}
	resetWaypointTime(timestamp_now());

	return status;
}

#define DIVE_REV_VEL -0.5
//#define DIVE_REV_VEL 0 // for lab testing
#define DIVE_PICTH (10.0 / 180.0 * M_PI)
#define DIVE_DEPTH 0.5

/**
 * Call Dubins path planner to generate a path from current location to the
 * destination waypoint
 */
int LocalPlanner::calculateWaypoints()
{

	// Get a copy of curr and dest pose
	Pose3D currPose = this->currPose;
	Pose3D destPose = this->destPose;
	double currVel = this->currVel[0];

	cout << timestamp_now() << " Calculating new waypoints..." << endl;
	cout << setprecision(2) << "CurrPose=" << currPose.getX() << ","
			<< currPose.getY() << "," << currPose.getZ() << " < "
			<< currPose.getYawRad() / M_PI * 180 << endl;
	cout << setprecision(2) << "DestPose=" << destPose.getX() << ","
			<< destPose.getY() << "," << destPose.getZ() << " < "
			<< destPose.getYawRad() / M_PI * 180 << endl;

	Pose3D destPoseRel = getRelativePose(destPose);
	double relAngle = atan2( destPoseRel.getY(), destPoseRel.getX() );
	cout << "Dest rel: X=" << destPoseRel.getX()
			<< ", Angle=" << relAngle/M_PI*180 << endl;
	vector<Pose3D> waypoints;
	// If the waypoint is just ahead of us no need to use Dubins. We will rely
	//	on the controller to get us there.
	if ( destPoseRel.getX() < 0 ||
			destPoseRel.getX() > 2*turningRadius ||
			fabs(relAngle) > 45./180*M_PI )
	{
		DubinsPath dp;
		dp.setCircleRadius(turningRadius);
		dp.setMaxPitch(maxPitch);
		dp.setWaypointDropDist(wpDropDist);
		dp.setWaypointDropAngleFromDropDist();

		// Don't use current pose but a pose looking a bit ahead. We adjust this
		//	by employing the current velocity
		double lookAheadTime = 0.2; // [s]
		Pose3D lookAheadPose;
		// Using fabs of vel to ensure that even if we are going backwards the
		// lookaheadpose is in front of us
		lookAheadPose.setX(fabs(currVel) * lookAheadTime);
		Pose3D startPose = currPose.compose(lookAheadPose);

		// Try to calculate a feasible Dubins path. If we fail we try
		//  a second time with twice the circle radius
		waypoints = dp.calcPath(startPose, destPose);

		// Todo: is this if statement correct?
		if ((waypoints.size() == 0))
		{
			cerr << "Failed to calculate a feasible path using Dubins path"
					<< endl;
			cerr << "Increasing the turn radius to " << turningRadius * 2
					<< " and trying again" << endl;

				waypoints.push_back(destPose);
				return false;
		}

		cout << "New waypoints calculated using Dubins" << endl;
	}
	else
	{
		cout
				<< "We are close to the new waypoint and use the controller to get us there."
				<< endl;
		waypoints.push_back(destPose);
	}

	// Managed to calculate a path to destination
	this->waypoints = waypoints;

	// Save the start pose and start velocity
	startPose = currPose;

	//setDestReached( false );

	cout << "waypoints = [" << endl;
	if (!fp.is_open())
		fp.open("/tmp/log_waypoint.txt", ios::out | ios::app);
	for (unsigned int i = 0; i < waypoints.size(); i++)
	{
		cout << waypoints.at(i).getX() << ", " << waypoints.at(i).getY() << ", "
				<< waypoints.at(i).getZ() << ";" << endl;
		fp << waypoints.at(i).getX() << " " << waypoints.at(i).getY() << " "
				<< waypoints.at(i).getZ() << endl;

	}
	fp << 0 << " " << 0 << " " << 0 << endl;
	fp.close();
	cout << "];" << endl;

	return true;
}

/**
 * Account for changes in system state
 */
int LocalPlanner::onGlobalState(
		const acfrlcm::auv_global_planner_state_t *gpStateLCM)
{
	cout << "Change of global state to: " << (int) (gpStateLCM->state) << endl;
	gpState = *gpStateLCM;

	/* for the moment, send a STOP command to the low level controller
	 * for transitions into any state but RUN.  This may need to be
	 * modified for more complex PAUSE or ABORT behaviours.
	 */
	if (gpState.state != acfrlcm::auv_global_planner_state_t::RUN)
	{
		// form a STOP message to send
		acfrlcm::auv_control_t cc;
		cc.utime = timestamp_now();

		// The instant we hit a waypoint, stop the motors, until the global
		// 	planner sends a waypoint. This fixes idle behaviour.
		cc.run_mode = acfrlcm::auv_control_t::STOP;
		lcm.publish("AUV_CONTROL", &cc);
	}
	return 1;
}

int LocalPlanner::loadConfig(char *program_name)
{
	BotParam *param = NULL;
	param = bot_param_new_from_server(lcm.getUnderlyingLCM(), 1);
	if (param == NULL
	)
		return 0;

	char rootkey[64];
	char key[128];
	sprintf(rootkey, "acfr.%s", program_name);

	sprintf(key, "%s.turning_radius", rootkey);
	turningRadius = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.maximum_pitch", rootkey);
	maxPitch = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.velocity_change_distance", rootkey);
	velChangeDist = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.default_leg_velocity", rootkey);
	defaultLegVel = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.look_ahead_velocity_scale", rootkey);
	lookaheadVelScale = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.max_dist_from_line", rootkey);
	maxDistFromLine = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.max_angle_from_line", rootkey);
	maxAngleFromLine = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.waypoint_timeout", rootkey);
	waypointTimeout = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.forward_bound", rootkey);
	forwardBound = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.side_bound", rootkey);
	sideBound = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.drop_dist", rootkey);
	wpDropDist = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.dist_to_dest_bound", rootkey);
	distToDestBound = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.max_angle_waypoint_change", rootkey);
	maxAngleWaypointChange = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.radius_increase", rootkey);
	radiusIncrease = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.max_angle", rootkey);
	maxAngle = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.drop_angle", rootkey);
	wpDropAngle = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.replan_interval", rootkey);
	replanInterval = bot_param_get_double_or_fail(param, key);

	return 1;
}

int LocalPlanner::process()
{
	//static void *processLCM(void *u) {
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

/**
 * Look at the next waypoint and
 * 1) remove it from the list if we have reached it or
 * 2) calculate commands for the controller
 */
int LocalPlanner::processWaypoints()
{

	// Nothing to do
	if (waypoints.size() == 0)
	{
		return 0;
	}

	// Get the next waypoint
	Pose3D wp = waypoints.at(0);

	// We have reached the next waypoint
	if (pointWithinBound(wp))
	{
		waypoints.erase(waypoints.begin());
		resetWaypointTime(timestamp_now());

		// No more waypoints to process
		if (waypoints.size() == 0)
		{
			cout << timestamp_now() << " No more waypoints!" << endl;

			if (pointWithinBound(destPose))
			{
				setDestReached(true);
				cout << "We have reached our destination :)" << endl;
			}

			// form a STOP message to send
			acfrlcm::auv_control_t cc;
			cc.utime = timestamp_now();
			cc.run_mode = acfrlcm::auv_control_t::STOP;
			lcm.publish("AUV_CONTROL", &cc);

			return getDestReached();
		}

	}

	Pose3D currPose = getCurrPose();

	// Calculate desired heading to way point
	//  This is not our bearing to the point but a global angle
	double desHeading = atan2(wp.getY() - currPose.getY(),
			wp.getX() - currPose.getX());

	// Calculate desired velocity. This is set to dest velocity by default
	double desVel = destVel;
	// Ramp down the velocity when close to the destination
	//double distToDest = getDistToDest();
	//if( distToDest < velChangeDist ) {
	//	desVel = destVel * (distToDest / velChangeDist);
	//}

	// form a message to send
	acfrlcm::auv_control_t cc;
	cc.utime = timestamp_now();
	cc.run_mode = acfrlcm::auv_control_t::RUN;
	cc.heading = desHeading;
	cc.vx = desVel;
	if (getDepthMode() == acfrlcm::auv_path_command_t::DEPTH)
	{
		cc.depth = wp.getZ();
		cc.depth_mode = acfrlcm::auv_control_t::DEPTH_MODE;
	}
	else
	{
		// set the depth goal using the filtered desired altitude.
		cc.depth = currPose.getZ() + (currAltitude - wp.getZ());
		cc.depth_mode = acfrlcm::auv_control_t::DEPTH_MODE;
	}
	lcm.publish("AUV_CONTROL", &cc);

	return 1;
}

int LocalPlanner::sendResponse()
{
	acfrlcm::auv_path_response_t pr;
	pr.utime = timestamp_now();
	pr.goal_id = destID;
	pr.are_we_there_yet = destReached;
	if (waypoints.size() > 2)
		pr.distance = waypoints.size() * wpDropDist;
	else
		pr.distance = currPose.positionDistance(destPose);
	lcm.publish("PATH_RESPONSE", &pr);

	return 1;
}

int main(int argc, char **argv)
{
	// install the signal handler
	mainExit = 0;
	signal(SIGINT, signalHandler);

	LocalPlanner *lp = new LocalPlanner();
	if (!lp->loadConfig(basename(argv[0])))
	{
		cerr << "Failed loading config" << endl;
		return 0;
	}

	//    pthread_t tidProcessWaypoints;
	//    pthread_create(&tidProcessWaypoints, NULL, processWaypoints, lp);
	//    pthread_detach(tidProcessWaypoints);

	while (!mainExit)
		lp->process();

	//    pthread_join(tidProcessWaypoints, NULL);

	delete lp;

	return 1;
}

