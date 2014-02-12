#include "local_planner.hpp"

// Exit handler, I swear this is the only global
int mainExit;
void signalHandler(int sig) {
	mainExit = 1;
}

// Nav callback
void onNavLCM(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
		const acfrlcm::auv_acfr_nav_t *nav, LocalPlanner *lp) {
	lp->onNav(nav);
}

// Path command callback
void onPathCommandLCM(const lcm::ReceiveBuffer* rbuf,
		const std::string& channel, const acfrlcm::auv_path_command_t *pc,
		LocalPlanner *lp) {
	lp->onPathCommand(pc);
}

// Global State command callback
void onGlobalStateLCM(const lcm::ReceiveBuffer* rbuf,
		const std::string& channel, const acfrlcm::auv_global_planner_state_t *gpState,
		LocalPlanner *lp) {
	lp->onGlobalState(gpState);
}

// heartbeat callback
void calculateLCM(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
		const perllcm::heartbeat_t *heartbeat, LocalPlanner *lp) {
	// We haven't reached our goal but don't have any more waypoints. Let's recalculate a feasible path
	// Addition: Make the replanning stage occur at a set period
	//        if( (lp->waypoints.size() == 0) && (lp->getDestReached() == false) && (lp->getNewDest() == true))// ||  (( timestamp_now() - lp->getReplanTime() )/1000000 > 5) ) // if we want faster replanning, or a different number than the heartbeat, this callback needs to be made faster and this variable can be used
	static long count = 0;
//	double timeSinceReplan = (timestamp_now() - lp->getReplanTime()) / 1000000.;

	if ((lp->getDestReached() == false && lp->getNewDest() == true) &&
	//((lp->waypoints.size() == 0) || (timeSinceReplan > lp->getReplanInterval()))
			(lp->getWaypointTimePassedSec() > lp->getWaypointTimeout())) {

		if (lp->getWaypointTimePassedSec() > lp->getWaypointTimeout())
			cout << "Way point timed out: "
					<< lp->getWaypointTimePassedSec() << " > "
							<< lp->getWaypointTimeout() << endl;

		cout << endl << "------------------------" << endl
				<< "RECALCULATING A NEW PATH" << endl
				<< "------------------------" << endl << endl;

		lp->calculateWaypoints();
		lp->resetWaypointTime(timestamp_now());
		lp->resetReplanTime(timestamp_now());
	} else if (++count % 10 == 0) {
//		cout << "Time since replanning:" << timeSinceReplan << endl << endl;
	}

	lp->sendResponse();
}

LocalPlanner::LocalPlanner() :
		startVel(0), destVel(0), newDest(false), destReached(false), depthMode(
				0), diveMode(0), destID(0), turningRadius(1.0), maxPitch(0), lookaheadVelScale(
				0), maxDistFromLine(0), maxAngleFromLine(0), velChangeDist(1.0), defaultLegVel(
				1.0), waypointTimeout(1e3), forwardBound(0.5), sideBound(2.0), distToDestBound(
				2.0), maxAngleWaypointChange(0.0), radiusIncrease(1.0), maxAngle(
				300), wpDropDist(4.0), wpDropAngle(M_PI / 18.), replanInterval(
				1.5), waypointTime(timestamp_now()), replanTime(timestamp_now()) {

	// sunscribe to the required LCM messages
	lcm.subscribeFunction("ACFR_NAV", onNavLCM, this);
	lcm.subscribeFunction("PATH_COMMAND", onPathCommandLCM, this);
	lcm.subscribeFunction("GLOBAL_STATE", onGlobalStateLCM, this);
	lcm.subscribeFunction("HEARTBEAT_5HZ", calculateLCM, this);

	gpState.state = acfrlcm::auv_global_planner_state_t::IDLE;

	//    pthread_mutex_init(&currPoseLock, NULL);
	//    pthread_mutex_init(&destPoseLock, NULL);
	//    pthread_mutex_init(&waypointsLock, NULL);
    fp.open("/tmp/log_waypoint.txt",
			ios::out);
	fp_wp.open(
            "/tmp/log_waypoint_now.txt",
			ios::out);
}

LocalPlanner::~LocalPlanner() {

	if (fp.is_open())
		fp.close();
	if (fp_wp.is_open())
		fp_wp.close();
	//    pthread_mutex_destroy(&currPoseLock);
	//    pthread_mutex_destroy(&destPoseLock);
	//    pthread_mutex_destroy(&waypointsLock);
}
/**
 * Copy current navigation status
 */
int LocalPlanner::onNav(const acfrlcm::auv_acfr_nav_t *nav) {

	currPose.setPosition(nav->x, nav->y, nav->depth);
	currAltitude = nav->altitude;

	// Instead of heading, we make the current pose "heading" actually the slip
	// 	angle (bearing), for control
#if 0
	currPose.setRollPitchYawRad( nav->roll, nav->pitch, nav->heading );
#else
	double bearing = atan2(
			nav->vy * cos(nav->heading) + nav->vx * sin(nav->heading),
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
		// if required process the special dive mode
		if (diveMode)
			dive();
		else
        {
			processWaypoints();
        }
	}
	
	return 1;
}

/**
 * Copy new destination pose and calculate a new path to this point
 */
int LocalPlanner::onPathCommand(const acfrlcm::auv_path_command_t *pc) {
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

	// special dive case, first we need to get off the surface
	if ((currPose.getZ() < 0.2)
			&& (depthMode == acfrlcm::auv_control_t::DEPTH_MODE)
			&& (pc->waypoint[2] > 0)) {
        setNewDest(false);
        setDestReached(false);
		diveStage = 0;
		diveMode = 1;
		cout << "Going into dive mode" << endl;
		return 1;
	}

	setDestReached(false);
	setNewDest(true);
	if (calculateWaypoints() == 0) {
		// TODO: We must do something more clever here!!!!!
		cout
				<< endl
				<< "----------------------------------------"
				<< "Can't calcualte a feasible path. Let's cruise and see what happens"
				<< "----------------------------------------" << endl << endl;
	}
	resetWaypointTime(timestamp_now());

	// for now only process waypoints or dive commands if we are in
	// RUN mode.  This will need to be modified to take into account
	// an active PAUSE mode.
	if (gpState.state == acfrlcm::auv_global_planner_state_t::RUN)
	{
		if (diveMode)
			dive();
		else
			processWaypoints();
	}

	return 1;
}

#define DIVE_REV_VEL -0.5
//#define DIVE_REV_VEL 0 // for lab testing
#define DIVE_PICTH (10.0 / 180.0 * M_PI)
#define DIVE_DEPTH 0.5

int LocalPlanner::dive() {
	// we have a list of things we need to achieve to get off the surface
	acfrlcm::auv_control_t cc;
	cc.utime = timestamp_now();
	cc.run_mode = acfrlcm::auv_control_t::DIVE;
	cc.depth_mode = acfrlcm::auv_control_t::PITCH_MODE;
	cout << "Dive stage : " << diveStage << " vx: " << currVel[0] << " depth: "
			<< currPose.getZ() << " pitch: "
			<< currPose.getRotation().getPitchRad() / M_PI * 180 << endl;
	switch (diveStage) {
	case 0:
		// get upto reverse speed
		cc.vx = DIVE_REV_VEL;
		cc.pitch = 0;
		lcm.publish("AUV_CONTROL", &cc);

		if (currVel[0] < (DIVE_REV_VEL + 0.1))
			diveStage = 1;
		break;

	case 1:
		// pitch the tail down
		cc.vx = DIVE_REV_VEL;
		cc.pitch = DIVE_PICTH;
		lcm.publish("AUV_CONTROL", &cc);

		// go until we are under, keeping in mind out pressure sensor is at the front
        if (currPose.getZ() > DIVE_DEPTH)
			diveStage = 2;
		break;

	case 2:
		// level out
		cc.vx = DIVE_REV_VEL;
		cc.pitch = 0;
		lcm.publish("AUV_CONTROL", &cc);

		if (fabs(currPose.getRotation().getPitchRad()) < (5.0 / 180.0 * M_PI))
			diveStage = 3;
		break;

	case 3:
		// go back to zero velocity and hope we stay down log enough to start
		cc.vx = 0;
		cc.pitch = 0;
		lcm.publish("AUV_CONTROL", &cc);

		// have we finished
		if (fabs(currVel[0]) < 0.1) {
			diveMode = 0;
			diveStage = 4;
            setDestReached(false);
            setNewDest(true);
            if (calculateWaypoints() == 0) {
                // TODO: We must do something more clever here!!!!!
                cout
                        << endl
                        << "----------------------------------------"
                        << "Can't calcualte a feasible path. Let's cruise and see what happens"
                        << "----------------------------------------" << endl << endl;
            }
            processWaypoints();
		}
		break;
	}
	return 1;
}

/**
 * Call Dubins path planner to generate a path from current location to the destination waypoint
 */
int LocalPlanner::calculateWaypoints() {

	// Get a copy of curr and dest pose
	//    pthread_mutex_lock(&currPoseLock);
	//    pthread_mutex_lock(&destPoseLock);
	Pose3D currPose = this->currPose;
	Pose3D destPose = this->destPose;
	double currV = this->currVel[0];
	//    pthread_mutex_unlock(&currPoseLock);
	//    pthread_mutex_unlock(&destPoseLock);

	cout << timestamp_now() << " Calculating new waypoints..." << endl;
	cout << setprecision(2) << "CurrPose=" << currPose.getX() << ","
			<< currPose.getY() << "," << currPose.getZ() << " < "
			<< currPose.getYawRad() / M_PI * 180 << endl;
	cout << setprecision(2) << "DestPose=" << destPose.getX() << ","
			<< destPose.getY() << "," << destPose.getZ() << " < "
			<< destPose.getYawRad() / M_PI * 180 << endl;

	Pose3D rot;
	rot.setRollPitchYawRad(M_PI, 0, 0); // Local coord has Z down where as global has Z up
	Pose3D wpRel = currPose.compose(rot).inverse().compose(destPose);

	vector<Pose3D> waypoints;
	// If the waypoint is just ahead of us no need to use Dubins. We will rely
	//	on the controller to get us there.
	// TODO: Should we change this so if the waypoint is more than 45deg to each
	//		side, then we calculate rather that if it is only behind us?
	if (0) {//(currPose.positionDistance(destPose) > 2 * turningRadius)
	//		|| (wpRel.getX() < 0)) {

		DubinsPath dp;
		dp.setCircleRadius(turningRadius);
		dp.setMaxPitch(maxPitch);
		dp.setWaypointDropDist(wpDropDist);
		//dp.setWaypointDropAngleRad(wpDropAngle * M_PI / 180.);
		dp.setWaypointDropAngleFromDropDist();

		// Don't use current pose but a pose looking a bit ahead. We adjust this
		//	by employing the current velocity
		double lookAheadTime = 0.2; // [s]
		Pose3D lookAheadPose;
		lookAheadPose.setX(currV * lookAheadTime);
		Pose3D startPose = currPose.compose(lookAheadPose);

		// Try to calculate a feasible Dubins path. If we fail we try
		//  a second time with twice the circle radius
		waypoints = dp.calcPath(startPose, destPose);

		if ((waypoints.size() == 0)
				&& ((currPose.positionDistance(destPose) > 4 * turningRadius)
						|| (wpRel.getX() < 0))) {
			cerr << "Failed to calculate a feasible path using Dubins path"
					<< endl;
			cerr << "Increasing the turn radius to " << turningRadius * 2
					<< " and trying again" << endl;

			dp.setCircleRadius(turningRadius * 2);
			waypoints = dp.calcPath(startPose, destPose);
			if (waypoints.size() == 0) {
				cerr << "LocalPlanner failed to generate a feasible path"
						<< endl;
				cerr << "\tfrom currPose: " << currPose.toString() << endl;
				cerr << "\tto destPose: " << destPose.toString() << endl;

				return 0;
			}
		}

		// Try a few perturbations to the original path parameters

		// First try to change the turning radius

		//    double pathLengths[ITERATION_NUM];
		//    double testTurningRadius[ITERATION_NUM];
		//    testTurningRadius[0] = turningRadius;
		//    bool feasibleFound = false;

		//    if (waypoints.size() != 0)
		//    {
		//        feasibleFound = true;
		//        pathLengths[0] = dp.getPathLength();
		//    }
		//    else
		//    {
		//        pathLengths[0] = DOUBLE_MAX; // max this out
		//    }

		//    for (int i=0;i<ITERATION_NUM-1;i++)
		//    {
		//        testTurningRadius[i+1] = testTurningRadius[i] + radiusIncrease;
		//        dp.setCircleRadius( testTurningRadius[i+1]);
		//        waypoints = dp.calcPath( currPose, destPose );

		//        if (waypoints.size() != 0)
		//        {
		//            feasibleFound = true;
		//            pathLengths[i+1] = dp.getPathLength();
		//        }
		//        else
		//        {
		//            pathLengths[i+1] = DOUBLE_MAX; // max this out
		//        }

		//    }

		//    //Check if we are going in a loop, if we are and we are trying to essentially go straight, and it looks like shifting the waypoint might help, try it to avoid the loop

		//    destPose.getYawRad();
		//    currPose.getYawRad();
		//    double desHeading = atan2(destPose.getY()-currPose.getY(),destPose.getX()-currPose.getX());

		//    if ( (dp.getMaxAngle() > maxAngle)&&( fabs(currPose.getYawRad()-desHeading)<maxAngleFromLine*M_PI/180 )&&( fabs(destPose.getYawRad()-desHeading)<maxAngleFromLine*M_PI/180 ) ) // if we are doing a loop, so that it only does this when way points are near aligned given the entry and exit headings, and poses
		//    {
		//        Pose3D destPoseNew = destPose;
		//        double rOld,pOld,hOld;

		//        destPoseNew.getRollPitchYawRad(rOld, pOld, hOld);

		//        double xOld = destPoseNew.getX();
		//        double yOld = destPoseNew.getY();
		//        double zOld = destPoseNew.getZ();

		//        double pathLengthsNew[ITERATION_NUM*ITERATION_NUM*ITERATION_NUM*ITERATION_NUM];
		//        double paramsNew[ITERATION_NUM*ITERATION_NUM*ITERATION_NUM*ITERATION_NUM][4];
		//        double turningRadiusNew;// = turningRadius
		//        double headingNew;// = destPose[6]
		//        double xNew;// = destPose[0]
		//        double yNew;// = destPose[1]
		//        int path_index = 0;
		//        bool feasibleFoundNew = false;

		//        for (int i=0;i<ITERATION_NUM;i++) // go through turning radius
		//        {
		//            for (int j = 0;j<ITERATION_NUM;j++) // go through heading angles
		//            {
		//                for (int k = 0;k<ITERATION_NUM;k++) // go through waypoint shifts X
		//                {
		//                    for (int l = 0;l<ITERATION_NUM;l++) // go through waypoint shifts Y
		//                    {

		//                        turningRadiusNew = turningRadius + i*radiusIncrease;
		//                        headingNew = hOld - maxAngleWaypointChange*(M_PI/180) + j*2*maxAngleWaypointChange/(ITERATION_NUM-1)*(M_PI/180);
		//                        xNew = xOld - maxDistFromLine + k*2*maxDistFromLine/(ITERATION_NUM-1);
		//                        yNew = yOld - maxDistFromLine + l*2*maxDistFromLine/(ITERATION_NUM-1);

		//                        destPoseNew.setPosition(xNew,yNew,zOld);
		//                        destPoseNew.setRollPitchYawRad(rOld, pOld, headingNew);

		//                        dp.setCircleRadius( turningRadiusNew);
		//                        waypoints = dp.calcPath( currPose, destPoseNew );

		//                        if (waypoints.size() != 0)
		//                        {
		//                            feasibleFoundNew = true;
		//                            pathLengthsNew[path_index] = dp.getPathLength();
		//                            paramsNew[path_index][0] = turningRadiusNew;
		//                            paramsNew[path_index][1] = headingNew;
		//                            paramsNew[path_index][2] = xNew;
		//                            paramsNew[path_index][3] = yNew;
		//                        }
		//                        else
		//                        {
		//                            pathLengthsNew[path_index] = DOUBLE_MAX; // max this out
		//                        }
		//                        path_index++;
		//                    }
		//                }
		//            }
		//        }

		//        if (feasibleFoundNew == true)
		//        {
		//            int bestIndex = getMin(pathLengthsNew,ITERATION_NUM*ITERATION_NUM*ITERATION_NUM*ITERATION_NUM);
		//            dp.setCircleRadius(paramsNew[bestIndex][0]);
		//            destPoseNew.setPosition(paramsNew[bestIndex][2],paramsNew[bestIndex][3],zOld);
		//            destPoseNew.setRollPitchYawRad(rOld, pOld, paramsNew[bestIndex][1]);

		//            cout << "best offsets: " << (paramsNew[bestIndex][0] - turningRadius) << " "<< (destPoseNew.getX() - destPose.getX()) << " "<< (destPoseNew.getY() - destPose.getY()) <<" "<<((destPoseNew.getYawRad()  - destPose.getYawRad())*180/M_PI)  << endl;
		//            cout << "old:"<< (turningRadius) << " "<< (destPose.getX()) << " "<< (destPose.getY()) <<" "<<((destPose.getYawRad())*180/M_PI)  << endl;
		//            cout << "new:"<< (paramsNew[bestIndex][0]) << " "<< (destPoseNew.getX()) << " "<< (destPoseNew.getY()) <<" "<<((destPoseNew.getYawRad())*180/M_PI)  << endl;
		//            waypoints = dp.calcPath( currPose, destPoseNew );
		//        }
		//        else
		//        {
		//            cerr << "LocalPlanner failed to generate a feasible path" << endl;
		//            cerr << "\tfrom currPose: " << currPose.toString() << endl;
		//            cerr << "\tto destPose: " << destPose.toString() << endl;

		//            return 0;
		//        }

		//    }
		//    else
		//    {
		//        double bestRadius;

		//        if (feasibleFound == true)
		//        {
		//            bestRadius = testTurningRadius[getMin(pathLengths,ITERATION_NUM)]; // get the minimum path length
		//            cout << "bestRadius: " << bestRadius << endl;
		//            dp.setCircleRadius( bestRadius);
		//            waypoints = dp.calcPath( currPose, destPose );
		//        }
		//        else
		//        {
		//            cerr << "LocalPlanner failed to generate a feasible path" << endl;
		//            cerr << "\tfrom currPose: " << currPose.toString() << endl;
		//            cerr << "\tto destPose: " << destPose.toString() << endl;

		//            return 0;
		//        }
		//    }
		cout << "New waypoints calculated using Dubins" << endl;
	} else {
		cout
				<< "We are close to the new waypoint and use the controller to get us there."
				<< endl;
		waypoints.push_back(destPose);
	}

	// Managed to calculate a path to destination
	this->waypoints = waypoints;

	// Save the start pose and start velocity
	startPose = currPose;
	startVel = currVel[0];

	//setDestReached( false );

	cout << "waypoints = [" << endl;
	if (!fp.is_open())
		fp.open(
                "/tmp/log_waypoint.txt",
				ios::out | ios::app);
	for (unsigned int i = 0; i < waypoints.size(); i++) {
		cout << waypoints.at(i).getX() << ", " << waypoints.at(i).getY() << ", "
				<< waypoints.at(i).getZ() << ";" << endl;
		fp << waypoints.at(i).getX() << " " << waypoints.at(i).getY() << " "
				<< waypoints.at(i).getZ() << endl;

	}
	fp << 0 << " " << 0 << " " << 0 << endl;
	fp.close();
	cout << "];" << endl;

	return 1;
}

/**
 * Account for changes in system state
 */
int LocalPlanner::onGlobalState(const acfrlcm::auv_global_planner_state_t *gpStateLCM) {
	cout << "Change of global state to: " << (int)(gpStateLCM->state) << endl;
	gpState = *gpStateLCM;
	// for the moment, send a STOP command to the low level controller
	// for transitions into any state but RUN.  This may need to be
	// modified for more complex PAUSE or ABORT behaviours.
    if (gpState.state != acfrlcm::auv_global_planner_state_t::RUN)
    {
                // form a STOP message to send
                acfrlcm::auv_control_t cc;
                cc.utime = timestamp_now();

                cc.run_mode = acfrlcm::auv_control_t::STOP; // The instant we hit a waypoint, stop the motors, until the global planner sends a waypoint. This fixes idle behaviour.
                lcm.publish("AUV_CONTROL", &cc);
    }
    return 1;
}



int LocalPlanner::loadConfig(char *program_name) {
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

int LocalPlanner::sendResponse() {
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

int LocalPlanner::process() {
	//static void *processLCM(void *u) {
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

//static void *processWaypoints(void *u) {
int LocalPlanner::processWaypoints() {

	unsigned int numWaypoints = this->waypoints.size();
	// Nothing to do
	if (numWaypoints == 0) {
		return 1;
	}

	// Get the first waypoint
	Pose3D waypoint = waypoints.at(0);

	// Get current pose
	Pose3D currPose = getCurrPose();
	Vector3D currVel3D = getCurrVel();
	double currVel = currVel3D[0];

	// Get destination pose
	Pose3D destPose = getDestPose();
	double destVel = getDestVel();

	double distToDest = currPose.positionDistance(destPose);

	// Check if the way point is ahead of us.
	// Convert the pose of waypoint relative to curr pose
	//  if the x is negative it is behind us and we will
	//  ignore it (delete it)
	Pose3D rot;
	rot.setRollPitchYawRad(M_PI, 0, 0); // Local coord has Z down where as global has Z up
	Pose3D wpRel = currPose.compose(rot).inverse().compose(waypoint);
	double distToWp = currPose.positionDistance(waypoint);

	//cout << endl << "====================" << endl;
	//cout << "\nWe have " << numWaypoints << " waypoints " << endl;
//	cout << setprecision(2) << "Current pose : " << currPose.getX() << ", "
//			<< currPose.getY() << ", " << currPose.getZ() << " < "
//			<< currPose.getYawRad() / M_PI * 180. << "deg" << endl;
//	cout << setprecision(2) << "Waypoint pose: " << waypoint.getX() << ", "
//			<< waypoint.getY() << ", " << waypoint.getZ() << " < "
//			<< waypoint.getYawRad() / M_PI * 180. << "deg" << endl;
//	cout << setprecision(2) << "\tRel pose: " << wpRel.getX() << ", "
//			<< wpRel.getY() << ", " << wpRel.getZ() << " < "
//			<< wpRel.getYawRad() / M_PI * 180 << "deg" << endl;
//	cout << setprecision(2) << "\tDist: " << distToWp << endl << endl;

	//cout << "Time for waypoint (Timeout): " << ( timestamp_now() - lp->getWaypointTime() )/1000000 << " (" << lp->getWaypointTimeout() << ")" << endl;
	// We have reached the final Global waypoint and the commanded velocity
	//	is zero. We want to hold position so we will not remove it from the
	//	list
	if (!getDestReached()
			&& ((distToDest < distToDestBound) || (distToWp < forwardBound)
					|| ((wpRel.getX() < 0) && (wpRel.getX() > -2 * forwardBound)
							&& (fabs(wpRel.getY()) < sideBound)))
			//|| ((timestamp_now() - lp->getWaypointTime()) / 1000000 > lp->getWaypointTimeout())
			) {
		if (distToDest < distToDestBound)
			cout << "\tReached Destination: " << destPose.getX() << ","
					<< destPose.getY() << "," << destPose.getZ() << " < "
					<< destPose.getYawRad() / M_PI * 180 << endl;
		else if (distToWp < forwardBound)
			cout << "\tReached " << (numWaypoints == 0 ? "FINAL" : "INTERIM")
					<< " WP: " << waypoint.getX() << ","
					<< waypoint.getY() << "," << waypoint.getZ() << " < "
					<< waypoint.getYawRad() / M_PI * 180 << endl;
		else if ((wpRel.getX() < forwardBound)
				&& (wpRel.getX() > -2 * forwardBound)
				&& (fabs(wpRel.getY()) < sideBound))
			cout << "\tIgnored: behind us" << endl;
		else
			cout << "\tIgnored: time out" << endl;

		// delete it from the list if it is not the last HOLDING waypoint
		if (numWaypoints > 1 || destVel != 0) {
			waypoints.erase(waypoints.begin());
			numWaypoints = waypoints.size();
			resetWaypointTime(timestamp_now());
		}

		// write to log file
		if (!fp_wp.is_open())
			fp_wp.open(
                    "/tmp/log_waypoint_now.txt",
					ios::out); // don't append
		for (unsigned int i = 0; i < waypoints.size(); i++) {
			fp_wp << waypoints.at(i).getX() << " " << waypoints.at(i).getY()
					<< " " << waypoints.at(i).getZ() << endl;

		}
		fp_wp << 0 << " " << 0 << " " << 0 << endl;
		fp_wp.close();

		if (numWaypoints == 0) {
			cout << timestamp_now() << " No more waypoints!" << endl;
			if (distToDest < distToDestBound) {
				// if the destination velocity is 0, then we assume this is a
				// 	hold position goto, so continually attempt to reach the
				// 	waypoint
				cout
						<< "We have reached DEST :) Waiting for next Global waypoint"
						<< endl;
				setDestReached(true);
				setNewDest(false);

                // form a STOP message to send
//                acfrlcm::auv_control_t cc;
//                cc.utime = timestamp_now();

//                cc.run_mode = acfrlcm::auv_control_t::STOP; // The instant we hit a waypoint, stop the motors, until the global planner sends a waypoint. This fixes idle behaviour.
//                lcm.publish("AUV_CONTROL", &cc);
			}

			return 1;
		} else if (numWaypoints == 1 && destVel == 0 && !getDestReached()) {
			cout << "We have reached DEST :) HOLDING waypoint" << endl;
			setDestReached(false);
			setNewDest(false);
		} else {
			return 1;
		}

	}

	// Calculate desired heading to way point
	//  This is not our bearing to the point but a global angle
	double desHeading = atan2(waypoint.getY() - currPose.getY(),
			waypoint.getX() - currPose.getX());

	// Calculate desired velocity. If within 5m from dest pose, start adjusting the velocity
	double desVel = 0;
	//double distToDestWp = numWaypoints * lp->getWpDropDist();
	//cout << "distToDestWp=" << distToDestWp << endl;
	double velChDist = getVelChangeDist();
	//cout << "velChDist=" << velChDist << endl;
	// Linear ramp of speed on last meter to the final way point

	if ((distToDest < velChDist + distToDestBound)
			&& (distToDest >= distToDestBound)) { //( distToDestWp < velChDist  ) {
		//    if (( wpDist < velChDist + distToDestBound)&&(wpDist>=distToDestBound)){ //( distToDestWp < velChDist  ) {

		desVel = currVel
				- (distToDest - velChDist - distToDestBound) / velChDist
						* (destVel - currVel);

		//        desVel =  currVel - (wpDist-velChDist-distToDestBound) / velChDist * (destVel-currVel);
	} else if (distToDest < distToDestBound) {
		//    else if (wpDist < distToDestBound){
		desVel = 0;
	}
	// Linear ramp up
	//        else if( distFromStart < velChDist  && lp->startVel < 1e-3 ) {
	//            desVel = lp->getDefaultLegVel();// * distFromStart / velChDist;
	//        }
	else {
		// set the desired velocity as the destination velocity
		desVel = destVel;// getDefaultLegVel();
	}
//	cout << "Destination velocity:" << destVel << endl;
//	cout << "Desired velocity:" << desVel << endl;

	//cout << "DESIRED" << endl;
	//cout << "\tHEADING  = " << desHeading/M_PI*180 << "deg" << endl;
	//cout << "\tVELOCITY = " << desVel << endl;
	//cout << "\tDEPTH    = " << waypoint.getZ() << endl;

	// for a message to send
	acfrlcm::auv_control_t cc;
	cc.utime = timestamp_now();

	//if( fabs(desVel) < 1e-3 ) {
	// stop the motors
	//cc.run_mode = acfrlcm::auv_control_t::STOP; // commented out so that the vehicle stays stationary even with currents, so the motor never stops

	//}
	//else {
	cc.run_mode = acfrlcm::auv_control_t::RUN;
	cc.heading = desHeading;
	if (getDepthMode() == acfrlcm::auv_path_command_t::DEPTH) {
		cc.depth = waypoint.getZ();
		cc.depth_mode = acfrlcm::auv_control_t::DEPTH_MODE;
	} else {
		//cc.altitude = waypoint.getZ();
		//cc.depth_mode = acfrlcm::auv_control_t::ALTITUDE_MODE;
		// for now let's set the depth goal using the filtered 
		// desired altitude. 
		double depth_ref = currPose.getZ() + (currAltitude - waypoint.getZ()); 
		cc.depth = depth_ref;
		cc.depth_mode = acfrlcm::auv_control_t::DEPTH_MODE;
	}
	cc.vx = desVel;
	//}
	// publish
	lcm.publish("AUV_CONTROL", &cc);

	//        usleep( 100000 );

	//    }

	return 1;
}

int main(int argc, char **argv) {
	// install the signal handler
	mainExit = 0;
	signal(SIGINT, signalHandler);

	LocalPlanner *lp = new LocalPlanner();
	if (!lp->loadConfig(basename(argv[0]))) {
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

int getMin(double array[], int size) // get the index to the minimum value in an array
		{
	int index = 0;
	for (int i = 0; i < size; i++) {
		if (array[i] < array[index]) {
			index = i;
		}
	}

	return index;

}
