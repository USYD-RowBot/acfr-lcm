#include "local_planner.hpp"
#include <string>
#include <libgen.h>
#include <signal.h>
#include <pthread.h>
#include <error.h>
#include <unistd.h>
#include <bot_param/param_client.h>
#include <cstdio>
#include <iomanip>
//#include <atomic>

#include "perls-lcmtypes++/acfrlcm/auv_path_response_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_control_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_local_path_t.hpp"

using namespace std;


// Exit handler, I swear this is the only global
bool mainExit;
void signalHandler(int sig)
{
	mainExit = true;
}

// Nav callback
void onNavLCM(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
		const acfrlcm::auv_acfr_nav_t *nav, LocalPlanner *lp)
{
	lp->onNav(nav);
}

// Obs Avoidance callback
void onOALCM(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
		const senlcm::oa_t *oa, LocalPlanner *lp)
{
	lp->onOA(oa);
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
	// We haven't reached our goal but don't have any more waypoints. Or we are in our hold position and waiting trill we drift too far.
	// 	Let's recalculate a feasible path.
	// Addition: Make the replanning stage occur at a set period
	//        if( (lp->waypoints.size() == 0) && (lp->getDestReached() == false) && (lp->getNewDest() == true))// ||  (( timestamp_now() - lp->getReplanTime() )/1000000 > 5) ) // if we want faster replanning, or a different number than the heartbeat, this callback needs to be made faster and this variable can be used
	static long count = 0;
	//	double timeSinceReplan = (timestamp_now() - lp->getReplanTime()) / 1000000.;

	if ((((lp->getDestReached() == false && lp->getNewDest() == true)|| (lp->getHoldmode() == true)) &&
		//((lp->waypoints.size() == 0) || (timeSinceReplan > lp->getReplanInterval()))
				(lp->getWaypointTimePassedSec() > lp->getWaypointTimeout()) ))
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
		forwardBound(0.5), sideBound(2.0), depthBound(0.25), headingBound(0.1), distToDestBound(2.0),
		maxAngleWaypointChange(0.0), radiusIncrease(1.0), maxAngle(300),
		wpDropDist(4.0), wpDropAngle(M_PI / 18.), replanInterval(1.5),
		waypointTime(timestamp_now()), replanTime(timestamp_now()), diffSteer(false), aborted(false), holdMode(false)
{
	gpState.state = acfrlcm::auv_global_planner_state_t::IDLE;

	fp.open("/tmp/log_waypoint.txt", ios::out);
	fp_wp.open("/tmp/log_waypoint_now.txt", ios::out);


	cout << endl << endl << endl << "LocalPlanner started" << endl;
}

LocalPlanner::~LocalPlanner()
{

	if (fp.is_open())
		fp.close();
	if (fp_wp.is_open())
		fp_wp.close();
}

int LocalPlanner::subscribeChannels()
{
	// sunscribe to the required LCM messages
	lcm.subscribeFunction(vehicle_name+".ACFR_NAV", onNavLCM, this);
	lcm.subscribeFunction(vehicle_name+".PATH_COMMAND", onPathCommandLCM, this);
	lcm.subscribeFunction(vehicle_name+".GLOBAL_STATE", onGlobalStateLCM, this);
	lcm.subscribeFunction("HEARTBEAT_5HZ", recalculate, this);
	lcm.subscribeFunction(vehicle_name+".OA", onOALCM, this);
	
    return 1;
}

int LocalPlanner::initialise()
{
	subscribeChannels();
	this->init();
	
	return 1;
}
	
/**
 * Copy current nav solution and process the waypoints
 */
/*
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
*/

/**
 * We have received a new path command.
 * Let's calcualte a new path.
 */
 /*
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
*/
#define DIVE_REV_VEL -0.5
//#define DIVE_REV_VEL 0 // for lab testing
#define DIVE_PITCH (10.0 / 180.0 * M_PI)
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

	bool success = false;
	vector<Pose3D> wps;
	
	
	// Special case where we have no X or Y so it's a depth change or a heading change without moving
	if((destPose.getX() != destPose.getX()) && (destPose.getY() != destPose.getY()))
	{
		cout << "Depth or Heading change" << endl;
	
		wps.push_back(destPose);
		waypoints.clear();
		waypoints = wps;	
		
		// Save the start pose and start velocity
		startPose = currPose;

		printWaypoints();
		publishWaypoints();

		return true;
	}

	Pose3D destPoseRel = getRelativePose(destPose);
	double relAngle = atan2( destPoseRel.getY(), destPoseRel.getX() );
	cout << "Dest rel: X=" << destPoseRel.getX() << " Y=" << destPoseRel.getY()
	<< ", angle=" << relAngle/M_PI*180 << " Diff Steer mode=" << diffSteer << endl;

	// If the waypoint is just ahead of us no need to use Dubins. We will rely
	//	on the controller to get us there.
	if ((destPoseRel.getX() < 0 ||
	    ((destPoseRel.getX() > 2*turningRadius)) ||
		((fabs(relAngle) > 45./180*M_PI) && !diffSteer) || 
		((fabs(destPoseRel.getX()) > 2) && (fabs(destPoseRel.getY() > 2)) && diffSteer))&& !aborted)
	//if(0)
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
		wps = dp.calcPath(startPose, destPose);

		// TODO: should we be more intelligent here?
		// maybe increase the radius and try again (we had this behavior before)
		if ((wps.size() == 0))
		{
			cerr << "Failed to calculate a feasible path using Dubins path"
					<< endl;

				wps.push_back(destPose);
				success = false;
		}
		else {
			cout << "New waypoints calculated using Dubins" << endl;
			success = true;
		}
	}
	else
	{
		cout << "We are close to the new waypoint "
				<< "and use the controller to get us there."
				<< endl;
		wps.push_back(destPose);
		success = true;
	}

	// Managed to calculate a path to destination
	// TODO: do we need mutex around this?
	waypoints.clear();
	waypoints = wps;

	// Save the start pose and start velocity
	startPose = currPose;

	//setDestReached( false );

	printWaypoints();
	publishWaypoints();

	return success;
}

/**
 * Account for changes in system state
 */
int LocalPlanner::onGlobalState(
		const acfrlcm::auv_global_planner_state_t *gpStateLCM)
{
	cout << "Change of global state to: " << (int) (gpStateLCM->state) << endl;
	gpState = *gpStateLCM;

	/* State change to IDLE should immediately kill motors and stop path replanning
	 * State change to RUN from PAUSE should move the robot to it's last position then continue the mission
	 * State change to PAUSE should hold position and return to last position if drifting occurs
	 * State change to ABORT should drive the robot to the surface unless it is already at the surface
	 */
	if (gpState.state == acfrlcm::auv_global_planner_state_t::IDLE)
	{
		//reset waypoints
		waypoints.clear();
		//Reset flags
		newDest = false;
		destReached = true;		
		// form a STOP message to send
		acfrlcm::auv_control_t cc;
		cc.utime = timestamp_now();
		cc.run_mode = acfrlcm::auv_control_t::STOP;
		lcm.publish(vehicle_name+".AUV_CONTROL", &cc);
	}

	if ((gpState.state == acfrlcm::auv_global_planner_state_t::RUN) && (holdMode)){
		//hold mode is off
		holdMode = false;
		//check to see if we are resuming or starting a new mission
		if (!newDest){
			newDest = true;
			oldPose = destPose;
			destPose = holdPose;
			resetWaypointTime(timestamp_now());
		}
		calculateWaypoints();
	}

	if (gpState.state == acfrlcm::auv_global_planner_state_t::PAUSE){
		//save destination for later
		holdPose = destPose;
		//make the hold point just ahead of us
		destPose.setPosition(currPose.getX()+currVel[0], currPose.getY()+currVel[1], currPose.getZ()+currVel[2]);
		destPose.setRollPitchYawRad(0, 0, currPose.getYawRad());
		//set flags
		holdMode = true;
		newDest = false;
		waypoints.clear();
		calculateWaypoints();
	}
	
	if (gpState.state == acfrlcm::auv_global_planner_state_t::ABORT)
	{
		cout << "Received ABORT via LCM" << endl;
		execute_abort();
	}
	return 1;
}

int LocalPlanner::loadConfig(char *program_name)
{
	BotParam *param = NULL;
	param = bot_param_new_from_server(lcm.getUnderlyingLCM(), 1);
	if (param == NULL)
		return 0;

	char rootkey[64];
	char key[128];
	sprintf(rootkey, "acfr.%s", program_name);

	sprintf(key, "%s.turning_radius", rootkey);
	turningRadius = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.minimum_altitude", rootkey);
	minAltitude = bot_param_get_double_or_fail(param, key);

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
	
	sprintf(key, "%s.depth_bound", rootkey);
	depthBound = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.heading_bound", rootkey);
	headingBound = bot_param_get_double_or_fail(param, key);

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

	sprintf(key, "%s.fwd_distance_slowdown", rootkey);
	fwd_distance_slowdown = bot_param_get_double_or_fail(param, key);
	
    sprintf(key, "%s.fwd_distance_min", rootkey);
	fwd_distance_min = bot_param_get_double_or_fail(param, key);
	return 1;
}

int LocalPlanner::process()
{
    // setup signal handlers
	mainExit = false;
	signal(SIGINT, signalHandler);

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

	return 0;
}

// Velocity adjustments based on obstacle avoidance
// Taken from Sirius trajectory.c

double LocalPlanner::calcVelocity(double desired_velocity, double desired_altitude)
{
    double obs_velocity = desired_velocity;
    double alt_velocity = desired_velocity;

    if((timestamp_now() - oa.utime) < 5e6)
    {
        if (fwd_distance_min > 0 &&  oa.forward_distance> 0)
        {
            /*
      	    Obstacle avoidance slowdown.  The desired velocity will
	        be ramped down linearly between the slowdown distance and
	        the minimum distance.

            speed
            ^         ____________ default speed
            |        /
            |       /
            |______/____________________________>  dist
		    ^  ^
	         min   slowdown
            */
            if (oa.forward_distance < fwd_distance_min)
	        {
	            obs_velocity = 0.0;
                cout << "Too close to obstacle. Setting xy_vel to " << obs_velocity << endl;
	        } 
            else if (oa.forward_distance < fwd_distance_slowdown) 
            {
		        double speedRampSlope = desired_velocity/(fwd_distance_slowdown - fwd_distance_min);
		        obs_velocity = speedRampSlope*(oa.forward_distance - fwd_distance_min);
                cout << "Too close obstacle. Setting xy_vel to " << obs_velocity << endl;
	        }
        }


        // now check the altitude following and slow down if we are outside of the desired
        // following band
        if(getDepthMode() == acfrlcm::auv_path_command_t::ALTITUDE)
        {
            /*
            Altitude following slowdown.  The desired velocity will
            be ramped down linearly as a function of the altitude error. Note that the desired
            altitude is stored in tm->depth.value

                         speed
                          ^
                   __ ___|______ default speed
                   /     |      \
          ___   __/      |       \______________ 0.1 speed
                         |
             ___________0|______________________>   alt err           
                  ^  ^          ^  ^
                         slowdown  max
            */ 
            // tm->depth.value is the desired altitude
    	    double altitude = fmin(oa.altitude, navAltitude);
	        double alt_error = altitude - desired_altitude;
            double alt_error_max = 0.75; // maximum slowdown when off by 1.0m
            double alt_error_slowdown = 0.25; // start slowing down when off by 0.25m
            double max_slowdown_factor = 0.1; // percentage of desired speed for max slowdown

            // FIXME: Create different behaviour for too high/low.  For now consider only low altitude.
            if ( alt_error < -alt_error_max )
	        {
		        alt_velocity = max_slowdown_factor * desired_velocity;
	        } 
            else if ( alt_error <  -alt_error_slowdown && alt_error_max - alt_error_slowdown > 0 ) 
            {
                // calculate the normalised slope of the slowdown
		        double speedRampSlope = (max_slowdown_factor-1)/(alt_error_max - alt_error_slowdown);
                // calculate the normalised intercept of the slowdown
                double speedRampIntercept = 1 - speedRampSlope*alt_error_slowdown;
                // convert the altitude error to a desired speed as a function of the default speed
		        alt_velocity = desired_velocity*(speedRampSlope*(fabs(alt_error)) + speedRampIntercept);
	        }
        } 
    }
    return fmin(obs_velocity, alt_velocity);
}


/**
 * Look at the next waypoint and
 * 1) remove it from the list if we have reached it or
 * 2) calculate commands for the controller
 */
int LocalPlanner::processWaypoints()
{
	Pose3D wp;
	if (waypoints.size() == 0)
	{
		// Nothing to do
		return 0;
	}
	// Get the next waypoint
	wp = waypoints.at(0);

	bool atDest = false;
	// Check if we are there yet taking into consideration the depth/heading only commands
/*	if((wp.getX() != wp.getX()) && (wp.getY() != wp.getY()) && (wp.getZ() != wp.getZ()))
		atDest = pointWithinHeading(wp);
	else if((wp.getX() != wp.getX()) && (wp.getY() != wp.getY()) && (wp.getYawRad() != wp.getYawRad()))
	{
		atDest = pointWithinDepth(wp);
		cout << "Checking Depth" << endl;
	}
	else
	*/
		atDest = pointWithinBound(wp);

	// We have reached the next waypoint
	if (atDest)
	{
		printf( "[%3.2f, %3.2f, %3.2f] reached.\n",
				wp.getX(),
				wp.getY(),
				wp.getZ() );
		waypoints.erase(waypoints.begin());
		resetWaypointTime(timestamp_now());

		printWaypoints();

		// No more waypoints to process
		if (waypoints.size() == 0)
		{
			cout << timestamp_now() << " No more waypoints!" << endl;

			if (pointWithinBound(destPose) && (gpState.state == acfrlcm::auv_global_planner_state_t::PAUSE))
			{
				cout << "Reached hold location" << endl;
			}
			else if ((pointWithinBound(destPose) && !holdMode))
			{
				setDestReached(true);
				cout << "We have reached our destination :)" << endl;
				return getDestReached();
			}

		}

	}

	Pose3D currPose = getCurrPose();

	// Calculate desired heading to way point
	//  This is not our bearing to the point but a global angle
	// Special case where we have no X or Y so it's a depth change or a heading change without moving
	double desHeading;
	if((destPose.getX() != destPose.getX()) && (destPose.getY() != destPose.getY()))
		desHeading = wp.getYawRad();
	else
		desHeading = atan2(wp.getY() - currPose.getY(),
			wp.getX() - currPose.getX());

	// Calculate desired velocity. This is set to dest velocity by default
	double desVel = destVel;
	// Ramp down the velocity when close to the destination
	double distToDest = getDistToDest();
	if( distToDest < velChangeDist ) 
		desVel = destVel * (distToDest / velChangeDist);

	// form a message to send
	acfrlcm::auv_control_t cc;
	cc.utime = timestamp_now();
	cc.run_mode = acfrlcm::auv_control_t::RUN;
	cc.heading = desHeading;
	//cc.vx = desVel;
    	static double depth_ref = 0.0;
    	double curr_depth_ref;

    // Use the obstacle avoidance altitude if available
    	double altitude;
	if((timestamp_now() - oa.utime) < 5e6)
    	altitude = fmin(oa.altitude, navAltitude);
    else
	     altitude = navAltitude;

	if (getDepthMode() == acfrlcm::auv_path_command_t::DEPTH)
	{
		//cc.depth = wp.getZ();
        curr_depth_ref = wp.getZ();

        // check we don't get closer to the bottom than our minimum
		double curr_alt_ref = currPose.getZ() + (altitude - minAltitude);

        if (curr_alt_ref < curr_depth_ref)
            curr_depth_ref = curr_alt_ref;

		cc.depth_mode = acfrlcm::auv_control_t::DEPTH_MODE;
	}
	else
	{
		// set the depth goal using the filtered desired altitude.
		//cc.depth = currPose.getZ() + (currAltitude - wp.getZ());
		curr_depth_ref = currPose.getZ() + (altitude - wp.getZ());

		cc.depth_mode = acfrlcm::auv_control_t::DEPTH_MODE;
	}
    
    // return velocty based on obstacle avoidance
    cc.vx = calcVelocity(desVel, altitude);
    
    // FIXME: limit the depth rate change to yield an achievable 
    // trajectory. This is modelled on a forward speed of 0.75m/s 
    // with a max pitch of 0.3rad.  This should be configurable or
    // calculated automatically.
    double NAV_DT = 0.1;
    double max_depth_ref_change = 0.2*NAV_DT;
    double depth_ref_error = curr_depth_ref - depth_ref;
    if (depth_ref_error > max_depth_ref_change) {
        depth_ref += max_depth_ref_change;     	
    }
    else if (depth_ref_error < -max_depth_ref_change)
        depth_ref -= max_depth_ref_change;
    else
        depth_ref = curr_depth_ref;

    cc.depth = curr_depth_ref;

	lcm.publish(vehicle_name+".AUV_CONTROL", &cc);
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

	lcm.publish(vehicle_name+".PATH_RESPONSE", &pr);

	
	return 1;
}

void LocalPlanner::printWaypoints() const {
	cout << "waypoints = [" << endl;
	for (unsigned int i = 0; i < waypoints.size(); i++)
	{
		printf( "%3.2f, %3.2f, %3.2f;\n",
				waypoints.at(i).getX(),
				waypoints.at(i).getY(),
				waypoints.at(i).getZ() );
	}
	cout << "];" << endl;
}
bool LocalPlanner::publishWaypoints() {
	acfrlcm::auv_local_path_t lp;
	lp.utime = timestamp_now();
	unsigned int i;
	// TODO: comparison between signed and unsiged!
	for( i = 0; i < waypoints.size() && i < lp.max_num_el; i++  ) {
		lp.x[i] = waypoints[i].getX();
		lp.y[i] = waypoints[i].getY();
		lp.z[i] = waypoints[i].getZ();
	}
	lp.num_el = i;
	lcm.publish(vehicle_name+".LOCAL_PATH", &lp);

	return true;
}

