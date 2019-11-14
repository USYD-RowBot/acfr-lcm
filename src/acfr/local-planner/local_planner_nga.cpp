#include "local_planner_nga.hpp"
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

/** *************************************************************************
 *
 * LocalPlanner
 *
 */
LocalPlannerTunnel::LocalPlannerTunnel() :
    LocalPlanner()
{
	gpState.state = acfrlcm::auv_global_planner_state_t::IDLE;

	fp.open("/tmp/log_waypoint.txt", ios::out);
	fp_wp.open("/tmp/log_waypoint_now.txt", ios::out);

	cout << endl << endl << endl << "LocalPlannerNGA started" << endl;
}

// this is so the parent class destructor is called
// do NOT delete
LocalPlannerTunnel::~LocalPlannerTunnel()
{
}

/**
 * Call Dubins path planner to generate a path from current location to the
 * destination waypoint
 */
int LocalPlannerTunnel::calculateWaypoints()
{

	// Get a copy of curr and dest pose
	Pose3D currPose = this->currPose;
	Pose3D destPose = this->destPose;
	//double currVel = this->currVel[0];

	cout << timestamp_now() << " Calculating new waypoints..." << endl;
	cout << "CurrPose=" << currPose.getX() << ","
			<< currPose.getY() << "," << currPose.getZ() << " < "
			<< currPose.getYawRad() / M_PI * 180 << endl;
	cout << "DestPose=" << destPose.getX() << ","
			<< destPose.getY() << "," << destPose.getZ() << " < "
			<< destPose.getYawRad() / M_PI * 180  << endl;

	Pose3D destPoseRel = getRelativePose(destPose);
	double relAngle = atan2( destPoseRel.getY(), destPoseRel.getX() );
	cout << "Dest rel: X=" << destPoseRel.getX()
			<< ", angle=" << relAngle/M_PI*180 << endl;

	bool success = false;
	vector<Pose3D> wps;

	// If the waypoint is just ahead of us no need to use Dubins. We will rely
	//	on the controller to get us there. Use controller for aborted ascent.
	if ((((destPoseRel.getX() < 0 ||   destPoseRel.getX() > 4*turningRadius) && this->destID > 0) && !aborted) && !holdMode)
	{
		DubinsPath dp;
		dp.setCircleRadius(turningRadius);
		dp.setMaxPitch(maxPitch);
		dp.setWaypointDropDist(wpDropDist);
		dp.setWaypointDropAngleFromDropDist();

		// Try to calculate a feasible Dubins path. If we fail we try
		//  a second time with twice the circle radius
		Pose3D startPoseRel = getRelativePose(oldPose);

		if (startPoseRel.getX() < 10)
			wps = dp.calcPath(oldPose, destPose); //path from start waypoint (if after hold then use last dubins waypoint) to end waypoint
		else
			wps = dp.calcPath(currPose, destPose); // path from current position to end waypoint

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

	printWaypoints();
	publishWaypoints();

	return success;
}


int LocalPlannerTunnel::loadConfig(char *program_name)
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


/**
 * Look at the next waypoint and
 * 1) remove it from the list if we have reached it or
 * 2) calculate commands for the controller
 */
int LocalPlannerTunnel::processWaypoints()
{
    static double depth_ref = 0.0;

	// Nothing to do
	if (waypoints.size() == 0)
		return 0;


	// Get the next waypoint
	Pose3D wp = waypoints.at(0);
	// bound check for alt
	Pose3D awp = waypoints.at(0);

	Pose3D adp = destPose;
	
	// when executing abort, check if we are on the surface
	if((aborted) && (currPose.getZ() < 1e-4)) 
		destPose.setPosition(currPose.getX(), currPose.getY(), currPose.getZ());

	// // this is just to do the bounding box check
	// if (getDepthMode() == acfrlcm::auv_control_t::ALTITUDE_MODE)
	// {
 //        if(depth_ref < 1e-4)
	// 		awp.setZ(depth_ref);
	// 	adp.setZ(depth_ref);
	// }
	// wp = waypoints.at(0);
	// We have reached the next waypoint
	if (pointWithinBound(awp) && !aborted)
	{
		printf( "[%3.2f, %3.2f, %3.2f] reached.\n",
				wp.getX(),
				wp.getY(),
				wp.getZ() );
		oldPose = awp; // save destpose for use in dubins next time
		waypoints.erase(waypoints.begin());
		resetWaypointTime(timestamp_now());
		if (!(waypoints.size() == 0))
			printWaypoints();
		// No more waypoints to process
		else
		{
			cout << timestamp_now() << " No more waypoints!" << endl;

			if ((pointWithinBound(adp)) && (gpState.state == acfrlcm::auv_global_planner_state_t::PAUSE))
			{
				cout << "Reached hold location" << endl;
			}
			else if ((pointWithinBound(adp)) && !holdMode)
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
	double desHeading = atan2(wp.getY() - currPose.getY(),
			wp.getX() - currPose.getX());

	// Calculate desired velocity. This is set to dest velocity by default
	double desVel = destVel;
	// Ramp down the velocity when close to the destination
    //double distToDest = getDistToDest();
	// 	cout << "desVel = " << destVel << ", distToDest = " << distToDest << endl;
	// if( distToDest < velChangeDist ) {
	// 	desVel = destVel * (distToDest / velChangeDist);
	// 	cout << desVel << " = " << destVel << " *( " << distToDest << " / " << velChangeDist << ")" << endl;
	// }

	// form a message to send
	acfrlcm::auv_control_t cc;
	cc.utime = timestamp_now();
	cc.run_mode = acfrlcm::auv_control_t::RUN;
	cc.heading = desHeading;
	//cc.vx = desVel;
	cc.altitude = 0;
	cc.pitch = 0;
    double curr_depth_ref;

    	// Use the obstacle avoidance altitude if available
    	double altitude;
	if(((timestamp_now() - oa.utime) < 5e6) && oa.altitude > 1e-4)
    	altitude = fmin(oa.altitude, navAltitude);
    else
	    altitude = navAltitude;

	if (getDepthMode() == acfrlcm::auv_path_command_t::DEPTH)
	{
		//cc.depth = wp.getZ();
        curr_depth_ref = wp.getZ();
        cc.altitude = altitude + currPose.getZ() - wp.getZ();
		cc.depth_mode = acfrlcm::auv_control_t::DEPTH_MODE;
	}
	else
	{
		// set the depth goal using the filtered desired altitude.
		//cc.depth = currPose.getZ() + (currAltitude - wp.getZ());
		curr_depth_ref = currPose.getZ() + (altitude - wp.getZ());
        cc.altitude = wp.getZ();
		cc.depth_mode = acfrlcm::auv_control_t::ALTITUDE_MODE;
	}

	// check we don't get closer to the bottom than our minimum
	double curr_alt_ref = currPose.getZ() + (altitude - minAltitude);
    if(curr_alt_ref < curr_depth_ref){
        curr_depth_ref = curr_alt_ref;
    }

	if(((((timestamp_now() - oa.utime) < 5e6) && oa.forward_distance > 0.4) && oa.altitude > 0.4) && currPose.getZ() > 2)
	{
		cc.vx = calcVelocity(desVel, cc.altitude);
	}
	else 
		cc.vx = desVel;
    // FIXME: limit the depth rate change to yield an achievable 
    // trajectory. This is modelled on a forward speed of 0.75m/s 
    // with a max pitch of 0.3rad.  This should be configurable or
    // calculated automatically.
 //    double NAV_DT = 0.1;
 //    double max_depth_ref_change = 0.2*NAV_DT;
 //    double depth_ref_error = curr_depth_ref - depth_ref;
	// if (depth_ref_error > max_depth_ref_change)
	// 	depth_ref += max_depth_ref_change;
	// else if (depth_ref_error < -max_depth_ref_change)
	// 	depth_ref -= max_depth_ref_change;
	// else

	// cc.pitch = -atan((oa.altitude - navAltitude)/1.5);


	//setting up simple linear regression to determine pitch angle based on oa and rdi altitude readings
	vector<double> x;
	vector<double> z;
	//if oa data is good then we use it
	if(((timestamp_now() - oa.utime) < 5e6) && oa.altitude > 1e-4)
	{
		x.push_back(1250.0);
		z.push_back(-oa.altitude);
	}
	//we should always have the nav altitude, currently the nav alt is only based off the rdi, change this in future is oa alt is piped into nav alt
	x.push_back(0.0);
	z.push_back(-navAltitude);
	//rdi sends out 4 beams at 30 degrees, if the data for a beam is good we find the alt at the beam end and the x distance from veh origin
    for (int i = 0; i < 4; i++)
    {
    	if(rdi[i] > 1e-4)
    	{
    		z.push_back(-rdi[i]*cos((30/180)*M_PI)); // trig to go from range to alt
    		if(i == 0 || i == 3)
    			x.push_back(-rdi[i]*cos((30/180)*M_PI)*sin((30/180)*M_PI)); // trig to go from alt to x dist
    		else
    			x.push_back(rdi[i]*cos((30/180)*M_PI)*sin((30/180)*M_PI)); // or x dist in opp direction
    	}
    }
    double n = x.size();
    if(n>1){
    	double avgX = accumulate(x.begin(), x.end(), 0.0) / n; //avg of x values
	    double avgZ = accumulate(z.begin(), z.end(), 0.0) / n; //average of z values

	    double numerator = 0.0;
	    double denominator = 0.0;

	    for(int i=0; i<n; ++i){
	        numerator += (x[i] - avgX) * (z[i] - avgZ);
	        denominator += (x[i] - avgX) * (x[i] - avgX);
	    }

	    cc.pitch = atan2(numerator , denominator); // check this direction is correct in real life
    }

	// // if we are likely to run aground then re calc the waypoints so we don't
	// if (fabs(depth_ref - curr_depth_ref) > 1e-3)
	// {
	 	depth_ref = curr_depth_ref;
	// 	destPose.setZ(depth_ref);
	// 	cc.pitch = -atan((oa.altitude - navAltitude)/1.5); 
	// }

	if (aborted)
		cc.depth = destPose.getZ();
	else
		cc.depth = depth_ref;

	lcm.publish(vehicle_name+".AUV_CONTROL", &cc);
	return 1;
}


/**
 * Copy current nav solution and process the waypoints
 */

int LocalPlannerTunnel::onNav(const acfrlcm::auv_acfr_nav_t *nav)
{

	currPose.setPosition(nav->x, nav->y, nav->depth);
	navAltitude = nav->altitude;

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
	if (gpState.state == acfrlcm::auv_global_planner_state_t::RUN || gpState.state ==  acfrlcm::auv_global_planner_state_t::ABORT 
		|| gpState.state ==  acfrlcm::auv_global_planner_state_t::PAUSE)
	{
		processWaypoints();
	}

	return 1;
}


/**
 * We have received a new path command.
 * Let's calcualte a new path.
 */
 
int LocalPlannerTunnel::onPathCommand(const acfrlcm::auv_path_command_t *pc)
{
	if (!getDestReached())
		oldPose = destPose; // we were asked to skip a waypoint so update the oldPose
	
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
				<< "Can't calculate a feasible path. Let's cruise and see what happens"
				<< "----------------------------------------" << endl << endl;
	}
	resetWaypointTime(timestamp_now());

	return status;
}

int LocalPlannerTunnel::init()
{
}

int LocalPlannerTunnel::execute_abort()
{
	cout << "Executing an abort" << endl;
	cout << "Abort position at " << currPose.getX() << " , " << currPose.getY() << endl;
	abortPose.setX(NAN);
	abortPose.setY(NAN);
	abortPose.setZ(-1.0);	//set destination depth 
	destPose = abortPose;
	depthMode = 0;
	destVel = 0;
	waypoints.clear();
	setNewDest(true);
	
	destID = -99;
	aborted = true;	

	if (currPose.getZ() > 0.2)
		calculateWaypoints();

	return 1;
}

