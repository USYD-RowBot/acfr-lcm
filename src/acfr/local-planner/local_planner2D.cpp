#include "local_planner2D.hpp"
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
#include "perls-lcmtypes++/acfrlcm/wam_v_control_t.hpp"
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
		const acfrlcm::auv_acfr_nav_t *nav, LocalPlanner2D *lp)
{
	lp->onNav(nav);
}

// Path command callback
void onPathCommandLCM(const lcm::ReceiveBuffer* rbuf,
		const std::string& channel, const acfrlcm::auv_path_command_t *pc,
		LocalPlanner2D *lp)
{
	lp->onPathCommand(pc);
}

// Global State command callback
void onGlobalStateLCM(const lcm::ReceiveBuffer* rbuf,
		const std::string& channel,
		const acfrlcm::auv_global_planner_state_t *gpState, LocalPlanner2D *lp)
{
	lp->onGlobalState(gpState);
}

// heartbeat callback
void recalculate(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
		const perllcm::heartbeat_t *heartbeat, LocalPlanner2D *lp)
{
	// We haven't reached our goal but don't have any more waypoints.
	// 	Let's recalculate a feasible path.
	// Addition: Make the replanning stage occur at a set period
	static long count = 0;

	if ((lp->getDestReached() == false && lp->getNewDest() == true) &&
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
 * LocalPlanner2D
 *
 */
LocalPlanner2D::LocalPlanner2D() :
		startVel(0), destVel(0), newDest(false), destReached(false),
		destID(0), turningRadius(1.0),
		lookaheadVelScale(0), maxDistFromLine(0), maxAngleFromLine(0),
		velChangeDist(1.0), defaultLegVel(1.0), waypointTimeout(1e3),
		forwardBound(0.5), sideBound(2.0), distToDestBound(2.0),
		maxAngleWaypointChange(0.0), radiusIncrease(1.0), maxAngle(300),
		wpDropDist(4.0), wpDropAngle(M_PI / 18.), replanInterval(1.5),
		waypointTime(timestamp_now()), replanTime(timestamp_now())
{
	gpState.state = acfrlcm::auv_global_planner_state_t::IDLE;

	fp.open("/tmp/log_waypoint.txt", ios::out);
	fp_wp.open("/tmp/log_waypoint_now.txt", ios::out);

	cout << endl << endl << endl << "LocalPlanner2D started" << endl;
}

LocalPlanner2D::~LocalPlanner2D()
{

	if (fp.is_open())
		fp.close();
	if (fp_wp.is_open())
		fp_wp.close();
}

int LocalPlanner2D::subscribeChannels()
{
	// sunscribe to the required LCM messages
	lcm.subscribeFunction(vehicle_name+".ACFR_NAV", onNavLCM, this);
	lcm.subscribeFunction(vehicle_name+".PATH_COMMAND", onPathCommandLCM, this);
	lcm.subscribeFunction(vehicle_name+".GLOBAL_STATE", onGlobalStateLCM, this);
	lcm.subscribeFunction("HEARTBEAT_5HZ", recalculate, this);
        return 1;
}


/**
 * Copy current nav solution and process the waypoints
 */
int LocalPlanner2D::onNav(const acfrlcm::auv_acfr_nav_t *nav)
{
	currPose.setPosition(nav->x, nav->y);

	// Instead of heading, we make the current pose "heading" actually the slip
	// 	angle (bearing), for control
	double bearing = atan2(
			+nav->vy * cos(nav->heading) + nav->vx * sin(nav->heading),
			-nav->vy * sin(nav->heading) + nav->vx * cos(nav->heading));
	while (bearing < -M_PI)
		bearing += 2 * M_PI;
	while (bearing > M_PI)
		bearing -= 2 * M_PI;
	currPose.setThetaRad(bearing);
	currVel = nav->vx, nav->vy;

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
int LocalPlanner2D::onPathCommand(const acfrlcm::auv_path_command_t *pc)
{
	bool status = false;
	// Reset destination pose
	destPose.setIdentity();

	// Set destination pose and velocity
	destPose.setPosition(pc->waypoint[0], pc->waypoint[1]);
	destPose.setThetaRad(pc->waypoint[5]);
	destVel = pc->waypoint[6];
	destID = pc->goal_id;

	cout << timestamp_now() << " Got a new DEST point " << endl << "\t"
			<< "DestPose=" << destPose.getX() << "," << destPose.getY() << ","
			<< destPose.getThetaRad() / M_PI * 180
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

/**
 * Call Dubins path planner to generate a path from current location to the
 * destination waypoint
 */
int LocalPlanner2D::calculateWaypoints()
{

	// Get a copy of curr and dest pose
	Pose2D currPose = this->currPose;
	Pose2D destPose = this->destPose;
	double currVel = this->currVel[0];

	cout << timestamp_now() << " Calculating new waypoints..." << endl;
	cout << setprecision(2) << "CurrPose=" << currPose.getX() << ","
			<< currPose.getY() << "," << " < "
			<< currPose.getThetaRad() / M_PI * 180 << endl;
	cout << setprecision(2) << "DestPose=" << destPose.getX() << ","
			<< destPose.getY() << "," << " < "
			<< destPose.getThetaRad() / M_PI * 180 << endl;

	Pose2D destPoseRel = getRelativePose(destPose);
	double relAngle = atan2( destPoseRel.getY(), destPoseRel.getX() );
	cout << "Dest rel: X=" << destPoseRel.getX() << " Y=" << destPoseRel.getY()
			<< ", angle=" << relAngle/M_PI*180 << endl;

	bool success = false;
	vector<Pose2D> wps;

	// If the waypoint is just ahead of us no need to use Dubins. We will rely
	//	on the controller to get us there.
	if (destPoseRel.getX() < 0 ||
	    destPoseRel.getX() > 2*turningRadius ||
		fabs(relAngle) > 45./180*M_PI )
	{
		DubinsPath2D dp;
		dp.setCircleRadius(turningRadius);
		dp.setWaypointDropDist(wpDropDist);
		dp.setWaypointDropAngleFromDropDist();

		// Don't use current pose but a pose looking a bit ahead. We adjust this
		//	by employing the current velocity
		double lookAheadTime = 0.2; // [s]
		Pose2D lookAheadPose;
		// Using fabs of vel to ensure that even if we are going backwards the
		// lookaheadpose is in front of us
		lookAheadPose.setX(fabs(currVel) * lookAheadTime);
		Pose2D startPose = currPose.compose(lookAheadPose);

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
int LocalPlanner2D::onGlobalState(
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
		acfrlcm::wam_v_control_t cc;
		cc.utime = timestamp_now();
		cc.run_mode = acfrlcm::wam_v_control_t::STOP;
		lcm.publish(vehicle_name+".WAM_V_CONTROL", &cc);
	}
	return 1;
}

int LocalPlanner2D::loadConfig(char *program_name)
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

int LocalPlanner2D::process()
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

/**
 * Look at the next waypoint and
 * 1) remove it from the list if we have reached it or
 * 2) calculate commands for the controller
 */
int LocalPlanner2D::processWaypoints()
{

	// Nothing to do
	if (waypoints.size() == 0)
	{
		return 0;
	}

	// Get the next waypoint
	Pose2D wp = waypoints.at(0);

	// We have reached the next waypoint
	if (pointWithinBound(wp))
	{

		printf( "[%3.2f, %3.2f] reached.\n",
				wp.getX(),
				wp.getY() );
		waypoints.erase(waypoints.begin());
		resetWaypointTime(timestamp_now());

		printWaypoints();

		// No more waypoints to process
		if (waypoints.size() == 0)
		{
			cout << timestamp_now() << " No more waypoints!" << endl;

			if (pointWithinBound(destPose))
			{
				setDestReached(true);
				cout << "We have reached our destination :)" << endl;
			}

//			// form a STOP message to send
//			acfrlcm::wam_v_control_t cc;
//			cc.utime = timestamp_now();
//			cc.run_mode = acfrlcm::wam_v_control_t::STOP;
//			lcm.publish("AUV_CONTROL", &cc);

			return getDestReached();
		}

	}

	Pose2D currPose = getCurrPose();

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
	acfrlcm::wam_v_control_t cc;
	cc.utime = timestamp_now();
	cc.run_mode = acfrlcm::wam_v_control_t::RUN;
	cc.heading = desHeading;
	cc.vx = desVel;

        //cout << "wpX: " << wp.getX() << " wpY:" << wp.getY() << " cX:" << currPose.getX() << " cY:" << currPose.getY() << " dH:" << desHeading << " dV:" << desVel << endl;

	lcm.publish(vehicle_name+".WAM_V_CONTROL", &cc);
	return 1;
}

int LocalPlanner2D::sendResponse()
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

void LocalPlanner2D::printWaypoints() const {
	cout << "waypoints = [" << endl;
	for (unsigned int i = 0; i < waypoints.size(); i++)
	{
		printf( "%3.2f, %3.2f;\n",
				waypoints.at(i).getX(),
				waypoints.at(i).getY());
	}
	cout << "];" << endl;
}

bool LocalPlanner2D::publishWaypoints() {
	acfrlcm::auv_local_path_t lp;
	lp.utime = timestamp_now();
	unsigned int i;
	// TODO: comparison between signed and unsiged!
	for( i = 0; i < waypoints.size() && i < lp.max_num_el; i++  ) {
		lp.x[i] = waypoints[i].getX();
		lp.y[i] = waypoints[i].getY();
	}
	lp.num_el = i;
	lcm.publish(vehicle_name+".LOCAL_PATH", &lp);
	return true;
}

