#include "local_planner_sirius.hpp"


LocalPlannerSirius::LocalPlannerSirius():LocalPlanner()
{
	diffSteer = true;
}


int LocalPlannerSirius::onNav(const acfrlcm::auv_acfr_nav_t *nav)
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
	if (gpState.state == acfrlcm::auv_global_planner_state_t::RUN || gpState.state == acfrlcm::auv_global_planner_state_t::ABORT)
	{
		processWaypoints();
	}

	return 1;
}

/**
 * We have received a new path command.
 * Let's calcualte a new path.
 */
int LocalPlannerSirius::onPathCommand(const acfrlcm::auv_path_command_t *pc)
{
	bool status = false;
	// Reset destination pose
	destPose.setIdentity();

	if(pc->run_mode == acfrlcm::auv_path_command_t::HOLD)
	{
		holdPose = currPose;
		holdMode = true;
		return true;
	}
	else
		holdMode = false;
	
	double x, y, z, h;
	x = pc->waypoint[0];
	y = pc->waypoint[1];
	z = pc->waypoint[2];
	depthMode = pc->depth_mode;
	h = pc->waypoint[5];

	if(x == x && y == y && h != h)
		h = atan2(y - currPose.getY(), x - currPose.getX());
		
	if(z != z)
		z = 1.5;

	// Set destination pose and velocity
	destPose.setPosition(x, y, z);
	destPose.setRollPitchYawRad(0.0, 0.0, h);
	destVel = pc->waypoint[6];
	destID = pc->goal_id;

	cout << timestamp_now() << " Got a new DEST point " << endl << "\t"
			<< "DestPose=" << destPose.getX() << "," << destPose.getY() << ","
			<< destPose.getZ() << " < " << destPose.getYawRad() / M_PI * 180
			<< endl;

	setDestReached(false);
	setDestReachedLatched(false);
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

int LocalPlannerSirius::init()
{
	char *lh = "127.0.0.1";
	create_udp_send(&auv_udp, lh, AUV_MSG_PORT);
	this->lcm.subscribe("HEARTBEAT_1HZ", &LocalPlannerSirius::heartbeat_callback, this);
	
	return 1;
	
}

void LocalPlannerSirius::heartbeat_callback(const lcm::ReceiveBuffer*rbuf, const std::string& channel,
                            const perllcm::heartbeat_t *hb)
{
	// Publish the AUV message for the old system on a UDP socket
	char auv_str[256];
	char time_str[32];
	rovtime2dsltime_str(((double)timestamp_now())/1e6, time_str);
	
	double distance;
	if((destPose.getX() != destPose.getX()) && (destPose.getY() != destPose.getY()) && (destPose.getYawRad() != destPose.getYawRad()))
		distance = destPose.getZ() - currPose.getZ();
	else if ((destPose.getX() != destPose.getX()) && (destPose.getY() != destPose.getY()) && (destPose.getZ() != destPose.getZ()))
		distance = 0.0;
	else if (waypoints.size() > 2)
		distance = waypoints.size() * wpDropDist;
	else
		distance = currPose.positionDistance(destPose);
		
	int len = sprintf(auv_str,
			  "AUV %s %s %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %d %.3f %.3f %d %.3f %02X\n",
			  time_str, "SIRIUS", 
			  currPose.getX(),
			  currPose.getY(),
			  currPose.getZ(),
			  currPose.getYawRad() / M_PI * 180,
		      currPose.getPitchRad() / M_PI * 180,
			  currPose.getRollRad() / M_PI * 180,
			  currAltitude,
			  currPose.getZ(),
			  destID,
			  0.0,
			  distance,
			  destReachedLatched, 
			  0.0,
			  0);
	send_udp(&auv_udp, auv_str, len);
	
	
}

//#define NAN std::numeric_limits::quiet_NaN()
int LocalPlannerSirius::execute_abort()
{	
	cout << "Executing an abort" << endl;
	abortPose.setPosition(NAN, NAN, -1.0);
	abortPose.setRollPitchYawRad(NAN, NAN, NAN);
	destPose = abortPose;
	depthMode = 0;
	setNewDest(true);
	
	destID = -99;
	aborted = true;	
	return 1;
}


