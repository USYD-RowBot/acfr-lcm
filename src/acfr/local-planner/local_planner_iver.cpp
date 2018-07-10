#include "local_planner_iver.hpp"

/**
 * Copy current nav solution and process the waypoints
 */

LocalPlannerIver::LocalPlannerIver():LocalPlanner()
{
}


int LocalPlannerIver::onNav(const acfrlcm::auv_acfr_nav_t *nav)
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
int LocalPlannerIver::onPathCommand(const acfrlcm::auv_path_command_t *pc)
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

int LocalPlannerIver::init()
{
}


int LocalPlannerIver::execute_abort()
{
}
