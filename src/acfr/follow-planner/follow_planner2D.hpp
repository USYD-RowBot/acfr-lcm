#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <small/linalg.hh>
#include <small/Pose2D.hh>
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_path_command_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_global_planner_state_t.hpp"
#include "acfr-mission/GotoPath.h"

#pragma once


#define DOUBLE_MAX std::numeric_limits< double >::max()
#define ITERATION_NUM 3

using namespace std;
using namespace SMALL;

class FollowPlanner2D
{
public:
	FollowPlanner2D();
	~FollowPlanner2D();
        int subscribeChannels();
	int loadConfig(char *programName);
	int onNav(const acfrlcm::auv_acfr_nav_t *nav);
	int onTargetNav(const acfrlcm::auv_acfr_nav_t *nav);
	int onGlobalState(const acfrlcm::auv_global_planner_state_t *gpState);
	int calculateWaypoint();
	int processWaypoints();
	int process();

	Pose2D getCurrPose(void) const
	{
		return currPose;
	}
	Pose2D getTargetPose(void) const
	{
		return targetPose;
	}

	Vector2D getCurrVel(void) const
	{
		return currVel;
	}
	double getTargetVel(void) const
	{
		return targetVel;
	}


	Pose2D getRelativePose( Pose2D p ) {
		Pose2D pRel = getCurrPose().inverse().compose(p);
		return pRel;
	}

	bool getDestReached(void) const
	{
		return destReached;
	}
	void setDestReached(bool b)
	{
		destReached = b;
	}

	bool getNewDest(void) const
	{
		return newDest;
	}
	void setNewDest(bool b)
	{
		newDest = b;
	}

	double getReplanInterval(void) const
	{
		return replanInterval;
	}

	double getWaypointTimeout(void) const
	{
		return waypointTimeout;
	}
	void resetWaypointTime(int64_t t)
	{
		waypointTime = t;
	}
	int64_t getWaypointTime(void) const
	{
		return waypointTime;
	}
	double getWaypointTimePassedSec(void) const
	{
		return (double) (timestamp_now() - getWaypointTime()) / 1000000.;
	}

	void resetReplanTime(int64_t t)
	{
		replanTime = t;
	}
	int64_t getReplanTime(void)
	{
		return replanTime;
	}

	void printWaypoint( void ) const;
	bool publishWaypoint(void);

        void setVehicleName(char *vn)
        {
                vehicle_name = vn;
        }
        void setTargetName(char *vn)
        {
                target_name = vn;
        }

	GotoPath gp;

	lcm::LCM lcm;

private:

	bool pointWithinBound(Pose2D p)
	{
		Pose2D pRel = getRelativePose(p);

		/*if ((pRel.getX() < forwardBound) &&
			(pRel.getX() > -2 * forwardBound) &&
			(std::fabs(pRel.getY()) < sideBound))
		{
			return true;
		}*/

		return false;
	}

	Pose2D currPose;
	Vector2D currVel;
	double currAltitude;

	Pose2D targetPose;
	double targetVel;

	// New destination from GLOBAL
	bool newDest;
	// Global destination
	bool destReached;

	int destID;

	acfrlcm::auv_global_planner_state_t gpState;

	double waypointTimeout;
	double distToTargetBound;
	double replanInterval;

	int64_t waypointTime;
	int64_t replanTime;

        string vehicle_name = "DEFAULT";
        string target_name = "ASV_TARGET";

};
