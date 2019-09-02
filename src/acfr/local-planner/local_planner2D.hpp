#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <fstream>

#include "DubinsPath2D.hpp"
#include "acfr-common/timestamp.h"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_path_command_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_global_planner_state_t.hpp"

#pragma once


#define DOUBLE_MAX std::numeric_limits< double >::max()
#define ITERATION_NUM 3


class LocalPlanner2D
{
public:
	LocalPlanner2D();
	~LocalPlanner2D();
        int subscribeChannels();
	int loadConfig(char *programName);
	int onNav(const acfrlcm::auv_acfr_nav_t *nav);
	int onPathCommand(const acfrlcm::auv_path_command_t *pc);
	int onGlobalState(const acfrlcm::auv_global_planner_state_t *gpState);
	int calculateWaypoints();
	int processWaypoints();
	int process();
	int sendResponse();

	Pose2D getCurrPose(void) const
	{
		return currPose;
	}
	Pose2D getDestPose(void) const
	{
		return destPose;
	}
	Pose2D getStartPose(void) const
	{
		return startPose;
	}

	Vector2D getCurrVel(void) const
	{
		return currVel;
	}
	double getDestVel(void) const
	{
		return destVel;
	}
	double getStartVel(void) const
	{
		return startVel;
	}

	double getDefaultLegVel(void) const
	{
		return defaultLegVel;
	}
	double getVelChangeDist(void) const
	{
		return velChangeDist;
	}

	double getDistToDest(void) const {
		return waypoints.size() * wpDropDist;
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

	double getWpDropDist(void) const
	{
		return wpDropDist;
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

	void printWaypoints( void ) const;
	bool publishWaypoints(void);

        void setVehicleName(char *vn)
        {
                vehicle_name = vn;
        }
	std::vector<Pose2D> waypoints;

	lcm::LCM lcm;
	std::ofstream fp, fp_wp;

private:

	bool pointWithinBound(Pose2D p)
	{
		Pose2D pRel = getRelativePose(p);

		if ((pRel.getX() < forwardBound) &&
			(pRel.getX() > -2 * forwardBound) &&
			(std::fabs(pRel.getY()) < sideBound))
		{
			return true;
		}

		return false;
	}

	Pose2D currPose;
	Vector2D currVel;
	double currAltitude;

	Pose2D startPose;
	double startVel;

	Pose2D destPose;
	double destVel;

	// New destination from GLOBAL
	bool newDest;
	// Global destination
	bool destReached;

	int destID;

	acfrlcm::auv_global_planner_state_t gpState;

	double turningRadius;
	double lookaheadVelScale;
	double maxDistFromLine;
	double maxAngleFromLine;
	double velChangeDist;
	double defaultLegVel;
	double waypointTimeout;
	double forwardBound;
	double sideBound;
	double distToDestBound;
	double maxAngleWaypointChange;
	double radiusIncrease;
	double maxAngle;
	double wpDropDist;
	double wpDropAngle;
	double replanInterval;

	int64_t waypointTime;
	int64_t replanTime;

        string vehicle_name = "DEFAULT";

};
