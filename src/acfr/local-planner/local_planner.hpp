#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <fstream>

#include "DubinsPath.h"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_path_command_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_global_planner_state_t.hpp"

#pragma once


#define DOUBLE_MAX std::numeric_limits< double >::max()
#define ITERATION_NUM 3


class LocalPlanner
{
public:
    LocalPlanner();
    virtual ~LocalPlanner();
    int subscribeChannels();
    virtual int loadConfig(char *programName);
    int onNav(const acfrlcm::auv_acfr_nav_t *nav);
    int onPathCommand(const acfrlcm::auv_path_command_t *pc);
    int onGlobalState(const acfrlcm::auv_global_planner_state_t *gpState);
    virtual int calculateWaypoints();
    virtual int processWaypoints();
    int process();
    int sendResponse();

    Pose3D getCurrPose(void) const
    {
        return currPose;
    }
    Pose3D getDestPose(void) const
    {
        return destPose;
    }
    Pose3D getStartPose(void) const
    {
        return startPose;
    }

    Vector3D getCurrVel(void) const
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

    int getDepthMode(void) const
    {
        return depthMode;
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

    // Convert pose to be relative to current pose. Note that the local coord
    //  has Z down where as global has Z up so a 180deg rotation is neededs
    Pose3D getRelativePose( Pose3D p ) {
        Pose3D rot; rot.setRollPitchYawRad(M_PI, 0, 0);
        Pose3D pRel = getCurrPose().compose(rot).inverse().compose(p);
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
    std::vector<Pose3D> waypoints;

    lcm::LCM lcm;
    std::ofstream fp, fp_wp;

protected:

    bool pointWithinBound(Pose3D p)
    {
        Pose3D pRel = getRelativePose(p);

        if ((pRel.getX() < forwardBound) &&
            (pRel.getX() > -2 * forwardBound) &&
            (std::fabs(pRel.getY()) < sideBound))
        {
            return true;
        }

        return false;
    }

    Pose3D currPose;
    Vector3D currVel;
    double currAltitude;

    Pose3D startPose;
    double startVel;

    Pose3D destPose;
    double destVel;

    // New destination from GLOBAL
    bool newDest;
    // Global destination
    bool destReached;

    int depthMode;

    int diveMode;
    int diveStage;

    int destID;

    acfrlcm::auv_global_planner_state_t gpState;

    double turningRadius;
    double minAltitude;
    double maxPitch;
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
    //double depth_ref;

    int64_t waypointTime;
    int64_t replanTime;

        string vehicle_name = "DEFAULT";

};
