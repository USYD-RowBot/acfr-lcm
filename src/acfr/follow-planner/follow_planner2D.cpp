#include "follow_planner2D.hpp"
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
#include "perls-lcmtypes++/acfrlcm/auv_global_planner_t.hpp"
#include "perls-lcmtypes++/acfrlcm/wam_v_control_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_local_path_t.hpp"

#include "acfr-mission/missionschema.hxx"

using namespace std;


// Exit handler, I swear this is the only global
bool mainExit;
void signalHandler(int sig)
{
	mainExit = true;
}

// Nav callback
void onNavLCM(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
		const acfrlcm::auv_acfr_nav_t *nav, FollowPlanner2D *fp)
{
	fp->onNav(nav);
}

// Nav callback
void onTargetNavLCM(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
		const acfrlcm::auv_acfr_nav_t *nav, FollowPlanner2D *fp)
{
	fp->onTargetNav(nav);
}


// Global State command callback
void onGlobalStateLCM(const lcm::ReceiveBuffer* rbuf,
		const std::string& channel,
		const acfrlcm::auv_global_planner_state_t *gpState, FollowPlanner2D *fp)
{
	fp->onGlobalState(gpState);
}

// heartbeat callback
void recalculate(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
		const perllcm::heartbeat_t *heartbeat, FollowPlanner2D *fp)
{
     static int counter = 0;
     int period = 10; 
     if (counter++ >= period)
     {
         fp->calculateWaypoint();
         counter = 0;
     }
}

/** *************************************************************************
 *
 * FollowPlanner2D
 *
 */
FollowPlanner2D::FollowPlanner2D() :
		targetVel(0), newDest(false), destReached(false),
		destID(0), 
		waypointTimeout(1e3),
		replanInterval(1.5),
		waypointTime(timestamp_now()), replanTime(timestamp_now())
{
	gpState.state = acfrlcm::auv_global_planner_state_t::IDLE;

	cout << endl << endl << endl << "FollowPlanner2D started" << endl;
}

FollowPlanner2D::~FollowPlanner2D()
{
}

int FollowPlanner2D::subscribeChannels()
{
	// sunscribe to the required LCM messages
	lcm.subscribeFunction(vehicle_name+".ACFR_NAV", onNavLCM, this);
	lcm.subscribeFunction(vehicle_name+".GLOBAL_STATE", onGlobalStateLCM, this);
	lcm.subscribeFunction("HEARTBEAT_1HZ", recalculate, this);

	lcm.subscribeFunction(target_name+".ACFR_NAV", onTargetNavLCM, this);
        return 1;
}


/**
 * Copy current nav solution and process the waypoints
 */
int FollowPlanner2D::onNav(const acfrlcm::auv_acfr_nav_t *nav)
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
	currVel = nav->vx;

	return 1;
}

/**
 * Copy target nav solution and process the waypoints
 */
int FollowPlanner2D::onTargetNav(const acfrlcm::auv_acfr_nav_t *nav)
{
	targetPose.setPosition(nav->x, nav->y);

	// Instead of heading, we make the current pose "heading" actually the slip
	// 	angle (bearing), for control
	double bearing = atan2(
			+nav->vy * cos(nav->heading) + nav->vx * sin(nav->heading),
			-nav->vy * sin(nav->heading) + nav->vx * cos(nav->heading));
	while (bearing < -M_PI)
		bearing += 2 * M_PI;
	while (bearing > M_PI)
		bearing -= 2 * M_PI;
	targetPose.setThetaRad(bearing);
	targetVel = nav->vx;

	return 1;
}


/**
 * Calculate a waypoint to follow the target 
 */
int FollowPlanner2D::calculateWaypoint()
{
        int Rmin = 30;
        int Rmax = 80;
        int Rtarget = (Rmax + Rmin)/2;

	// Get a copy of curr and dest pose
	Pose2D currPose = this->currPose;
	Pose2D targetPose = this->targetPose;
	double currVel = this->currVel[0];

	cout << timestamp_now() << " Calculating new waypoint..." << endl;
	cout << setprecision(2) << "CurrPose=" << currPose.getX() << ","
			<< currPose.getY() << "," << " < "
			<< currPose.getThetaRad() / M_PI * 180 << endl;
	cout << setprecision(2) << "TargetPose=" << targetPose.getX() << ","
			<< targetPose.getY() << "," << " < "
			<< targetPose.getThetaRad() / M_PI * 180 << endl;

	Pose2D targetPoseRel = getRelativePose(targetPose);
	//double relAngle = atan2( -targetPoseRel.getY(), -targetPoseRel.getX() );
	double relAngle = atan2( -targetPose.getY() + currPose.getY(), -targetPose.getX() + currPose.getX() );
        double relDist = sqrt( pow(targetPoseRel.getX(), 2) + pow(targetPoseRel.getY(), 2));
        cout << "dX=" << -targetPose.getX() + currPose.getX() << ", dY=" << -targetPose.getY() + currPose.getY() << endl;
	cout << "Target rel: X=" << targetPoseRel.getX() << " Y=" << targetPoseRel.getY()
			<< ", angle=" << relAngle/M_PI*180 << ", dist=" << relDist << endl;

        if (relDist < Rmin || relDist > Rmax)
        {
            Pose2D gotoPose(targetPose.getX() + Rtarget*cos(relAngle),
                targetPose.getY() + Rtarget*sin(relAngle), relAngle);
            gp.setPose(gotoPose);
            gp.setHeadingRad(relAngle+M_PI/2);
            gp.setTimeout(100);
            gp.setVelocity(max(targetVel, 1.5));

            printWaypoint();
            publishWaypoint();
        }

	return true;
}

/**
 * Account for changes in system state
 */
int FollowPlanner2D::onGlobalState(
		const acfrlcm::auv_global_planner_state_t *gpStateLCM)
{
	cout << "Change of global state to: " << (int) (gpStateLCM->state) << endl;
	gpState = *gpStateLCM;

	return 1;
}

int FollowPlanner2D::loadConfig(char *program_name)
{
	BotParam *param = NULL;
	param = bot_param_new_from_server(lcm.getUnderlyingLCM(), 1);
	if (param == NULL
	)
		return 0;

/*
	char rootkey[64];
	char key[128];
	sprintf(rootkey, "acfr.%s", program_name);

	sprintf(key, "%s.waypoint_timeout", rootkey);
	waypointTimeout = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.dist_to_dest_bound", rootkey);
	distToTargetBound = bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.replan_interval", rootkey);
	replanInterval = bot_param_get_double_or_fail(param, key);
*/
	return 1;
}

int FollowPlanner2D::process()
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

void FollowPlanner2D::printWaypoint() const {
	/*cout << "waypoint = [" << endl;
	gp.printPath();
	cout << "];" << endl;*/
}

bool FollowPlanner2D::publishWaypoint() {

    //
    // Create the mission object and associated global parameters
    //
    ::acfr::missions::lat latitude(39);
    ::acfr::missions::lon longitude(-15);
    ::acfr::missions::latlonelement_t loc(latitude, longitude);
    ::acfr::missions::m dist(100);
    ::acfr::missions::distanceelement_t tradius(dist);
    ::acfr::missions::distanceelement_t ddistance(dist);
    ::acfr::missions::angleelement_t dangle;
    ::acfr::missions::timeelement_t mtimeout(1000);
    ::xml_schema::string desc;
    ::acfr::missions::globals_t global(loc, tradius, ddistance, dangle, mtimeout, desc);
    ::acfr::missions::mission_t missionXML(global);// = gp.toXML();

    //
    // Retrieve a reference to the list of mission primitives
    //
    ::acfr::missions::mission_t::primitive_sequence& ps(missionXML.primitive());

    //
    // Create some mission primitives and add to the mission list
    //
    ::acfr::missions::commandprimitive_t gpXML = gp.toXML();
    ps.push_back(gpXML);

    //
    // Prepare namespace mapping and schema location information.
    //
    xml_schema::namespace_infomap map;

    map[""].name = "http://auv.acfr.usyd.edu.au/missions/";
    map[""].schema = "missionschema.xsd";

    // Convert to a string an write it out.
    //
    std::stringstream mission_string;
    ::acfr::missions::mission (mission_string, missionXML, map);

    acfrlcm::auv_global_planner_t gp_msg;
    gp_msg.utime = timestamp_now();
    gp_msg.str = mission_string.str();
    gp_msg.command = acfrlcm::auv_global_planner_t::GOTO;
    cout << "Sending new GOTO mission" << endl;
    cout << gp_msg.str << endl;
    lcm.publish(vehicle_name+".TASK_PLANNER_COMMAND", &gp_msg);

    return true;
}

