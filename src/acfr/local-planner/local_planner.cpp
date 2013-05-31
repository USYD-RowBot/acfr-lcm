#include "local_planner.hpp"

// Exit handler, I swear this is the only global
int mainExit;
void signalHandler(int sig) {
    mainExit = 1;
}


// Nav callback
void onNavLCM(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const acfrlcm::auv_acfr_nav_t *nav, LocalPlanner *lp) 
{
    lp->onNav(nav);
}

// Path command callback
void onPathCommandLCM(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const acfrlcm::auv_path_command_t *pc, LocalPlanner *lp) 
{
    lp->onPathCommand(pc);
}

// heartbeat callback
void calculateLCM(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const perllcm::heartbeat_t *heartbeat, LocalPlanner *lp) 
{
    // We haven't reached our goal but don't have any more waypoints. Let's recalculate a feasible path
    if( lp->waypoints.size() == 0 && lp->getDestReached() == false && lp->getNewDest() == true ) {
        cout << endl << endl << "RECALCULATING a feasible path" << endl << endl;
        lp->calculateWaypoints();
    }
    
    lp->sendResponse();
}


LocalPlanner::LocalPlanner() : startVel(0), destVel(0), newDest(false), destReached(false), 
    destID(0), wpDropDist(4.0), turningRadius(0), maxPitch(0), 
    lookaheadVelScale(0), maxDistFromLine(0), maxAngleFromLine(0), velChangeDist(0), waypointCounter(0) {
    
    // sunscribe to the required LCM messages
    lcm.subscribeFunction("ACFR_NAV", onNavLCM, this);
    lcm.subscribeFunction("PATH_COMMAND", onPathCommandLCM, this);
    lcm.subscribeFunction("HEARTBEAT_1HZ", calculateLCM, this);
	
    
//    pthread_mutex_init(&currPoseLock, NULL);
//    pthread_mutex_init(&destPoseLock, NULL);
//    pthread_mutex_init(&waypointsLock, NULL);                      

}

LocalPlanner::~LocalPlanner() {
//    pthread_mutex_destroy(&currPoseLock);
//    pthread_mutex_destroy(&destPoseLock);
//    pthread_mutex_destroy(&waypointsLock);                      
}
/**
 * Copy current navigation status
 */
int LocalPlanner::onNav(const acfrlcm::auv_acfr_nav_t *nav) {

//    pthread_mutex_lock(&currPoseLock);
    currPose.setPosition( nav->x, nav->y, nav->depth );
    currPose.setRollPitchYawRad( nav->roll, nav->pitch, nav->heading );
    currVel = nav->vx, nav->vy, nav->vz;
//    pthread_mutex_unlock(&currPoseLock);
    processWaypoints();
    return 1;
}

/**
 * Copy new destination pose and calculate a new path to this point
 */
int LocalPlanner::onPathCommand(const acfrlcm::auv_path_command_t *pc) {
//    pthread_mutex_lock(&destPoseLock);
    // Reset destination pose
    destPose.setIdentity();
    
    // Set destination pose and velocity
    destPose.setPosition( pc->waypoint[0], pc->waypoint[1], pc->waypoint[2] );
    destPose.setRollPitchYawRad( pc->waypoint[3], pc->waypoint[4], pc->waypoint[5] );
    destVel = pc->waypoint[6];
    depthMode = pc->depth_mode;
    destID = pc->goal_id;
//    pthread_mutex_unlock(&destPoseLock);    
    
    cout << "\nGot a new DEST point " << endl;
    destReached = false;
    newDest = true;
    calculateWaypoints();
    processWaypoints();
    return 1;
}

/**
 * Call Dubins path planner to generate a path from current location to the destination waypoint
 */
int LocalPlanner::calculateWaypoints() {
    
    
    DubinsPath dp;
    dp.setCircleRadius( turningRadius );
    dp.setMaxPitch( maxPitch );
    dp.setWaypointDropDist( wpDropDist );
    
    // Get a copy of curr and dest pose
//    pthread_mutex_lock(&currPoseLock);
//    pthread_mutex_lock(&destPoseLock);
    Pose3D currPose = this->currPose;
    Pose3D destPose = this->destPose;
//    pthread_mutex_unlock(&currPoseLock);
//    pthread_mutex_unlock(&destPoseLock);
            
    cout << "CurrPose=" << currPose << endl;
    cout << "DestPose=" << destPose << endl;

    // Try to calculate a feasible Dubins path. If we fail we try
    //  a second time with twice the circle radius
    vector<Pose3D> waypoints = dp.calcPath( currPose, destPose );
        
    if( waypoints.size() == 0 ) {
        cerr << "Failed to calculate a feasible path using Dubins path" << endl;
        cerr << "Increasing the turn radius to " << turningRadius * 2 << " and trying again" << endl;
        
        dp.setCircleRadius( turningRadius * 2 );
        waypoints = dp.calcPath( currPose, destPose );
        if( waypoints.size() == 0 ) {
            cerr << "LocalPlanner failed to generate a feasible path" << endl;
            cerr << "\tfrom currPose: " << currPose.toString() << endl;
            cerr << "\tto destPose: " << destPose.toString() << endl;
    
            return 0;
        }
    }

    
    // Managed to calculate a path to destination
//    pthread_mutex_lock(&waypointsLock);
    this->waypoints = waypoints;    
//    pthread_mutex_unlock(&waypointsLock);

    // Save the start pose and lashstart velocity
    startPose = currPose;
    startVel  = currVel[0];
    
    destReached = false;
    
    cout << "waypoints = [" << endl;
    for( unsigned int i = 0; i < waypoints.size(); i++ ) {
        cout << waypoints.at(i).getX() << ", " << waypoints.at(i).getY() << ", " << waypoints.at(i).getZ() << ";" << endl; 
    }
    cout << "];" << endl;

    return 1;
}





int LocalPlanner::loadConfig(char *program_name)
{
    BotParam *param = NULL;
    param = bot_param_new_from_server (lcm.getUnderlyingLCM(), 1);
    if(param == NULL)
        return 0;

    char rootkey[64];        
    char key[128];
    sprintf (rootkey, "acfr.%s", program_name);

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
    waypointTimeout	 = bot_param_get_double_or_fail(param, key);

    return 1;
}

int LocalPlanner::sendResponse() {
    acfrlcm::auv_path_response_t pr;
    pr.utime = timestamp_now();
    pr.goal_id = destID;
    pr.are_we_there_yet = destReached;
    if(waypoints.size() > 2)
        pr.distance = waypoints.size() * wpDropDist;
    else
        pr.distance = currPose.positionDistance(destPose);
    lcm.publish("PATH_RESPONSE", &pr);
    
    return 1;
}

int LocalPlanner::process() {
    
    int fd = lcm.getFileno();
    fd_set rfds;
    while(!mainExit)
    {
        FD_ZERO (&rfds);
        FD_SET (fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int ret = select (fd + 1, &rfds, NULL, NULL, &timeout);
        if(ret > 0)
            lcm.handle();
    }
    
    return 1;
}


//static void *processWaypoints(void *u) {
int LocalPlanner::processWaypoints() {
    LocalPlanner *lp = this; //(LocalPlanner *)u;
        
//    while(!mainExit) {
    
//        pthread_mutex_lock(&lp->waypointsLock);
        unsigned int numWaypoints = lp->waypoints.size();
        if( numWaypoints == 0 ) {
			
//            pthread_mutex_unlock(&lp->waypointsLock);
            // SLEEP 0.1s
			lp->resetWaypointCounter();
//            usleep( 100000 );
            return 1;
        }
        
        // Get the first waypoint
        Pose3D waypoint = lp->waypoints.at(0);
//		pthread_mutex_unlock(&lp->waypointsLock);

		

        // Get current pose
//        pthread_mutex_lock(&lp->currPoseLock);
        Pose3D currPose = lp->getCurrPose();
        Vector3D currVel3D = lp->getCurrVel();
        double currVel = currVel3D[0];
//        pthread_mutex_unlock(&lp->currPoseLock);
        
        // Get destination pose
//        pthread_mutex_lock(&lp->destPoseLock);
        Pose3D destPose = lp->getDestPose();
        double destVel = lp->getDestVel();
//        pthread_mutex_unlock(&lp->destPoseLock);

        double distToDest = currPose.positionDistance( destPose );
                
        // Check if the way point is ahead of us.
        // Convert the pose of waypoint relative to curr pose
        //  if the x is negative it is behind us and we will 
        //  ignore it (delete it)
        Pose3D rot; rot.setRollPitchYawRad( M_PI, 0, 0 ); // Local coord has Z down where as global has Z up
        Pose3D wpRel = currPose.compose(rot).inverse().compose( waypoint );
        
        cout << endl << "====================" << endl;
        cout << "\nWe have " << numWaypoints << " waypoints " << endl;        
        cout << "Current pose is   : " << currPose.getX() << ", " << currPose.getY() << ", " << currPose.getZ() << " < " << currPose.getYawRad()/M_PI*180. << "deg" << endl;
        cout << "Next way point is : " << waypoint.getX() << ", " << waypoint.getY() << ", " << waypoint.getZ() << " < " << waypoint.getYawRad()/M_PI*180. << "deg" << endl;
        cout << "Relative pose is  : " << wpRel.getX() << ", " << wpRel.getY() << ", " << wpRel.getZ() << " < " << wpRel.getYawRad()/M_PI*180. << "deg" << endl;
        cout << "Dist to DEST is   : " << distToDest << endl;
        
       
		lp->incrementWaypointCounter();

		cout << "Time for waypoint (Timeout): " << lp->getWaypointCounter()*0.1 << " (" << lp->getWaypointTimeout() << ")" << endl;

        if(( wpRel.getX() < 0.2 && fabs(wpRel.getY()) < 2.0 )||(lp->getWaypointCounter() > lp->getWaypointTimeout()/0.1)) {
            // delete it from the list
//            pthread_mutex_lock(&lp->waypointsLock);
            lp->waypoints.erase( lp->waypoints.begin() );
            numWaypoints = lp->waypoints.size();
//			pthread_mutex_unlock(&lp->waypointsLock);
			lp->resetWaypointCounter();
            cout << "Ignored as it is behind us" << endl;
            
            if( numWaypoints == 0 ) {
                cout << "No more waypoints!" << endl;
                
                if( distToDest < 1.0 ) {
                    lp->setDestReached( true );
                    lp->setNewDest( false );
                    cout << "We have reached DEST :) " << endl;
                }
            }                
            return 1;
        }
        

        
        // Calculate desired heading to way point
        //  This is not our bearing to the point but a global angle
        
		double desHeading = atan2( waypoint.getY()-currPose.getY(), waypoint.getX()-currPose.getX() ); 
		
        

        // Calculate desired velocity. If within 5m from dest pose, start adjusting the velocity
        double desVel = 0;
        double distToDestWp = numWaypoints * lp->getWpDropDist();
        cout << "\ndistToDestWp=" << distToDestWp << endl;
        double velChDist = lp->getVelChangeDist();
        // Linear ramp of speed on last meter to the final way point
        if( distToDestWp < velChDist  ) { //( distToDest < velChDist ) {
            desVel =  currVel - (distToDest-velChDist) / velChDist * (destVel-currVel);
        }
        // Linear ramp up
//        else if( distFromStart < velChDist  && lp->startVel < 1e-3 ) {
//            desVel = lp->getDefaultLegVel();// * distFromStart / velChDist;
//        } 
        else {
            desVel = lp->getDefaultLegVel();
        }
        
        
        cout << "DESIRED" << endl;
        cout << "\tHEADING  = " << desHeading/M_PI*180 << "deg" << endl;
        cout << "\tVELOCITY = " << desVel << endl;
        cout << "\tDEPTH    = " << waypoint.getZ() << endl;
        
        // for a message to send
        acfrlcm::auv_control_t cc;
        cc.utime = timestamp_now();
		
		
        if( desVel < 1e-3 ) {
            // stop the motors
            cc.run_mode = acfrlcm::auv_control_t::STOP;
        }
        else {
            cc.run_mode = acfrlcm::auv_control_t::RUN;
            cc.heading = desHeading;
            if(lp->getDepthMode() == acfrlcm::auv_path_command_t::DEPTH)
            {
                cc.depth = waypoint.getZ();
                cc.depth_mode = acfrlcm::auv_control_t::DEPTH_MODE;
            }
            else
            {
                cc.altitude = waypoint.getZ();
                cc.depth_mode = acfrlcm::auv_control_t::ALTITUDE_MODE;
            }
            cc.vx = desVel;
        }
        // publish
        lp->lcm.publish("AUV_CONTROL", &cc);
        
        
//        usleep( 100000 );
        
        
        
//    }
    
    
    return 1;
}


int main(int argc, char **argv)
{
    // install the signal handler
    mainExit = 0;
    signal(SIGINT, signalHandler);
    
    LocalPlanner *lp = new LocalPlanner();
    if(!lp->loadConfig(basename(argv[0])))
    {
        cerr << "Failed loading config" << endl;
        return 0;
    }
    
//    pthread_t tidProcessWaypoints;
//    pthread_create(&tidProcessWaypoints, NULL, processWaypoints, lp);
//    pthread_detach(tidProcessWaypoints);
    
    while(!mainExit)
        lp->process();
    
//    pthread_join(tidProcessWaypoints, NULL);
    
    delete lp;
    
    return 1;
}
