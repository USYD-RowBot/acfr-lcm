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
    // Addition: Make the replanning stage occur at a set period
//        if( (lp->waypoints.size() == 0) && (lp->getDestReached() == false) && (lp->getNewDest() == true))// ||  (( timestamp_now() - lp->getReplanTime() )/1000000 > 5) ) // if we want faster replanning, or a different number than the heartbeat, this callback needs to be made faster and this variable can be used

    cout << endl <<"Timestamp for replanning pre:" << (timestamp_now() - lp->getReplanTime()) <<endl<< endl;
    if ((lp->waypoints.size() == 0) || (( timestamp_now() - lp->getReplanTime() )/1000000 > 3500))
    {


        cout << endl << endl << "RECALCULATING a feasible path" << endl << endl;
        lp->calculateWaypoints();

             lp->resetReplanTime(timestamp_now());
    }
    cout << endl <<"Timestamp for replanning:" << (timestamp_now() - lp->getReplanTime())/1000000 <<endl<< endl;
    lp->sendResponse();
}


LocalPlanner::LocalPlanner() : startVel(0), destVel(0), newDest(false), destReached(false), 
    destID(0), turningRadius(0), maxPitch(0), wpDropDist(4.0), forwardBound(0.5), sideBound(2.0), distToDestBound(2.0),
    lookaheadVelScale(0), maxDistFromLine(0), maxAngleFromLine(0), velChangeDist(0), waypointTime(timestamp_now()),
    maxAngleWaypointChange(0.0), radiusIncrease(1.0), maxAngle(300), diveMode(0), replanTime(timestamp_now()), wpDropAngle(M_PI/18.) {
    
    // sunscribe to the required LCM messages
    lcm.subscribeFunction("ACFR_NAV", onNavLCM, this);
    lcm.subscribeFunction("PATH_COMMAND", onPathCommandLCM, this);
    lcm.subscribeFunction("HEARTBEAT_1HZ", calculateLCM, this);

    fp.open("/tmp/log_waypoint.txt", ios::out);
    fp_wp.open("/tmp/log_waypoint_now.txt",ios::out);

    //    pthread_mutex_init(&currPoseLock, NULL);
    //    pthread_mutex_init(&destPoseLock, NULL);
    //    pthread_mutex_init(&waypointsLock, NULL);

}

LocalPlanner::~LocalPlanner() {
    
    fp.close();
    fp_wp.close();
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
    //    currPose.setRollPitchYawRad( nav->roll, nav->pitch, nav->heading );

    //instead of heading, we make the current pose "heading" actually the slip angle, for control

    double bearing = atan2(nav->vy*cos(nav->heading)+nav->vx*sin(nav->heading),-nav->vy*sin(nav->heading)+nav->vx*cos(nav->heading));

    while(bearing < -M_PI)
        bearing += 2*M_PI;
    while(bearing > M_PI)
        bearing -= 2*M_PI;

    currPose.setRollPitchYawRad( nav->roll, nav->pitch, bearing );

    currVel = nav->vx, nav->vy, nav->vz;
    //    pthread_mutex_unlock(&currPoseLock);
    
    // if required process the special dive mode
    if(diveMode)
        dive();
    else
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
    
    // special dive case, first we need to get off the surface
    if((currPose.getZ() < 0.2) && (depthMode == acfrlcm::auv_control_t::DEPTH_MODE) && (pc->waypoint[2] > 0)) {
        diveStage = 0;
        diveMode = 1;
        return 1;
    }

    destReached = false;
    newDest = true;
    calculateWaypoints();
    if(diveMode)
        dive();
    else
        processWaypoints();

    return 1;
}

#define DIVE_REV_VEL -0.5
#define DIVE_PICTH (10.0 / 180.0 * M_PI)
#define DIVE_DEPTH 0.5

int LocalPlanner::dive() {
    // we have a list of things we need to achieve to get off the surface
    acfrlcm::auv_control_t cc;
    cc.utime = timestamp_now();
    cc.run_mode = acfrlcm::auv_control_t::DIVE;
    cc.depth_mode = acfrlcm::auv_control_t::PITCH_MODE;
    cout << "Dive stage : " << diveStage << " vx: " << currVel[0] << " depth: " << currPose.getZ() << " pitch: " << currPose.getRotation().getPitchRad() / M_PI * 180 << endl;
    switch(diveStage) {
    case 0:
        // get upto reverse speed
        cc.vx = DIVE_REV_VEL;
        cc.pitch = 0;
        lcm.publish("AUV_CONTROL", &cc);

        if(currVel[0] < (DIVE_REV_VEL + 0.1))
            diveStage = 1;
        break;
        
    case 1:
        // pitch the tail down
        cc.vx = DIVE_REV_VEL;
        cc.pitch = DIVE_PICTH;
        lcm.publish("AUV_CONTROL", &cc);

        // go until we are under, keeping in mind out pressure sensor is at the front
        if(currPose.getZ() > DIVE_DEPTH)
            diveStage = 2;
        break;
        
    case 2:
        // level out
        cc.vx = DIVE_REV_VEL;
        cc.pitch = 0;
        lcm.publish("AUV_CONTROL", &cc);

        if(fabs(currPose.getRotation().getPitchRad()) < (5.0 / 180.0 * M_PI))
            diveStage = 3;
        break;

    case 3:
        // go back to zero velocity and hope we stay down log enough to start
        cc.vx = 0;
        cc.pitch = 0;
        lcm.publish("AUV_CONTROL", &cc);

        // have we finished
        if(fabs(currVel[0]) < 0.1) {
            diveMode = 0;
            diveStage = 4;
        }
        break;
    }
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
    dp.setWaypointDropAngle( wpDropAngle * M_PI/180.);
    
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

    // Try a few perturbations to the original path parameters

    // First try to change the turning radius

//    double pathLengths[ITERATION_NUM];
//    double testTurningRadius[ITERATION_NUM];
//    testTurningRadius[0] = turningRadius;
//    bool feasibleFound = false;

//    if (waypoints.size() != 0)
//    {
//        feasibleFound = true;
//        pathLengths[0] = dp.getPathLength();
//    }
//    else
//    {
//        pathLengths[0] = DOUBLE_MAX; // max this out
//    }

//    for (int i=0;i<ITERATION_NUM-1;i++)
//    {
//        testTurningRadius[i+1] = testTurningRadius[i] + radiusIncrease;
//        dp.setCircleRadius( testTurningRadius[i+1]);
//        waypoints = dp.calcPath( currPose, destPose );

//        if (waypoints.size() != 0)
//        {
//            feasibleFound = true;
//            pathLengths[i+1] = dp.getPathLength();
//        }
//        else
//        {
//            pathLengths[i+1] = DOUBLE_MAX; // max this out
//        }

//    }

//    //Check if we are going in a loop, if we are and we are trying to essentially go straight, and it looks like shifting the waypoint might help, try it to avoid the loop

//    destPose.getYawRad();
//    currPose.getYawRad();
//    double desHeading = atan2(destPose.getY()-currPose.getY(),destPose.getX()-currPose.getX());

//    if ( (dp.getMaxAngle() > maxAngle)&&( fabs(currPose.getYawRad()-desHeading)<maxAngleFromLine*M_PI/180 )&&( fabs(destPose.getYawRad()-desHeading)<maxAngleFromLine*M_PI/180 ) ) // if we are doing a loop, so that it only does this when way points are near aligned given the entry and exit headings, and poses
//    {
//        Pose3D destPoseNew = destPose;
//        double rOld,pOld,hOld;

//        destPoseNew.getRollPitchYawRad(rOld, pOld, hOld);

//        double xOld = destPoseNew.getX();
//        double yOld = destPoseNew.getY();
//        double zOld = destPoseNew.getZ();

//        double pathLengthsNew[ITERATION_NUM*ITERATION_NUM*ITERATION_NUM*ITERATION_NUM];
//        double paramsNew[ITERATION_NUM*ITERATION_NUM*ITERATION_NUM*ITERATION_NUM][4];
//        double turningRadiusNew;// = turningRadius
//        double headingNew;// = destPose[6]
//        double xNew;// = destPose[0]
//        double yNew;// = destPose[1]
//        int path_index = 0;
//        bool feasibleFoundNew = false;

//        for (int i=0;i<ITERATION_NUM;i++) // go through turning radius
//        {
//            for (int j = 0;j<ITERATION_NUM;j++) // go through heading angles
//            {
//                for (int k = 0;k<ITERATION_NUM;k++) // go through waypoint shifts X
//                {
//                    for (int l = 0;l<ITERATION_NUM;l++) // go through waypoint shifts Y
//                    {

//                        turningRadiusNew = turningRadius + i*radiusIncrease;
//                        headingNew = hOld - maxAngleWaypointChange*(M_PI/180) + j*2*maxAngleWaypointChange/(ITERATION_NUM-1)*(M_PI/180);
//                        xNew = xOld - maxDistFromLine + k*2*maxDistFromLine/(ITERATION_NUM-1);
//                        yNew = yOld - maxDistFromLine + l*2*maxDistFromLine/(ITERATION_NUM-1);

//                        destPoseNew.setPosition(xNew,yNew,zOld);
//                        destPoseNew.setRollPitchYawRad(rOld, pOld, headingNew);

//                        dp.setCircleRadius( turningRadiusNew);
//                        waypoints = dp.calcPath( currPose, destPoseNew );

//                        if (waypoints.size() != 0)
//                        {
//                            feasibleFoundNew = true;
//                            pathLengthsNew[path_index] = dp.getPathLength();
//                            paramsNew[path_index][0] = turningRadiusNew;
//                            paramsNew[path_index][1] = headingNew;
//                            paramsNew[path_index][2] = xNew;
//                            paramsNew[path_index][3] = yNew;
//                        }
//                        else
//                        {
//                            pathLengthsNew[path_index] = DOUBLE_MAX; // max this out
//                        }
//                        path_index++;
//                    }
//                }
//            }
//        }

//        if (feasibleFoundNew == true)
//        {
//            int bestIndex = getMin(pathLengthsNew,ITERATION_NUM*ITERATION_NUM*ITERATION_NUM*ITERATION_NUM);
//            dp.setCircleRadius(paramsNew[bestIndex][0]);
//            destPoseNew.setPosition(paramsNew[bestIndex][2],paramsNew[bestIndex][3],zOld);
//            destPoseNew.setRollPitchYawRad(rOld, pOld, paramsNew[bestIndex][1]);

//            cout << "best offsets: " << (paramsNew[bestIndex][0] - turningRadius) << " "<< (destPoseNew.getX() - destPose.getX()) << " "<< (destPoseNew.getY() - destPose.getY()) <<" "<<((destPoseNew.getYawRad()  - destPose.getYawRad())*180/M_PI)  << endl;
//            cout << "old:"<< (turningRadius) << " "<< (destPose.getX()) << " "<< (destPose.getY()) <<" "<<((destPose.getYawRad())*180/M_PI)  << endl;
//            cout << "new:"<< (paramsNew[bestIndex][0]) << " "<< (destPoseNew.getX()) << " "<< (destPoseNew.getY()) <<" "<<((destPoseNew.getYawRad())*180/M_PI)  << endl;
//            waypoints = dp.calcPath( currPose, destPoseNew );
//        }
//        else
//        {
//            cerr << "LocalPlanner failed to generate a feasible path" << endl;
//            cerr << "\tfrom currPose: " << currPose.toString() << endl;
//            cerr << "\tto destPose: " << destPose.toString() << endl;

//            return 0;
//        }

//    }
//    else
//    {
//        double bestRadius;

//        if (feasibleFound == true)
//        {
//            bestRadius = testTurningRadius[getMin(pathLengths,ITERATION_NUM)]; // get the minimum path length
//            cout << "bestRadius: " << bestRadius << endl;
//            dp.setCircleRadius( bestRadius);
//            waypoints = dp.calcPath( currPose, destPose );
//        }
//        else
//        {
//            cerr << "LocalPlanner failed to generate a feasible path" << endl;
//            cerr << "\tfrom currPose: " << currPose.toString() << endl;
//            cerr << "\tto destPose: " << destPose.toString() << endl;

//            return 0;
//        }
//    }


    // Managed to calculate a path to destination
    //    pthread_mutex_lock(&waypointsLock);
    this->waypoints = waypoints;
    //    pthread_mutex_unlock(&waypointsLock);

    // Save the start pose and start velocity
    startPose = currPose;
    startVel  = currVel[0];
    
    destReached = false;
    
    cout << "waypoints = [" << endl;
    for( unsigned int i = 0; i < waypoints.size(); i++ ) {
        cout << waypoints.at(i).getX() << ", " << waypoints.at(i).getY() << ", " << waypoints.at(i).getZ() << ";" << endl;
        fp << waypoints.at(i).getX() << " " << waypoints.at(i).getY() << " " << waypoints.at(i).getZ() << endl;

    }
    fp << 0 << " " << 0 << " " << 0 << endl;
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

    sprintf(key, "%s.forward_bound", rootkey);
    forwardBound	 = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.side_bound", rootkey);
    sideBound	 = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.drop_dist", rootkey);
    wpDropDist	 = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.dist_to_dest_bound", rootkey);
    distToDestBound	 = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.max_angle_waypoint_change", rootkey);
    maxAngleWaypointChange = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.radius_increase", rootkey);
    radiusIncrease = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.max_angle", rootkey);
    maxAngle = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.drop_angle", rootkey);
    wpDropAngle = bot_param_get_double_or_fail(param, key);


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
    //static void *processLCM(void *u) {
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
        lp->resetWaypointTime(timestamp_now());
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

    double wpDist = pow(pow(wpRel.getX(),2) + pow(wpRel.getY(),2),0.5);

    //cout << endl << "====================" << endl;
    //cout << "\nWe have " << numWaypoints << " waypoints " << endl;
    //cout << "Current pose is   : " << currPose.getX() << ", " << currPose.getY() << ", " << currPose.getZ() << " < " << currPose.getYawRad()/M_PI*180. << "deg" << endl;
    //cout << "Next way point is : " << waypoint.getX() << ", " << waypoint.getY() << ", " << waypoint.getZ() << " < " << waypoint.getYawRad()/M_PI*180. << "deg" << endl;
    cout << "Relative pose is  : " << wpRel.getX() << ", " << wpRel.getY() << ", " << wpRel.getZ() << " < " << wpRel.getYawRad()/M_PI*180 << "deg" << endl;
    cout << "Dist to DEST is  : " << distToDest << endl;


    //cout << "Time for waypoint (Timeout): " << ( timestamp_now() - lp->getWaypointTime() )/1000000 << " (" << lp->getWaypointTimeout() << ")" << endl;

    if( ( distToDest < distToDestBound ) ||
        ( (wpRel.getX() < forwardBound) && (wpRel.getX() > -2*forwardBound) && (fabs(wpRel.getY()) < sideBound) ) ||
        (( timestamp_now() - lp->getWaypointTime() )/1000000 > lp->getWaypointTimeout() ) )  // the waypoint timeout may be redundant if the waypoints are being continually recalculated //||( ( timestamp_now() - lp->getWaypointTime() )/1000000 > lp->getWaypointTimeout() ) )
    {
        // delete it from the list
        //            pthread_mutex_lock(&lp->waypointsLock);
        lp->waypoints.erase( lp->waypoints.begin() );
        numWaypoints = lp->waypoints.size();


        for( unsigned int i = 0; i < waypoints.size(); i++ ) {
            fp_wp << waypoints.at(i).getX() << " " << waypoints.at(i).getY() << " " << waypoints.at(i).getZ() << endl;

        }
        fp_wp << 0 << " " << 0 << " " << 0 << endl;


        //			pthread_mutex_unlock(&lp->waypointsLock);
                lp->resetWaypointTime(timestamp_now());
        cout << "Ignored as it is behind us" << endl;

        if( numWaypoints == 0 ) {
            cout << "No more waypoints!" << endl;

            if( distToDest < distToDestBound ) {
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
    //double distToDestWp = numWaypoints * lp->getWpDropDist();
    //cout << "distToDestWp=" << distToDestWp << endl;
    double velChDist = lp->getVelChangeDist();
    //cout << "velChDist=" << velChDist << endl;
    // Linear ramp of speed on last meter to the final way point

    cout << "Dest velocity:" << destVel << endl;

    if (( distToDest < velChDist + distToDestBound)&&(distToDest>=distToDestBound)) { //( distToDestWp < velChDist  ) {
        //    if (( wpDist < velChDist + distToDestBound)&&(wpDist>=distToDestBound)){ //( distToDestWp < velChDist  ) {

        desVel =  currVel - (distToDest-velChDist-distToDestBound) / velChDist * (destVel-currVel);

        //        desVel =  currVel - (wpDist-velChDist-distToDestBound) / velChDist * (destVel-currVel);
        cout << "Destination velocity:" << destVel << endl;
        cout << "Desired velocity:" << desVel << endl;
    }
    else if (distToDest < distToDestBound){
        //    else if (wpDist < distToDestBound){
        desVel = 0;
        cout << "Destination velocity:" << destVel << endl;
        cout << "Desired velocity:" << desVel << endl;
    }

    // Linear ramp up
    //        else if( distFromStart < velChDist  && lp->startVel < 1e-3 ) {
    //            desVel = lp->getDefaultLegVel();// * distFromStart / velChDist;
    //        }
    else {
        desVel = lp->getDefaultLegVel();
    }




    //cout << "DESIRED" << endl;
    //cout << "\tHEADING  = " << desHeading/M_PI*180 << "deg" << endl;
    //cout << "\tVELOCITY = " << desVel << endl;
    //cout << "\tDEPTH    = " << waypoint.getZ() << endl;

    // for a message to send
    acfrlcm::auv_control_t cc;
    cc.utime = timestamp_now();


    //if( desVel < 1e-3 ) {
    // stop the motors
    //cc.run_mode = acfrlcm::auv_control_t::STOP; // commented out so that the vehicle stays stationary even with currents, so the motor never stops

    //}
    //else {
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
    //}
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

int getMin(double array[],int size) // get the index to the minimum value in an array
{
    int index = 0;
    for (int i = 1; i < size; i++) {
        if ( array[i] < array[index]) {
            index = i;
        }
    }

    return index;

}
