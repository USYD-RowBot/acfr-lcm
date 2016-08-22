#include "path_planner.hpp"

// Exit handler, I swear this is the only global
int main_exit;
void signal_handler(int sig)
{
    main_exit = 1;
}

// Nav callback
void on_nav(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const auv_acfr_nav_t *nav, path_planner *pp) 
{
    // we are just going to make a copy of the data
    memcpy(&pp->nav, nav, sizeof(auv_acfr_nav_t));
}

// Path command callback
void on_path_command(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const auv_path_command_t *pc, path_planner *pp) 
{

    // we just got a leg command so we need to recalculate the path
    Pose3D currPose, destPose;
	currPose.setPosition(pc->wpt_one[0], pc->wpt_one[1], pc->wpt_one[2]);
	currPose.setRollPitchYawRad(pc->wpt_one[3], pc->wpt_one[4], pc->wpt_one[5]);

	destPose.setPosition(pc->wpt_two[0], pc->wpt_two[1], pc->wpt_two[2]);
	destPose.setRollPitchYawRad(pc->wpt_two[3], pc->wpt_two[4], pc->wpt_two[5]);

	pp->dp.setCircleRadius(pp->turn_radius);
	pp->dp.setWaypointDropDist(pp->drop_distance);
	pp->dp.setWaypointDropAngle(pp->drop_angle/180*M_PI);

	pp->path = pp->dp.calcPath( currPose, destPose );
	
	pp->start_point = pp->path.begin();
	pp->end_point = pp->path.begin() + 1;
	pp->done = 0;
	
	pp->depth_mode = pc->depth_mode;
	
	// fixed velocity for now
	pp->velocity  = pc->wpt_one[6];
	
}


void calculate(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const heartbeat_t *heartbeat, path_planner *pp) 
{
    // are we there yet, 2D version for now
    double distance = sqrt(pow(((*pp->end_point).getX() - pp->nav.x), 2) + pow(((*pp->end_point).getY() - pp->nav.y), 2));
    
    if(distance < pp->at_goal_distance)
    {
        // send the next segment
        if(pp->end_point != pp->path.end())
        {
            pp->start_point++;
            pp->end_point++;
        }
        else
            pp->done = 1;
    }

    auv_path_command_t pc;
    pc.utime = timestamp_now();
    
    if(pp->done)
    {
        // send a stop
        pc.command_type = auv_path_command_t::STOP;
    }
    else
    {
        pc.wpt_one[0] = (*pp->start_point).getX();
        pc.wpt_one[1] = (*pp->start_point).getY();
        pc.wpt_one[2] = (*pp->start_point).getZ();
//        pc.wpt_one[3] = (*pp->start_point[3]);
//        pc.wpt_one[4] = (*pp->start_point[4];
//        pc.wpt_one[5] = (*pp->start_point[5];
        pc.wpt_one[6] = pp->velocity;

        pc.wpt_two[0] = (*pp->end_point).getX();
        pc.wpt_two[1] = (*pp->end_point).getY();
        pc.wpt_two[2] = (*pp->end_point).getZ();
//        pc.wpt_two[3] = (*pp->end_point[3];
//        pc.wpt_two[4] = (*pp->end_point[4];
//        pc.wpt_two[5] = (*pp->end_point[5];
        pc.wpt_one[6] = pp->velocity;

        pc.command_type = auv_path_command_t::DOUBLE;
        pc.depth_mode = pp->depth_mode;
    }
    
    pp->lcm.publish("PATH_COMMAND", &pc);
        
        
}


path_planner::path_planner()
{
    lcm.subscribeFunction("ACFR_NAV.*", on_nav, this);
    lcm.subscribeFunction("HEARTBEAT_1HZ", calculate, this);                                
    lcm.subscribeFunction("LEG_COMMAND", on_path_command, this);
}

path_planner::~path_planner()
{
}

int path_planner::process()
{

    int fd = lcm.getFileno();
    fd_set rfds;
    while(!main_exit)
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

int path_planner::load_config(char *program_name)
{
    BotParam *param = NULL;
    param = bot_param_new_from_server (lcm.getUnderlyingLCM(), 1);
    if(param == NULL)
        return 0;
        
    char rootkey[64];        
    char key[128];
    sprintf (rootkey, "acfr.%s", program_name);

    sprintf(key, "%s.drop_distance", rootkey);
    drop_distance = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.drop_angle", rootkey);
    drop_angle = bot_param_get_double_or_fail(param, key);
    
    sprintf(key, "%s.turn_radius", rootkey);
    turn_radius = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.at_goal_distance", rootkey);
    at_goal_distance = bot_param_get_double_or_fail(param, key);
    
    return 1;
}

int main(int argc, char **argv)
{
    // install the signal handler
    main_exit = 0;
    signal(SIGINT, signal_handler);
    
    path_planner *pp = new path_planner();
    if(!pp->load_config(basename(argv[0])))
    {
        cerr << "Failed loading config" << endl;
        return 0;
    }
    
    // enter the main loop
    pp->process();
    
    delete pp;
    
    return 1;
}
