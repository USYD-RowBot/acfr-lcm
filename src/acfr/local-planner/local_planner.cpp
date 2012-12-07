/*
 *  Local trajectory planner
 * 
 *  Takes in either one or two way points and sends the AUV along the
 *  line.  It is the responsibility of the module above to determine
 *  if we have reached out goal.
 *
 *  Christian Lees
 *  Navid Nourani-Vatani
 *
 *  ACFR
 *  21/11/12
 */

#include "local_planner.hpp"

// Exit handler, I swear this is the only global
int main_exit;
void signal_handler(int sig)
{
    main_exit = 1;
}

// Helper functions
double hypot_vector(Vector v)
{
    double a;
    for(int i=0; i<v.size(); i++)
        a += pow(v(i),2);
    return sqrt(a);
}


// Nav callback
void on_nav(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const auv_acfr_nav_t *nav, local_planner *lp) 
{
    // we are just going to make a copy of the data
    memcpy(&lp->nav, nav, sizeof(auv_acfr_nav_t));
}

// Path command callback
void on_path_command(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const auv_path_command_t *pc, local_planner *lp) 
{
    // we are just going to make a copy of the data
    memcpy(&lp->path_command, pc, sizeof(auv_path_command_t));
}

// Calculate the path, this is where the magic happens
void calculate(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const heartbeat_t *heartbeat, local_planner *lp) 
{
    // calculate out look ahead point
    double look_ahead_distance = lp->nav.vx * lp->look_ahead_velocity_scale + lp->turning_radius;
    Vector lap(3);
    lap(0) =  look_ahead_distance;
    lap(1) = 0;
    lap(2) = 0;
    
    // transform it to the earth frame
    Vector lap_earth = lp->transform_to_earth(lap);
    
    Vector wpt1(3), wpt2(3);
    
    double desired_velocity;
    
    //  what mode are we in
    if(lp->path_command.command_type == auv_path_command_t::SINGLE)
    {
        wpt1(0) = lp->nav.x;
        wpt1(1) = lp->nav.y;
        wpt1(2) = lp->nav.depth;
        
        wpt2(0) = lp->path_command.wpt_one[0];
        wpt2(1) = lp->path_command.wpt_one[1];
        wpt2(2) = lp->path_command.wpt_one[2];
        desired_velocity = lp->path_command.wpt_one[6];
    }
    else if(lp->path_command.command_type == auv_path_command_t::DOUBLE)
    {
        wpt1(0) = lp->path_command.wpt_one[0];
        wpt1(1) = lp->path_command.wpt_one[1];
        wpt1(2) = lp->path_command.wpt_one[2];

        wpt2(0) = lp->path_command.wpt_two[0];
        wpt2(1) = lp->path_command.wpt_two[1];
        wpt2(2) = lp->path_command.wpt_two[2];
        
        // check where we are to set the velocity
        Vector now(3);
        now(0) = lp->nav.x;
        now(1) = lp->nav.y;
        now(2) = lp->nav.depth;
        if(hypot_vector(wpt2 - now) < lp->velocity_change_distance) 
            desired_velocity = lp->path_command.wpt_two[6];
        else
           desired_velocity = lp->path_command.wpt_one[6];
    }
    else
    {
        // stop the motors
        auv_control_t cc;
        cc.utime = timestamp_now();
        cc.run_mode = auv_control_t::STOP;
        lp->lcm.publish("AUV_CONTROL", &cc);    
        
        return;
    }
     
        
        
    // calculate the distance
    double u = ((lap(0) - wpt1(0)) * (wpt2(0) - wpt1(0)) + (lap(1) - wpt1(1)) * (wpt2(1) - wpt1(1)) + (lap(2) - wpt1(2)) * (wpt2(2) - wpt1(2))) / pow(hypot_vector(wpt2 - wpt1), 2);
    Vector point_inter = wpt1 + u * (wpt2 - wpt1);
    
    double distance = hypot_vector(point_inter - lap);
    
    // transform the point on the line into the body frame so we can work out which side it is on
    Vector point_inter_body = lp->transform_to_body(point_inter);
    if(point_inter_body(1) < 0)
        distance = -distance;
        
    // depth difference
    double depth_diff = point_inter(2) - lap(2);
    
    // calculate the angle between the current heading and the line tangent at the closes point on the line
    double line_angle = atan2(wpt2(1) - wpt1(1), wpt2(0) - wpt1(0));
    double angle_diff = line_angle - lp->nav.heading;
    while(angle_diff < -M_PI)
        angle_diff += 2*M_PI;

    while(angle_diff > M_PI)
        angle_diff -= 2*M_PI;
        
    //Calculate the desired vel, heading and depth to be commanded to the controller    
    
    //desired way point heading
    double gamma;
    if(abs(distance) > lp->max_dist_from_line)
        gamma = 0.9;
    else if(abs(angle_diff) > lp->max_angle_from_line)
        gamma = 0.1;
    else
        gamma = 0.75;

    double desired_heading = gamma * atan2(distance, look_ahead_distance) + (1 - gamma) * angle_diff;

    
    // desired way point depth/altitude
    double desired_depth = point_inter(2);

    // desired way point velocity
    
    // for a message to send
    auv_control_t cc;
    cc.utime = timestamp_now();
    cc.run_mode = auv_control_t::RUN;
    cc.heading = desired_heading;
    if(lp->path_command.depth_mode == auv_path_command_t::DEPTH)
    {
        cc.depth = desired_depth;
        cc.depth_mode = auv_control_t::DEPTH_MODE;
    }
    else
    {
        cc.altitude = desired_depth;
        cc.depth_mode = auv_control_t::ALTITUDE_MODE;
    }
    cc.vx = desired_velocity;
    
    // publish
    lp->lcm.publish("AUV_CONTROL", &cc);
}



local_planner::local_planner()
{
    lcm.subscribeFunction("ACFR_NAV", on_nav, this);
    lcm.subscribeFunction("PATH_COMMAND", on_path_command, this);
    lcm.subscribeFunction("HEARTBEAT_1HZ", calculate, this);                                
}

local_planner::~local_planner()
{
}


Vector local_planner::transform_to_earth(Vector point)
{
    
    Matrix rot(rot_mat_back(nav.roll, nav.pitch, nav.heading));
    Vector new_point(prod(rot, point));
    
    return new_point;
}
        
Vector local_planner::transform_to_body(Vector point)
{
    
    Matrix rot(rot_mat_forward(nav.roll, nav.pitch, nav.heading));
    Vector new_point(prod(rot, point));
    
    return new_point;
}

int local_planner::load_config(char *program_name)
{
    BotParam *param = NULL;
    param = bot_param_new_from_server (lcm.getUnderlyingLCM(), 1);
    if(param == NULL)
        return 0;
        
    char rootkey[64];        
    char key[128];
    sprintf (rootkey, "acfr.%s", program_name);

    sprintf(key, "%s.look_ahead_velocity_scale", rootkey);
    look_ahead_velocity_scale = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.turning_radius", rootkey);
    turning_radius = bot_param_get_double_or_fail(param, key);
    
    sprintf(key, "%s.max_dist_from_line", rootkey);
    max_dist_from_line = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.max_angle_from_line", rootkey);
    max_angle_from_line = bot_param_get_double_or_fail(param, key);

    sprintf(key, "%s.velocity_change_distance", rootkey);
    velocity_change_distance = bot_param_get_double_or_fail(param, key);

    return 1;
}

int local_planner::process()
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


int main(int argc, char **argv)
{
    // install the signal handler
    main_exit = 0;
    signal(SIGINT, signal_handler);
    
    local_planner *lp = new local_planner();
    if(!lp->load_config(basename(argv[0])))
    {
        cerr << "Failed loading config" << endl;
        return 0;
    }
    
    // enter the main loop
    lp->process();
    
    delete lp;
    
    return 1;
}
    
    
