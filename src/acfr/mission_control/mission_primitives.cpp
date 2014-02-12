
// Contains the functions to turn the primitives into a list of points

#include "mission_primitives.hpp"

int mission_line::to_points(list<goal_point> &points)
{
    // a line consists of two points
    // fisrt check to see if the proceeding point is the same as the one we
    // are inserting, if not we will put in two points
    
    // previous location
//    goal_point& prev; // = *points.end();
    if(!points.empty())
    {
        goal_point &prev = points.back();
       
        if((prev.loc != start) || !commands.empty())
        {
            // we are inserting a start point
            goal_point point;    
            point.loc = start;
            point.vel[0] = start_vel[0];
            point.vel[1] = start_vel[1];
            point.vel[2] = start_vel[2];        
            point.timeout = MIN_TIMEOUT;
            point.commands = commands;
            point.type = GOAL;
            point.depth_mode = depth_mode;
            
            points.push_back(point);
        }
        else
        {
            // we are only inserting one point which means we will need
            // to insert the commands
            prev.commands.splice(prev.commands.end(), commands);
        }
    }
    else
    {
        goal_point point;    
        point.loc = start;
        point.vel[0] = start_vel[0];
        point.vel[1] = start_vel[1];
        point.vel[2] = start_vel[2];        
        point.timeout = MIN_TIMEOUT;
        point.commands = commands;
        point.type = GOAL;
        point.depth_mode = depth_mode;
        
        points.push_back(point);
    }    
    
    // we always insert the end point
    goal_point point;
    point.loc = end;
    point.vel[0] = end_vel[0];
    point.vel[1] = end_vel[1];
    point.vel[2] = end_vel[2];        
    timeout = timeout;
    point.type = GOAL;
    point.depth_mode = depth_mode;
    points.push_back(point);
 
    return 1;
}
 
// Insert a goto point        
int mission_goto::to_points(list<goal_point> &points)    
{
    goal_point prev; // = *points.end();
    if(!points.empty())
    {
        prev = points.back();
        prev.commands.splice(prev.commands.end(), commands);
    }
    else if(!commands.empty())
    {
        // insert a command only goal point
        goal_point command_point;
        command_point.timeout = MIN_TIMEOUT;
        command_point.type = COMMAND;
        command_point.commands = commands;
        points.push_back(command_point);
    }


    goal_point point;
    point.loc = end;
    point.vel[0] = vel[0];
    point.vel[1] = vel[1];
    point.vel[2] = vel[2];        
    point.timeout = timeout;
    point.commands = commands;
    point.type = GOAL;
    point.depth_mode = depth_mode;
        
    points.push_back(point);
    
    return 1;
}



// Create a set of points that define a grid
// We will make the grid straight, then offset it and rotate it
int mission_grid::to_points(list<goal_point> &points)    
{
    // work out how many points we have
    int num_points = number_lines * 2;
    double short_leg = width / (double)(number_lines - 1);
    double long_leg = length;
    
    // the grid position is the center of the grid so our staring point is
    double start_x = -width / 2.0;
    double start_y = -length / 2.0;
    
    
    // lay out the initial grid, centered about 0, 0, depth
    list<goal_point> grid_points;
    double heading, y;
    double depth = center.getZ();
    for(int i=0; i<number_lines; i++)
    {
        goal_point p1;
        if(!(i % 2))
        {
            heading = 0;
            y = start_y;
        }
        else
        {
            heading = M_PI;
            y = -start_y;
        }
        p1.loc.setPosition(start_x + i*short_leg, y, depth);
        
        p1.loc.setRollPitchYawRad(0.0, 0.0, heading);
        p1.vel[0] = velocity;
        p1.vel[1] = 0;
        p1.vel[2] = 0;
        p1.type = GOAL;
        p1.depth_mode = depth_mode;
        //cout << "P1 " << p1 << endl;        
        grid_points.push_back(p1);

        goal_point p2;
        p2.loc.setPosition(start_x + i*short_leg, -y, depth);
        if(!(i % 2))
            heading = M_PI/2;
        else
            heading = -M_PI/2;
        p2.loc.setRollPitchYawRad(0.0, 0.0, heading);
        p2.vel[0] = velocity;
        p2.vel[1] = 0;
        p2.vel[2] = 0;
        p2.type = GOAL;
        p2.depth_mode = depth_mode;
        //cout << "P2 " << p2 << endl;
        grid_points.push_back(p2);
    }
    
    // now rotate the grid points about the center and move then so the
    // grid center is in the correct place
    SMALL::Pose3D grid_center;
    grid_center.setPosition(0.0, 0.0, 0.0);
    grid_center.setRollPitchYawRad(0.0, 0.0, rotation);
    for(list<goal_point>::iterator i = grid_points.begin(); i != grid_points.end(); ++i)
    {
        (*i).loc = grid_center.transformTo((*i).loc.getPosition());
        (*i).loc.setX((*i).loc.getX() + center.getX());
        (*i).loc.setY((*i).loc.getY() + center.getY());
    }
    
    // now put the points on the end of the list
    points.splice(points.end(), grid_points);
    
    return 1;
            
}

int mission_hold::to_points(list<goal_point> &points)    
{
}

int mission_com::to_points(list<goal_point> &points)    
{
    goal_point point;
    point.type = COMMAND;
    point.timeout = MIN_TIMEOUT;
    point.commands = commands;
    points.push_back(point);
    
    return 1;
}

int mission_zambonie::to_points(list<goal_point> &points)
{
    // Work out the centre overlap required for a compass error of 1 degree
    // this is the worst case.
    double gap = tan(1.0 / 180.0 * M_PI) * length;
    double half_width = width / 2.0 - gap;
    
    // The number of boxes required to fill the space
    int num_boxes = ceil(width / 2 + gap / spacing);
    
    // Given the centre work out the bottom left corner       
    double start_x = -width / 2.0 * direction;
    double start_y = -length / 2.0 * direction;
    double depth = center.getZ();
      
    goal_point p1, p2, p3, p4;
    list<goal_point> grid_points;
        
    // lets make some boxes
    for(int i=0; i<num_boxes; i++)
    {
        p1.loc.setRollPitchYawRad(0.0, 0.0, 0.0);
        p1.loc.setPosition(start_x + i * spacing * direction, start_y, depth);
        p1.vel[0] = velocity;
        p1.vel[1] = 0;
        p1.vel[2] = 0;
        p1.type = GOAL;
        p1.depth_mode = depth_mode;
        grid_points.push_back(p1);
       
        p2.loc.setRollPitchYawRad(0.0, 0.0, M_PI / 2.0 * direction);
        p2.loc.setPosition(start_x + i * spacing * direction, -start_y, depth);
        p2.vel[0] = velocity;
        p2.vel[1] = 0;
        p2.vel[2] = 0;
        p2.type = GOAL;
        p2.depth_mode = depth_mode;
        grid_points.push_back(p2);
 
        p3.loc.setRollPitchYawRad(0.0, 0.0, M_PI / 2.0 * direction);
        p3.loc.setPosition(start_x + i * spacing * direction + half_width * direction, -start_y, depth);
        p3.vel[0] = velocity;
        p3.vel[1] = 0;
        p3.vel[2] = 0;
        p3.type = GOAL;
        p3.depth_mode = depth_mode;
        grid_points.push_back(p3);
 
        p4.loc.setRollPitchYawRad(0.0, 0.0, -M_PI / 2.0 * direction);
        p4.loc.setPosition(start_x + i * spacing * direction + half_width * direction, start_y, depth);
        p4.vel[0] = velocity;
        p4.vel[1] = 0;
        p4.vel[2] = 0;
        p4.type = GOAL;
        p4.depth_mode = depth_mode;
        grid_points.push_back(p4);
    }
    
    // now rotate the grid points about the center and move then so the
    // grid center is in the correct place
    SMALL::Pose3D grid_center;
    grid_center.setPosition(0.0, 0.0, 0.0);
    grid_center.setRollPitchYawRad(0.0, 0.0, rotation);
    for(list<goal_point>::iterator i = grid_points.begin(); i != grid_points.end(); ++i)
    {
        (*i).loc = grid_center.transformTo((*i).loc.getPosition());
        (*i).loc.setX((*i).loc.getX() + center.getX());
        (*i).loc.setY((*i).loc.getY() + center.getY());
    }
    
    // now put the points on the end of the list
    points.splice(points.end(), grid_points);
    
    return 1;                     
}
