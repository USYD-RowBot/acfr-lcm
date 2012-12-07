// Class definitions of the mission primatives

#include <small/Pose3D.hh>
#include <list>
#include "waypoint.hpp"

#ifndef MISSION_PRIMATIVES_HPP
#define MISSION_PRIMATIVES_HPP


// Base clase
class mission_primative
{
    public:
        double timeout;
        int id;
        depth_mode_t depth_mode;
        list<mission_command> commands;

};

class mission_com : public mission_primative
{
    public:
        int to_points(list<goal_point>&);
};

class mission_line : public mission_primative
{   
    public:
        SMALL::Pose3D start;
        SMALL::Pose3D end;
        double start_vel[3];
        double end_vel[3];
        int to_points(list<goal_point>&);


};

class mission_goto : public mission_primative
{
    public:
        SMALL::Pose3D end;
        double vel[3];
        int to_points(list<goal_point>&);
};

class mission_grid : public mission_primative
{
    public:
        SMALL::Pose3D center;
        double length;
        double width;
        int number_lines;
        double rotation;
        double velocity;
        int to_points(list<goal_point>&);
};	

class mission_pause : public mission_primative
{   
    public:
        double time;
        int to_points(list<goal_point>&);
};    

class mission_hold : public mission_primative
{
    public:
        SMALL::Pose3D position;
        double time;
        int to_points(list<goal_point>&);
};


#endif
