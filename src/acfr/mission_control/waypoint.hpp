
#include <small/Pose3D.hh>
#include <list>
#include <iomanip>

#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

#define MIN_TIMEOUT  1

using namespace std;

typedef enum
{
    CAMERA,
    DVL
}   mission_command_device_t;

typedef enum
{
    ON_OFF,
    CAMERA_RATE,
    CAMERA_STROBE_DURATION,
    PD0,
    PD5
} mission_command_type_t;

typedef enum
{
    DEPTH_MODE_DEPTH,
    DEPTH_MODE_ALTITUDE
} depth_mode_t;

typedef enum
{
    GOAL,
    COMMAND
} mission_goal_type_t;

class mission_command
{
    public:
        mission_command_device_t device;
        mission_command_type_t command;
        int value_int;
        double value_double;
};

// location class
class location
{
    public:
        SMALL::Pose3D loc;
        int64_t time;
};

// goal class, inherites waypoint and adds a time out, velocity and some control tags
class goal_point : public location
{
    public:
        double timeout;
        double vel[3];
        list<mission_command> commands;
        mission_goal_type_t type;
        depth_mode_t depth_mode;
        
        friend std::ostream &operator<<(std::ostream &os, const goal_point &p)
        {
            return os << setiosflags(ios::fixed) << setprecision(3) <<
                p.loc.getX() << " " << p.loc.getY() << " " << p.loc.getZ() << " " << 
                p.loc.getRollRad() / M_PI * 180 << " " << p.loc.getPitchRad() / M_PI * 180 << " " << p.loc.getYawRad() / M_PI * 180 << " " <<
                p.vel[0] << " " << p.vel[1] << " " << p.vel[2] << " " <<
                p.timeout << " " <<
                ((p.depth_mode == DEPTH_MODE_ALTITUDE) ? "A" : "D") << " " <<
                ((p.type == GOAL) ? "G" : "C");
         }   
};

#endif
