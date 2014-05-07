#include <small/Pose3D.hh>
#include <list>
#include <iomanip>
#include "mission_command.hpp"

#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

#define MIN_TIMEOUT  1

using namespace std;

// Depth mode, we are always doing one of two things
typedef enum {
	DEPTH_MODE_DEPTH, DEPTH_MODE_ALTITUDE
} depthModeT;

// Mission goal type is used as we can have primitives that just contain commands
typedef enum {
	GOAL, COMMAND
} missionGoalTypeT;

class WaypointSimple {
    public:
        WaypointSimple(double _x, double _y, double _z) { x = _x; y = _y, z = _z;};
        double x;
        double y;
        double z;
};

// goal class, inherites waypoint and adds a time out, velocity and some control tags
class waypoint {
	public:
		SMALL::Pose3D pose;
		SMALL::Vector3D velocity;
		double timeout;
		int id;
		list<MissionCommand> commands;
		missionGoalTypeT goalType;
		depthModeT depthMode;

		friend std::ostream &operator<<(std::ostream &os, const waypoint &p) {
			return os << setiosflags(ios::fixed) << setprecision(3) << p.pose.getX() << " " << p.pose.getY() << " "
					<< p.pose.getZ() << " " << p.pose.getRollRad() / M_PI * 180 << " "
					<< p.pose.getPitchRad() / M_PI * 180 << " " << p.pose.getYawRad() / M_PI * 180 << " " << p.velocity[0]
					<< " " << p.velocity[1] << " " << p.velocity[2] << " " << p.timeout << " "; // <<
			//((p.depth_mode == DEPTH_MODE_ALTITUDE) ? "A" : "D") << " " <<
			//((p.type == GOAL) ? "G" : "C");
		}
};

#endif
