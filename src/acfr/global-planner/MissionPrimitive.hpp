#include <list>
#include <small/Pose3D.hh>

#include "mission_command.hpp"
#include "waypoint.hpp"

#ifndef MISSION_PRIMATIVE_HPP
#define MISSION_PRIMATIVE_HPP

class MissionPrimitive {
protected:

	std::list<waypoint> path;
	std::list<waypoint>::iterator it;

	double timeout;
	depthModeT depthMode;
	missionGoalTypeT goalType;

	SMALL::Pose3D position;
	double heading;
	double length;
	double width;
	double turnRadius;
	double pitch;
	double pathOffset;
	double velocity;
	double centerOverlap;
	double minWidthLength;
	double altitudeChange;

	int direction;
	double numLoops;

	double dropDist;
	double dropAngle;

public:
	static const int DIRECTION_CW = 1;
	static const int DIRECTION_CCW = -1;

	MissionPrimitive() :
			timeout(0), depthMode(DEPTH_MODE_DEPTH), goalType(GOAL), heading(0), length(
					0), width(0), turnRadius(7.5), pitch(0), pathOffset(0), velocity(
					1), centerOverlap(0), minWidthLength(0), altitudeChange(0), direction(
					DIRECTION_CW), numLoops(0), dropDist(1), dropAngle(dropDist/turnRadius) {

	}

	virtual ~MissionPrimitive() {
	}
	virtual bool calcPath(void) = 0;

	bool generatePath(double _timeout, depthModeT _depthMode,
			list<MissionCommand> &commands, int wpStartNum = 0) {
		timeout = _timeout;
		depthMode = _depthMode;

		if (!calcPath()) {
			return false;
		}

		double deltaTimeout = timeout / path.size();

		it = path.begin();
		(*it).commands = commands;
		for (; it != path.end(); ++it) {
			(*it).timeout = deltaTimeout;
			(*it).depthMode = depthMode;
			(*it).velocity[0] = velocity;
			(*it).id = wpStartNum++;
		}

		return true;
	}

	std::list<waypoint> & getPath(void) {
		return path;
	}

	waypoint getWaypoint(unsigned int i) {
		it = path.begin();
		for (unsigned int ii = 0; ii < i; ii++)
			it++;
		return (*it);
	}

	void setTimeout(double t) {
		timeout = t;
	}
	void setDepthMode(depthModeT d) {
		depthMode = d;
	}
	void setMissionGoalType(missionGoalTypeT m) {
		goalType = m;
	}

	double calculatePathOffset(double coverageWidth, double overlap) {
		double offset = (coverageWidth - overlap) / sqrt(2.);
		return offset;
	}

	std::list<waypoint> getPath(void) const {
		return path;
	}

	void setPose(SMALL::Pose3D p) {
		position = p;
	}
	void setPosition(double x, double y, double z) {
		position.setPosition(x, y, z);
	}
	void setHeadingRad(double h) {
		heading = h;
	}
	void setHeadingDeg(double h) {
		setHeadingRad(h / 180. * M_PI);
	}
	bool setLength(double l) {
		if (l <= 0)
			return false;
		length = l;
		return true;
	}
	bool setWidth(double w) {
		if (w <= 0)
			return false;
		width = w;
		return true;
	}
	bool setTurnRadius(double r) {
		if (r <= 0)
			return false;
		turnRadius = r;
		return true;
	}
	void setPitchRad(double p) {
		this->pitch = fabs(p);
	}
	void setPitchDeg(double p) {
		setPitchRad(p / 180. * M_PI);
	}

	bool setPathOffset(double o) {
		if (o <= 0)
			return false;
		pathOffset = o;
		return true;
	}
	bool setVelocity(double v) {
		if (v < 0)
			return false;
		velocity = v;
		return true;
	}
	void setDirectionCW(void) {
		direction = DIRECTION_CW;
	}
	void setDirectionCCW(void) {
		direction = DIRECTION_CCW;
	}

	bool setDropDist(double d) {
		if (d <= 0)
			return false;
		dropDist = d;
		return true;
	}
	bool setDropAngleRad(double d) {
		if (d <= 0)
			return false;
		dropAngle = d;
		return true;
	}
	bool setDropAngleDeg(double p) {
		return setDropAngleRad(p / 180. * M_PI);
	}
	bool setCenterOverlap(double o) {
		if (o > 0) {
			this->centerOverlap = o;
			return true;
		}
		return false;
	}
	void setCenterOverlapDeg(double o) {
	}

	bool setMinWidthLength(double m) {
		if (m < 0)
			return false;
		minWidthLength = m;
		return true;
	}

	void setAltitudeChange(double a) {
		altitudeChange = a;
	}
	void setNumLoops(unsigned int n) {
		numLoops = n;
	}

	void printPath(void) {
		cout << "path = [" << endl;
		it = path.begin();
		for (; it != path.end(); it++)
		{
			printf( "%3.2f, %3.2f, %3.2f;\n",
					(*it).pose.getX(),
					(*it).pose.getY(),
					(*it).pose.getZ() );
		}
		cout << "];" << endl;
	}
};

#endif
