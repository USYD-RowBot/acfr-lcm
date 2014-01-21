//
// C++ Interface: DubinsPath
//
// Description:
//
//
// Author: Navid Nourani <navid.nourani-vatani@sydney.edu.au>, (C) 2012
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef DUBINSPATH_H
#define DUBINSPATH_H

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <limits>

#include <small/linalg.hh>
#include <small/Pose3D.hh>

using namespace std;
using namespace SMALL;

class DubinsPath {

private:
	int minDistI;
	double minDist;
	double minPitch;
	int minSide1;
	int minSide2;
	Pose3D minPose1;
	Pose3D minPose2;
	Pose3D minC1;
	Pose3D minC2;

	double circleRad;
	double maxPitch;
	double dropDist;
	double dropAngle;
	double angle1_report;
	double angle2_report;

	vector<Pose3D> path;
	double pathLength;

public:
	DubinsPath();
	DubinsPath(double circleRad, double maxPitch, double dropDist, double dropAngle);

	vector<Pose3D> calcPath(Pose3D currPose, Pose3D destPose);

	vector<Pose3D> getPath(void) const {
		return this->path;
	}
	double getPathLength(void) const {
		return this->pathLength;
	}

	bool setCircleRadius(double rad) {
		if (rad > 0) {
			this->circleRad = rad;
			return true;
		}

		return false;
	}
	bool setMaxPitch(double pitch) {
		this->maxPitch = pitch;
		return true;
	}

	bool setWaypointDropDist(double dist) {
		if (dist > 0) {
			this->dropDist = dist;
			return true;
		}

		return false;
	}
	bool setWaypointDropAngleFromDropDist(void) {
		if( this->dropDist <= 0 ) {
			return false;
		}
		// 2*PI / circumference / dist
		//this->dropAngle = (2*M_PI)/(2*M_PI*this->circleRad / this->dropDist);
		this->dropAngle = dropDist / circleRad;
		return true;
	}
	bool setWaypointDropAngleRad(double ang) {
		if (ang > 0) {
			this->dropAngle = ang;
			return true;
		}

		return false;
	}

	double getMaxAngle() {
		return (fabs(angle1_report) > fabs(angle2_report) ?
				fabs(angle1_report) / M_PI * 180 :
				fabs(angle2_report) / M_PI * 180);
	}

public:
	static const int right = -1;
	static const int left = -right;
	static const int side[2];
	vector<string> sideStr;

};

#endif // DUBINSPATH_H
