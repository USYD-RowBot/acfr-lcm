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

#include <string>
#include <vector>
#include <limits>

#include <small/linalg.hh>
#include <small/Pose3D.hh>

using namespace std;
using namespace SMALL;

class DubinsPath {

protected:
	const static int right = -1;
	const static int left = -right;
	int side[2];
	string sideStr[2];

private:
	int minDistI;
	double minDist;
	int minSide1;
	int minSide2;
	Pose3D minPose1;
	Pose3D minPose2;
	Pose3D minC1;
	Pose3D minC2;

	double circleRad;
	double dropDist;
	double dropAngle;

	vector<Pose3D> path;

public:
	DubinsPath(double circleRad = 1.0, double dropDist = 1.0,
			double dropAngle = M_PI / 18);

	vector<Pose3D> calcPath(Pose3D currPose, Pose3D destPose);

	vector<Pose3D> getPath(void) const {
		return this->path;
	}

	bool setCircleRadius(double rad) {
		if (rad > 0) {
			this->circleRad = rad;
			return true;
		}

		return false;
	}
	bool setWaypointDropDist(double dist) {
		if (dist > 0) {
			this->dropDist = dist;
			return true;
		}

		return false;
	}

	bool setWaypointDropAngle(double ang) {
		if (ang > 0) {
			this->dropAngle = ang;
			return true;
		}

		return false;
	}
};

#endif // DUBINSPATH_H
