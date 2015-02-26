/*
 * SpiralInwardPath.h
 *
 *  Created on: Dec 13, 2012
 *      Author: navid
 */

#ifndef SPIRALINWARDPATH_H_
#define SPIRALINWARDPATH_H_

#include <math.h>
#include <vector>
#include <small/Pose3D.hh>

#include "MissionPrimitive.hpp"

using namespace std;
using namespace SMALL;

class SpiralInwardPath: public MissionPrimitive {


	public:
		SpiralInwardPath() {
		}
		SpiralInwardPath( Pose3D &p, double h, int d, double r, double dd, double da, double po, double w, double l ) {
			position = p;
			heading = h;
			direction = d;
			turnRadius = r;
			dropDist = dd;
			dropAngle = da;
			pathOffset = po;
			width = w;
			length = l;
		}

		bool calcPath(void);

		void printPose(Pose3D p) {
			cout << p.getX() << ", " << p.getY() << " < " << p.getYawRad() / M_PI * 180 << endl;
		}
};

#endif /* SPIRALINWARDPATH_H_ */
