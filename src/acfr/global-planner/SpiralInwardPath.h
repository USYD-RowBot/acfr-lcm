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

		bool calcPath(void);

		void printPose(Pose3D p) {
			cout << p.getX() << ", " << p.getY() << " < " << p.getYawRad() / M_PI * 180 << endl;
		}
};

#endif /* SPIRALINWARDPATH_H_ */
