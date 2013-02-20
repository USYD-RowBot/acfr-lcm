/*
 * SpiralPath.h
 *
 *  Created on: Dec 13, 2012
 *      Author: navid
 */

#ifndef SPIRALPATH_H_
#define SPIRALPATH_H_

#include <math.h>
#include <vector>
#include <small/Pose3D.hh>

#include "MissionPrimitive.hpp"

using namespace std;
using namespace SMALL;

class SpiralPath : public MissionPrimitive {


	public:
		SpiralPath()  {
		}

		bool calcPath(void);



		void printPose(Pose3D p) {
			cout << p.getX() << ", " << p.getY() << " < " << p.getYawRad()/M_PI*180 << endl;
		}
};

#endif /* SPIRALPATH_H_ */
