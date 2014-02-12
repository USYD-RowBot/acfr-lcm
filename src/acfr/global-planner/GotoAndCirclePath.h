/*
 * GotoPath.h
 *
 *  Created on: Dec 19, 2012
 *      Author: navid
 */

#ifndef GOTOANDCIRCLEPATH_H_
#define GOTOANDCIRCLEPATH_H_

#include "small/Pose3D.hh"
#include "MissionPrimitive.hpp"

using namespace SMALL;

class GotoAndCirclePath : public MissionPrimitive {

	public:
		GotoAndCirclePath() {
			direction = DIRECTION_CCW;
		}

		bool calcPath(void);

};

#endif /* GOTOANDCIRCLE_H_ */
