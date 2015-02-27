/*
 * CoveragePath.h
 *
 *  Created on: Mar 10, 2014
 *      Author: navid
 */

#ifndef COVERAGEPATH_H_
#define COVERAGEPATH_H_

#include <math.h>
#include <vector>
#include <small/Pose3D.hh>

#include "MissionPrimitive.hpp"
#include "ZamboniePath.h"
#include "SpiralInwardPath.h"

using namespace std;
using namespace SMALL;

class CoveragePath: public MissionPrimitive {


	public:
		CoveragePath() {
		}

		bool calcPath(void);

};

#endif /* COVERAGEPATH_H_ */
