/*
 * ZamboniePath.h
 *
 *  Created on: Dec 13, 2012
 *      Author: navid
 */

#ifndef ZAMBONIEPATH_H_
#define ZAMBONIEPATH_H_

#include <math.h>
#include <vector>
#include <small/Pose3D.hh>

#include "MissionPrimitive.hpp"
using namespace std;
using namespace SMALL;

class ZamboniePath : public MissionPrimitive {

	private:
	    const string primitiveType;
	
	public:
		ZamboniePath() : primitiveType("Zambonie")  {
		}
		~ZamboniePath() {}

		bool calcPath(void);


		void printPose(Pose3D p) {
			cout << p.getX() << ", " << p.getY() << " < " << p.getYawRad() / M_PI * 180 << endl;
		}
};

#endif /* ZAMBONIEPATH_H_ */
