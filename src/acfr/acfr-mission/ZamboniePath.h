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

		ZamboniePath( Pose3D &p, double h, int d, double r, double dd, double da, double po, double w, double l ) {
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


		bool calcPath(void) { return calcPath(true); };
		bool calcPath(bool straightLeadIn = true);

};

#endif /* ZAMBONIEPATH_H_ */
