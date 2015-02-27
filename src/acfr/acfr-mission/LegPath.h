/*
 * LegPath.h
 *
 *  Created on: Dec 19, 2012
 *      Author: navid
 */

#ifndef LEGPATH_H_
#define LEGPATH_H_

#include "small/Pose3D.hh"
#include "MissionPrimitive.hpp"

class LegPath : public MissionPrimitive {

	public:
		LegPath() {}

		bool calcPath(void);

};

#endif /* LEGPATH_H_ */
