/*
 * GotoPath.h
 *
 *  Created on: Dec 19, 2012
 *      Author: navid
 */

#ifndef GOTOPATH_H_
#define GOTOPATH_H_

#include "small/Pose3D.hh"
#include "MissionPrimitive.hpp"

class GotoPath : public MissionPrimitive {

	public:
		GotoPath() {}

		bool calcPath(void);

};

#endif /* GOTO_H_ */
