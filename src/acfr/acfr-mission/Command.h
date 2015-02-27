/*
 * Command.h
 *
 *  Created on: Dec 19, 2012
 *      Author: navid
 */

#ifndef COMMAND_H_
#define COMMAND_H_

#include "small/Pose3D.hh"
#include "MissionPrimitive.hpp"

class Command : public MissionPrimitive {

	public:
		Command() {}

		bool calcPath(void){ 
		    path.clear();
			waypoint wp;
			wp.goalType = COMMAND;
			path.push_back(wp);
			return true;
		};

};

#endif /* COMMAND_H_ */
