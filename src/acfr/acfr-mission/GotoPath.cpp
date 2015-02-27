/*
 * GotoPath.cpp
 *
 *  Created on: Dec 19, 2012
 *      Author: navid
 */

#include "GotoPath.h"

bool GotoPath::calcPath( void ) {

	waypoint wp;
	wp.pose = position;
	wp.pose.setRollPitchYawRad(0, 0, heading);
	wp.velocity = this->velocity, 0, 0;
	path.push_back( wp );

	return true;
}


