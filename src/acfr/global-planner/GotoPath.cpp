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
	path.push_back( wp );
	cout << "GoTo:calcPath heading=" << heading << endl;
	return true;
}


