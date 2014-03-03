/*
 *  LegPath.cpp
 *
 *  Created on: Dec 19, 2012
 *      Author: navid
 */

#include "LegPath.h"

bool LegPath::calcPath(void) {

	if( pitch != 0 && altitudeChange != 0 ) {
		cerr << "Must set only pitch OR altitudeChange. Not both." << endl;
		return false;
	}
	else if( altitudeChange ){
		pitch = asin( (position.getZ() + altitudeChange) / length);
	}


	waypoint wp;
	// First way point
	wp.pose = position;
	wp.pose.setRollPitchYawRad(0, 0, heading);
	path.push_back(wp);

	for( int i = dropDist; i < length; i+=dropDist ) {
		wp.pose.setPosition(
				position.getX() + i*cos(heading),
				position.getY() + i*sin(heading),
				position.getZ() + i*sin(pitch));
		wp.pose.setRollPitchYawRad(0, 0, heading);
		path.push_back(wp);
	}

	// Last way point
	wp.pose.setPosition(
			position.getX() + length * cos(heading),
			position.getY() + length * sin(heading),
			position.getZ() + length*sin(pitch));
	wp.pose.setRollPitchYawRad(0, 0, heading);
	path.push_back(wp);

	return true;
}

