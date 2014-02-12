/*
 *  LegPath.cpp
 *
 *  Created on: Dec 19, 2012
 *      Author: navid
 */

#include "LegPath.h"

bool LegPath::calcPath(void) {

	double Z = 0;
	if( pitch != 0 && altitudeChange != 0 ) {
		cerr << "Must set only pitch OR altitudeChange. Not both." << endl;
		return false;
	}
	else if( pitch != 0 ) {
		Z = length * sin(pitch);
	}
	else {
		Z = altitudeChange;
	}


	waypoint wp;
	// First way point
	wp.pose = position;
	wp.pose.setRollPitchYawRad(0, 0, heading);
	path.push_back(wp);
	cout << "First pose = " << wp.pose << endl;

	// Last way point
	wp.pose.setPosition(length * cos(heading), length * sin(heading), Z);
	wp.pose.setRollPitchYawRad(0, 0, heading);
	path.push_back(wp);
	cout << "Second pose = " << wp.pose << endl;
	return true;
}

