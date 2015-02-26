/*
 * SpiralPath.cpp
 *
 *  Created on: Dec 13, 2012
 *      Author: navid
 */

#include "SpiralPath.h"

bool SpiralPath::calcPath(void) {

	waypoint wp;
	Pose3D c;
	Pose3D currPose;
	Pose3D nextPose;

	cout << endl << "SpiralPath" << endl;
	cout << "TurnRad  = " << this->turnRadius << endl;
	cout << "DropDist = " << this->dropDist << endl;
	cout << "DropAngl = " << this->dropAngle / M_PI * 180 << endl;
    cout << "Num Loop = " << this->numLoops << endl;
    cout << "Alt Chng = " << altitudeChange << endl;
    
	// Calculate number of loops
	if( numLoops == 0 && altitudeChange == 0 ) {
		cerr << "Need to know either number of loops or the change in altitude" << endl;
		return false;
	}

	// Positive altitude is downward, Negative altitude is upward
	double desPitch = 0;
	if( altitudeChange != 0 ) {
		double dist = fabs(altitudeChange / sin(pitch));
		numLoops = dist / (2 * M_PI * turnRadius);
		desPitch = pitch;
		cout << "To change the altitude by " << altitudeChange << "m with a pitch angle of " << pitch / M_PI * 180
				<< "deg \na distance of " << dist << "m has to be travelled. \nThis corresponds to " << numLoops
				<< " loops with radius " << turnRadius << "m" << endl;
	}

	int sign = altitudeChange / fabs(altitudeChange);
	cout << "Spiraling " << (sign < 0 ? "upwards" : "downwards") << endl;

	double rads = 2 * M_PI * numLoops;
	cout << "That is " << rads / M_PI * 180 << "degs" << endl;

	int nWpC = ceil(rads / dropAngle);

	for( int i = 0; i < nWpC; i++ ) {
		double heading = direction * dropAngle * i;
		double dist = (dropAngle * i) * M_PI;
		double z = dist * sign * sin(desPitch);
		nextPose.setPosition(turnRadius * cos(heading), turnRadius * sin(heading), z);
		nextPose.setRollPitchYawRad(0, 0, heading + direction * M_PI / 2);
		currPose = position.compose(nextPose);
		wp.pose = currPose;
		path.push_back(wp);
	}

	return true;
}

