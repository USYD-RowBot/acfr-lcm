/*
 * ZamboniePath.cpp
 *
 *  Created on: Dec 13, 2012
 *      Author: navid
 */

#include "ZamboniePath.h"

bool ZamboniePath::calcPath(bool straightLeadIn) {

	waypoint wp;
	Pose3D currPose;
	Pose3D nextPose;
	Pose3D c;

	cout << endl << "====================" << endl;
	cout << "ZamboniePath" << endl;
	cout << "Start pose = " << position.getX() << ", " << position.getY() << ", " << position.getZ() << endl;
	cout << "Heading = " << heading << endl;
	cout << "Length/Width = " << length << "/" << width << endl;
	cout << "Offset   = " << this->pathOffset << endl;
	cout << "TurnRad  = " << this->turnRadius << endl;
	cout << "DropDist = " << this->dropDist << endl;
	cout << "DropAngl = " << this->dropAngle / M_PI * 180 << endl;
	cout << "Direction= " << (direction == DIRECTION_CW ? "CW" : "CCW") << endl;

	// Perform initial checks
	bool error = false;
	if( this->length <= 0 || this->width <= 0 || this->pathOffset <= 0 ) {
		cerr << "Error: Invalid width/length/offset" << endl;
		error = true;
	}
	if( this->length < 2 * turnRadius ) {
		cerr << "Error: Area LENGTH too short. Min l=2*turnRadius=" << 2 * turnRadius
				<< "m" << endl;
		error = true;
	}
	if (this->width < 4 * turnRadius + 2 * pathOffset + 2 * centerOverlap) {
		cerr << "Error: Area WIDTH too short. Min w=4*turnRadius+2*pathOffset+2*centerOverlap="
				<< 4 * turnRadius + 2 * pathOffset + 2 * centerOverlap << "m"
				<< endl;
		error = true;
	}
	if( true == error ) {
		cerr << "Fix this and try again. Exiting..." << endl;
		return false;
	}

	// Calculate number of loops
	//numLoops = (width / (2 * pathOffset) * 2) / 2 + 0.2;
    // add a couple of extra loops to ensure overlap in the centre
	numLoops = (width / (2 * pathOffset) * 2) / 2 + 2;
	cout << "Num loops = " << numLoops << endl;

	// Loop leg length
	double legs[4];
	legs[0] = length - 2 * turnRadius;
	legs[1] = width/2 - 2 * turnRadius + pathOffset;
	legs[2] = length - 2 * turnRadius;
	legs[3] = width/2 - 2 * turnRadius;
	cout << "Loop legs = " << legs[0] << ", " << legs[1] << ", " << legs[2] << ", " << legs[3] << endl;

	int nWpC = floor(0.5 * M_PI / dropAngle);
	double dropAngleC = (0.5 * M_PI) / nWpC;

	path.clear();
	// Start pose
	//	- bottom-left when performing clockwise spirals
	//	- bottom-right when performing counter-clockwise spirals
	currPose.setPosition(position.getX(), position.getY(), position.getZ());
	currPose.setRollPitchYawRad(0, 0, heading);
#if 0 // When doing half a circle
	// calculate the centre of the circle we are turning around
	nextPose.setIdentity();
	nextPose.setPosition(turnRadius, -direction * turnRadius, 0);
	c = currPose.compose(nextPose);
	c.setRollPitchYawRad(0, 0, heading-M_PI);
	// waypoints around this circle
	for( int i = 0; i < nWpC; i++ ) {
		double heading = -direction * dropAngleC * (i + 1);
		nextPose.setPosition(turnRadius * cos(heading), turnRadius * sin(heading), 0);
		nextPose.setRollPitchYawRad(0, 0, heading - direction * M_PI / 2);
		wp.pose = c.compose(nextPose);
		path.push_back(wp);
	}
#else // When doing a straight line
    // account for the combined paths that require stitching together two
    // primitives by deciding whether to include a lead in.
	nextPose.setIdentity();
    if (straightLeadIn == true) {
	    for( int i = 0; i < turnRadius; i+= dropDist ) {
    		nextPose.setPosition(i, 0, 0);
    		wp.pose = currPose.compose(nextPose);
    		path.push_back(wp);
    	}
	    nextPose.setPosition( turnRadius, 0, 0 );
    } else {
        cout << "Omitting leadin positions" << endl;
	    nextPose.setPosition( pathOffset, 0, 0 );
    }
#endif

	// Now set the current pose to the start of the first straight leg
	currPose = currPose.compose( nextPose );
	wp.pose = currPose;
	path.push_back(wp);

	// For every loop
	double loopNum = 0;
	while( loopNum < numLoops ) {

		for( int leg = 0; leg <= 3; leg++ ) {

			// Leg X way points
			int nWp = floor(legs[leg] / this->dropDist);
			for( int i = 0; i < nWp; i++ ) {
				nextPose.setIdentity();
				nextPose.setPosition((i+1)*dropDist, 0, 0);
				wp.pose = currPose.compose(nextPose);
				path.push_back(wp);
			}

			// We are now at the end of the leg
			nextPose.setIdentity();
			nextPose.setPosition(legs[leg], 0, 0);
			currPose = currPose.compose(nextPose);

			// Leg X to X+1 turn way points
			nextPose.setIdentity();
			nextPose.setPosition(0, -direction * turnRadius, 0);
			c = currPose.compose(nextPose);
			c.setRollPitchYawRad(0, 0, atan2( currPose.getY()-c.getY(), currPose.getX()-c.getX() ));
			for( int i = 0; i < nWpC; i++ ) {
				double heading = -direction * dropAngleC * (i + 1);
				nextPose.setPosition(turnRadius * cos(heading), turnRadius * sin(heading), 0);
				nextPose.setRollPitchYawRad(0, 0, heading - direction * M_PI / 2);
				wp.pose = c.compose(nextPose);
				path.push_back(wp);
			}

			// We are now at the end of the leg turn
			double heading = -direction * M_PI/2;
			nextPose.setPosition( turnRadius * cos(heading), turnRadius * sin(heading), 0 );
			nextPose.setRollPitchYawRad(0,0,heading-direction*M_PI/2);
			currPose = c.compose(nextPose);

			if( (loopNum+=.25) > numLoops ) {
				break;
			}

		}

	}


	return true;
}

