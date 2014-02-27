/*
 * ZamboniePath.cpp
 *
 *  Created on: Dec 13, 2012
 *      Author: navid
 */

#include "ZamboniePath.h"

bool ZamboniePath::calcPath(void) {

	waypoint wp;
	Pose3D currPose;
	Pose3D nextPose;
	Pose3D c;

	cout << endl << "ZamboniePath" << endl;
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

	// Calculate loop width and length
	double loopWidth = this->width / 2 - centerOverlap;
	double loopLength = this->length;
	cout << "Loop W/L = " << loopWidth << "/" << loopLength << endl;

	// Calculate number of loops
	numLoops = ceil(width / (2 * pathOffset) * 2) / 2;
	cout << "Num loops = " << numLoops << endl;

	// Loop leg length
	double legs[4];
	legs[0] = loopLength - 2 * turnRadius;
	legs[1] = loopWidth - 2 * turnRadius;
	legs[2] = loopLength - 2 * turnRadius;
	legs[3] = loopWidth - 2 * turnRadius - this->pathOffset;
	cout << "Loop legs = " << legs[0] << ", " << legs[1] << ", " << legs[2] << ", " << legs[3] << endl;

	int nWpC = floor(0.5 * M_PI / dropAngle);
	double dropAngleC = (0.5 * M_PI) / nWpC;

	// Calculate start pose
	//	- bottom-left when performing clockwise spirals
	//	- bottom-right when performing counter-clockwise spirals
	currPose.setPosition(position.getX(), position.getY(), position.getZ());
	currPose.setRollPitchYawRad(0, 0, heading - M_PI/2*direction);
	wp.pose = currPose;
	path.push_back(wp);

	// Initial way points
	nextPose.setIdentity();
	nextPose.setPosition(0, direction * turnRadius, 0);
	c = currPose.compose(nextPose);
	c.setRollPitchYawRad(0, 0, atan2(currPose.getY() - c.getY(), currPose.getX() - c.getX()));
	for( int i = 0; i < nWpC; i++ ) {
		double heading = direction * dropAngleC * (i + 1);
		nextPose.setPosition(turnRadius * cos(heading), turnRadius * sin(heading), 0);
		nextPose.setRollPitchYawRad(0, 0, heading + direction * M_PI / 2);
		currPose = c.compose(nextPose);
		wp.pose = currPose;
		path.push_back(wp);
	}

	// For every loop
	double loopNum = 0;
	while( loopNum < numLoops ) {

		for( int leg = 0; leg <= 3; leg++ ) {

			// Leg X way points
			nextPose.setIdentity();
			int nWp = floor(legs[leg] / this->dropDist);
			for( int i = 0; i < nWp; i++ ) {
				nextPose.setPosition(dropDist, 0, 0);
				currPose = currPose.compose(nextPose);
				wp.pose = currPose;
				path.push_back(wp);
			}
			// Insert the last way point
			double rem = legs[leg]-nWp*dropDist;
			if( rem > 1e-3 ) {
				nextPose.setPosition(legs[leg]-nWp*dropDist, 0, 0);
				currPose = currPose.compose(nextPose);
				wp.pose = currPose;
				path.push_back(wp);
			}

			// Leg X-X+1 turn way points
			nextPose.setIdentity();
			nextPose.setPosition(0, direction * turnRadius, 0);
			c = currPose.compose(nextPose);
			c.setRollPitchYawRad(0, 0, atan2(currPose.getY() - c.getY(), currPose.getX() - c.getX()));
			for( int i = 0; i < nWpC; i++ ) {
				double heading = direction * dropAngleC * (i + 1);
				nextPose.setPosition(turnRadius * cos(heading), turnRadius * sin(heading), 0);
				nextPose.setRollPitchYawRad(0, 0, heading + direction * M_PI / 2);
				currPose = c.compose(nextPose);
				wp.pose = currPose;
				path.push_back(wp);
			}

			loopNum += 0.25;
			if( loopNum > numLoops ) {
				break;
			}

		}

	}

	printPath();

	return true;
}

