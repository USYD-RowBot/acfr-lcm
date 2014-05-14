/*
 * SpiralInwardPath.cpp
 *
 *  Created on: Dec 13, 2012
 *      Author: navid
 */

#include "SpiralInwardPath.h"

bool SpiralInwardPath::calcPath(void) {

	waypoint wp;
	Pose3D currPose;
	Pose3D nextPose;
	Pose3D c;

	cout << endl << "====================" << endl;
	cout << "SpiralInwardPath" << endl;
	cout << "Start pose = " << position.getX() << ", " << position.getY() << ", " << position.getZ() << endl;
	cout << "Heading = " << heading << endl;
	cout << "Length/Width = " << length << "/" << width << endl;
	cout << "Direction = " << (direction == DIRECTION_CCW ? "Counter " : "") << "Clockwise" << endl;
	cout << "Offset   = " << this->pathOffset << endl;
	cout << "TurnRad  = " << this->turnRadius << endl;
	cout << "DropDist = " << this->dropDist << endl;
	cout << "DropAngl = " << this->dropAngle / M_PI * 180 << endl;

	// Perform initial checks
	if( numLoops == 0 && minWidthLength == 0 ) {
		minWidthLength = 2.0 * turnRadius;
	}
	if( this->length <= 0 || this->width <= 0 || this->pathOffset <= 0 ) {
		cerr << "Invalid width/length/offset" << endl;
		return false;
	}
	// Remaining area too small
	if( min(width, length) < minWidthLength ) {
		cout << "Defined area too small: min W/L = 2.0*turnRadius = " << minWidthLength << endl;
		return false;
	}

	// Num waypoint in the turns
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
	for( int i = 0; i < turnRadius; i+= dropDist ) {
		nextPose.setPosition(i, 0, 0);
		wp.pose = currPose.compose(nextPose);
		path.push_back(wp);
	}
#endif

	nextPose.setIdentity();
	nextPose.setPosition(turnRadius,0,0);
	currPose = currPose.compose(nextPose);

	// For every loop
	double loopNum = 1;
	double loopLength = length;
	double loopWidth  = width;
	while( true ) {
		double legs[4];
		legs[0] = loopLength - 2*turnRadius;
		legs[1] = loopWidth - 2*turnRadius;
		legs[2] = loopLength - 2*turnRadius;
		legs[3] = loopWidth - 2 * turnRadius - this->pathOffset;

		for( int leg = 0; leg <= 3; leg++ ) {

			// Remaining area too small
			if( legs[leg] < 0 ) {
				cout << "Remaining area too small to continue" << endl;
				break;
			}

			// Leg X way points
			int nWp = floor(legs[leg] / this->dropDist);
			nextPose.setIdentity();
			nextPose.setPosition(dropDist, 0, 0);
			//dropDist = legs[leg] / nWp;
			for( int i = 0; i < nWp; i++ ) {
				currPose = currPose.compose(nextPose);
				wp.pose = currPose;
				path.push_back(wp);
			}
			// Insert the last way point
			double rem = legs[leg]-nWp*dropDist;
			if( rem > 1e-3 ) {
				nextPose.setPosition(rem, 0, 0);
				currPose = currPose.compose(nextPose);
				wp.pose = currPose;
				path.push_back(wp);
			}

			// Leg X to X+1 turn way points
			nextPose.setIdentity();
			nextPose.setPosition(0, -direction * turnRadius, 0);
			c = currPose.compose(nextPose);
			c.setRollPitchYawRad(0, 0, atan2(currPose.getY() - c.getY(), currPose.getX() - c.getX()));
			for( int i = 0; i < nWpC; i++ ) {
				double heading = -direction * dropAngleC * (i + 1);
				nextPose.setPosition(turnRadius * cos(heading), turnRadius * sin(heading), 0);
				nextPose.setRollPitchYawRad(0, 0, heading - direction * M_PI / 2);
				currPose = c.compose(nextPose);
				wp.pose = currPose;
				path.push_back(wp);
			}

		}

		// offset way point
		nextPose.setIdentity();
		nextPose.setPosition(pathOffset, 0, 0);
		currPose = currPose.compose(nextPose);

		// Inc loop number
		loopNum++;
		loopLength -= 2*pathOffset;
		loopWidth  -= 2*pathOffset;
		if( min(loopWidth, loopLength) < minWidthLength )
			break;
		// Max number of loops done
		if( numLoops > 0 && loopNum > numLoops ) {
			cout << "numLoops reached" << endl;
			break;
		}
	}

	return true;
}

