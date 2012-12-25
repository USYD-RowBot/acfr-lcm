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
	cout << "Center = " << position.getX() << ", " << position.getY() << ", " << position.getZ() << endl;
	cout << "Length/Width = " << length << "/" << width << endl;
	cout << "Direction = " << "(" << direction << ")" << (direction == 1 ? "Counter " : "") << "Clockwise" << endl;
	cout << "Offset   = " << this->pathOffset << endl;
	cout << "TurnRad  = " << this->turnRadius << endl;
	cout << "DropDist = " << this->dropDist << endl;
	cout << "DropAngl = " << this->dropAngle / M_PI * 180 << endl;

	// Perform initial checks
	if( this->length <= 0 || this->width <= 0 || this->pathOffset <= 0 ) {
		cerr << "Invalid width/length/offset" << endl;
		return false;
	}

	if( numLoops == 0 && minWidthLength == 0 ) {
		minWidthLength = 2.5 * turnRadius;
	}

	// Calculate start pose
	//	- bottom-left when performing clockwise spirals
	//	- bottom-right when performing counter-clockwise spirals
	currPose.setIdentity();
	currPose.setPosition(position.getX() - this->width / 2 * direction, position.getY() - this->length / 2, position.getZ());
	currPose.setRollPitchYawRad(0, 0, heading + M_PI / 2 - direction * M_PI / 2);
	wp.pose = currPose;
	path.push_back(wp);
	cout << "Start pose " << currPose.getX() << ", " << currPose.getY() << ", " << currPose.getZ() << " < "
			<< currPose.getYawRad() / M_PI * 180 << endl;

	// Initial way points
	int nWp0 = floor(this->turnRadius / dropDist);
	cout << turnRadius << " / " << dropDist << " = " << nWp0 << endl;
	for( int i = 0; i <= nWp0 - 1; i++ ) {
		nextPose.setIdentity();
		nextPose.setPosition(dropDist, 0, 0);
		currPose = currPose.compose(nextPose);
		wp.pose = currPose;
		path.push_back(wp);
	}
	double rem = turnRadius-nWp0*dropDist;
	cout << "rem = " << rem;
	if( rem > 1e-3 ) {
		nextPose.setPosition(rem, 0, 0);
		currPose = currPose.compose(nextPose);
		wp.pose = currPose;
		path.push_back(wp);
	}


	// For every loop
	unsigned int loopNum = 1;
	while( true ) {
		cout << "\nLoop # " << loopNum << endl;

		// Max number of loops done
		if( this->numLoops > 0 && loopNum > numLoops ) {
			cout << "numLoops reached" << endl;
			break;
		}

		// Calculate loop width and length
		double loopWidth = this->width - (loopNum - 1) * 2 * this->pathOffset;
		double loopLength = this->length - (loopNum - 1) * 2 * this->pathOffset;

		// Remaining area too small
		if( min(loopWidth, loopLength) < 2.5 * turnRadius || min(loopWidth, loopLength) < minWidthLength ) {
			cout << "Remaining area too small to continue" << endl;
			break;
		}

		double legs[4];
		legs[0] = loopLength - 2 * turnRadius;
		legs[1] = loopWidth - 2 * turnRadius;
		legs[2] = loopLength - 2 * turnRadius;
		legs[3] = loopWidth - 2 * turnRadius - this->pathOffset;
		cout << "Loop legs = " << legs[0] << ", " << legs[1] << ", " << legs[2] << ", " << legs[3] << endl;

		int nWpC = floor(0.5 * M_PI / dropAngle);
		double dropAngleC = (0.5 * M_PI) / nWpC;

		for( int leg = 0; leg <= 3; leg++ ) {

			// Leg X way points
			nextPose.setIdentity();
			int nWp = floor(legs[leg] / this->dropDist);
			//dropDist = legs[leg] / nWp;
			for( int i = 0; i < nWp; i++ ) {
				nextPose.setPosition(dropDist, 0, 0);
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

		}

		// offset way point
		nextPose.setIdentity();
		nextPose.setPosition(pathOffset, 0, 0);
		currPose = currPose.compose(nextPose);
		wp.pose = currPose;
		path.push_back(wp);


		// Inc loop number
		loopNum++;

	}

	return true;
}

