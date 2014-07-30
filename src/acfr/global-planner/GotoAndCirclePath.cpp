/*
 * GotoAndCirclePath.cpp
 *
 *  Created on: Nov 28, 2013
 *      Author: navid
 */

#include "GotoAndCirclePath.h"

bool GotoAndCirclePath::calcPath(void) {

	waypoint wp;
	Pose3D pRel;
	double finalWPVel = 0;
	cout << "numLoops = " << numLoops << endl
			<< "timeout = " << timeout << endl
			<< "leg vel = " << velocity << endl
			<< "FinalWPVel = " << finalWPVel << endl;
#if 1
	if( numLoops == 0 && timeout == 0 ) {
		cerr << "Either number of loops or timeout has to be set" << endl;
		return false;
	}
	else if( numLoops == 0 && timeout > 0 ) {
		// calculate number of loops based on the timeout and the velocity
		numLoops = timeout * velocity;
		finalWPVel = velocity;
	}
	else if( numLoops > 0 && timeout == 0 ) {
		// calculate the timeout based on the nubmer of loops and the velocity
		timeout = numLoops / velocity;
		finalWPVel = velocity;
	}
	else if( numLoops > 0 && timeout > 0 ){
		// calculate the velocity based on the number of loops and the timeout
		// NOTE: the requested velocity in this mode corresponds to the final
		//			waypoint velocity
		finalWPVel = velocity;
		velocity = numLoops / timeout;
	}
	cout << "numLoops = " << numLoops << endl
			<< "timeout = " << timeout << endl
			<< "leg vel = " << velocity << endl
			<< "FinalWPVel = " << finalWPVel << endl;
#endif
	// Go to the desired position
	wp.pose = position;
	wp.pose.setRollPitchYawRad(0, 0, heading);
	path.push_back(wp);

	// Perform 3/4 a circle
	Pose3D cc;
	cc.setPosition(position.getX(), position.getY() + turnRadius * direction,
			position.getZ());
	cc.setRollPitchYawRad(0, 0,
			atan2(position.getY() - cc.getY(), position.getX() - cc.getX()));
	for (double tT = dropAngle; tT < 1.5 * M_PI; tT += dropAngle) {
		heading += dropAngle * direction;
		while (heading > M_PI)
			heading -= 2 * M_PI;
		while (heading < -M_PI)
			heading += 2 * M_PI;

		pRel.setIdentity();
		pRel.setPosition(turnRadius * cos(tT * direction),
				turnRadius * sin(tT * direction), 0);
		pRel = cc.compose(pRel);
		pRel.setRollPitchYawRad(0, 0, heading);

		wp.pose = pRel;
		path.push_back(wp);
	}

	// Now perform as many loops as necessary to hold here
	cc.setPosition(position.getX(), position.getY(), position.getZ());
	cc.setRollPitchYawRad(0, 0, position.getYawRad()+M_PI);
	for (double tT = 0; tT < 2*M_PI*numLoops; tT += dropAngle) {
		heading += dropAngle * direction;
		while (heading > M_PI)
			heading -= 2 * M_PI;
		while (heading < -M_PI)
			heading += 2 * M_PI;

		pRel.setIdentity();
		pRel.setPosition(turnRadius * cos(tT * direction),
				turnRadius * sin(tT * direction), 0);
		pRel = cc.compose(pRel);
		pRel.setRollPitchYawRad(0, 0, heading);

		wp.pose = pRel;
		path.push_back(wp);
	}

//	cout << "p = [" << endl;
//	for (it = path.begin(); it != path.end(); it++) {
//		cout << (*it).pose.getX() << ", " << (*it).pose.getY() << ", "
//				<< (*it).pose.getZ() << ", " << (*it).pose.getYawRad()/M_PI*180 << endl;
//	}
//	cout << "];" << endl;
//	cout << "plot3d(p', 'r-o'); view(0,90); axis equal" << endl;
	return true;
}
