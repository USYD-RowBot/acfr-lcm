//
// C++ Implementation: DubinsPath
//
// Description:
//
//
// Author: Navid Nourani <navid.nourani@csiro.au>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//

#include "DubinsPath.h"

const int DubinsPath::side[2] = {right, left};


/** **************************************************
 *  Constructor
 */
DubinsPath::DubinsPath() :
		minDistI(0), minDist(0), minPitch(0), minSide1(0), minSide2(0), circleRad(
				1.0), maxPitch(0.0), dropDist(1.0), angle1_report(0), angle2_report(
				0) {
	setWaypointDropAngleFromDropDist();
	sideStr.push_back("right");
	sideStr.push_back("left");
}

DubinsPath::DubinsPath(double _circRad, double _maxPitch, double _dropDist,
		double _dropAngle) :
		minDistI(0), minDist(0), minPitch(0), minSide1(0), minSide2(0), circleRad(
				_circRad), maxPitch(_maxPitch), dropDist(_dropDist), dropAngle(
				_dropAngle), angle1_report(0), angle2_report(0) {

	sideStr.push_back("right");
	sideStr.push_back("left");
}

/** **************************************************
 *  Calculates the shortest feasible Dubin's path.
 */
vector<Pose3D> DubinsPath::calcPath(Pose3D currPose, Pose3D destPose) {

	int circleSide1;
	int circleSide2;
	Pose3D cSrc;
	Pose3D cDest;
	Pose3D circRelPose;
	Pose3D p1;
	Pose3D q1;
	Pose3D p2;
	Pose3D q2;
	bool useP1 = false;
	bool useQ1 = false;
	bool useP2 = false;
	bool useQ2 = false;

	Pose3D pose1[4];
	Pose3D pose2[4];
	double dists[4];
	this->minDist = numeric_limits<double>::max();
	this->minDistI = -1;

	int run = 0;
	bool tooClose = false;

	//
	//  1) Source Circle
	//
	for (int i = 0; i < 2; i++) {
//		cout << "Source circle on " << sideStr[i] << endl;

		// Center of source circle
		circleSide1 = this->side[i]; // defines left or right side

		circRelPose.setY(this->circleRad * this->side[i]);
		cSrc = currPose.compose(circRelPose);
		cSrc.setRollPitchYawRad(0, 0, currPose.getYawRad() - M_PI / 2);

		//
		// 2) Dest Circle
		//
		for (int j = 0; j < 2; j++) {
//			cout << "Dest circle on " << sideStr[j] << endl;
			run = 2 * (i) + j;

			tooClose = false;

			// Center of destination circle
			circleSide2 = side[j];

			circRelPose.setY(this->circleRad * this->side[j]);
			cDest = destPose.compose(circRelPose);
			cDest.setRollPitchYawRad(0, 0, destPose.getYawRad() - M_PI / 2);

			// Check for overlap between the circles
			double dist = cSrc.positionDistance(cDest);
			if (dist < 2 * this->circleRad) {
//				cout << "Circles overlap! No X-tangent solution possible."
//						<< endl;
				tooClose = true;
			}

			//
			// 3) Angle of line between circle centers
			//
			double lineC2CTheta = atan2(cDest.getY() - cSrc.getY(),
					cDest.getX() - cSrc.getX());
			double lineC2CLen = hypot(cDest.getX() - cSrc.getX(),
					cDest.getY() - cSrc.getY());
//			cout << "lineC2CTheta " << lineC2CTheta / M_PI * 180
//					<< ", lineC2CLen " << lineC2CLen << endl;

			//
			// 4) Calculate rotation on source and target side
			//      and parallel tangents
			//
			// The perpendicular points
			double dX = this->circleRad * sin(lineC2CTheta);
			double dY = this->circleRad * cos(lineC2CTheta);
			p1.setPosition(cSrc.getX() + dX, cSrc.getY() - dY, 0);
			p1.setRollPitchYawRad(0, 0, lineC2CTheta + M_PI / 2);
			q1.setPosition(cSrc.getX() - dX, cSrc.getY() + dY, 0);
			q1.setRollPitchYawRad(0, 0, lineC2CTheta + M_PI / 2);
			p2.setPosition(cDest.getX() + dX, cDest.getY() - dY, 0);
			p2.setRollPitchYawRad(0, 0, lineC2CTheta + M_PI / 2);
			q2.setPosition(cDest.getX() - dX, cDest.getY() + dY, 0);
			q2.setRollPitchYawRad(0, 0, lineC2CTheta + M_PI / 2);
//			cout << "p1 " << p1.toString() << endl;
//			cout << "q1 " << q1.toString() << endl;
//			cout << "p2 " << p2.toString() << endl;
//			cout << "q2 " << q2.toString() << endl;

			// Determinants of source circle
			Vector2D p1p2;
			p1p2 = (p2.getX() - p1.getX()), (p2.getY() - p1.getY());
			Vector2D p1q2;
			p1q2 = (q2.getX() - p1.getX()), (q2.getY() - p1.getY());
			Vector2D q1p2;
			q1p2 = (p2.getX() - q1.getX()), (p2.getY() - q1.getY());
			Vector2D q1q2;
			q1q2 = (q2.getX() - q1.getX()), (q2.getY() - q1.getY());
			Vector2D c1p1;
			c1p1 = (p1.getX() - cSrc.getX()), (p1.getY() - cSrc.getY());
			Vector2D c1q1;
			c1q1 = (q1.getX() - cSrc.getX()), (q1.getY() - cSrc.getY());
			double detc1p1p2 = c1p1[0] * p1p2[1] - c1p1[1] * p1p2[0];
			double detc1p1q2 = c1p1[0] * p1q2[1] - c1p1[1] * p1q2[0];
			double detc1q1p2 = c1q1[0] * q1p2[1] - c1q1[1] * q1p2[0];
			double detc1q1q2 = c1q1[0] * q1q2[1] - c1q1[1] * q1q2[0];
			// Determinants for target circle
			Vector2D p2p1;
			p2p1 = p1.getX() - p2.getX(), p1.getY() - p2.getY();
			Vector2D q2p1;
			q2p1 = p1.getX() - q2.getX(), p1.getY() - q2.getY();
			Vector2D p2q1;
			p2q1 = q1.getX() - p2.getX(), q1.getY() - p2.getY();
			Vector2D q2q1;
			q2q1 = q1.getX() - q2.getX(), q1.getY() - q2.getY();
			Vector2D p2c2;
			p2c2 = cDest.getX() - p2.getX(), cDest.getY() - p2.getY();
			Vector2D q2c2;
			q2c2 = cDest.getX() - q2.getX(), cDest.getY() - q2.getY();
			double detp1p2c2 = p1p2[0] * p2c2[1] - p1p2[1] * p2c2[0];
			double detq1p2c2 = q1p2[0] * p2c2[1] - q1p2[1] * p2c2[0];
			double detp1q2c2 = p1q2[0] * q2c2[1] - p1q2[1] * q2c2[0];
			double detq1q2c2 = q1q2[0] * q2c2[1] - q1q2[1] * q2c2[0];
			// source point is p1
			if (detc1p1p2 * circleSide1 > 0 && detc1p1q2 * circleSide1 > 0) {
				pose1[run] = p1;
				useP1 = true;
				useQ1 = false;
//				cout << "useP1" << endl;
			}
			// source point is q1
			else if (detc1q1p2 * circleSide1 > 0
					&& detc1q1q2 * circleSide1 > 0) {
				pose1[run] = q1;
				useQ1 = true;
				useP1 = false;
//				cout << "useQ1" << endl;
			}
			// target point is p2
			if (detp1p2c2 * circleSide2 > 0 && detq1p2c2 * circleSide2 > 0) {
				pose2[run] = p2;
				useP2 = true;
				useQ2 = false;
//				cout << "useP2" << endl;
			}
			// target point is q2
			else if (detp1q2c2 * circleSide2 > 0
					&& detq1q2c2 * circleSide2 > 0) {
				pose2[run] = q2;
				useQ2 = true;
				useP2 = false;
//				cout << "useQ2" << endl;
			}

			//
			// 4b) Find the X-tangent if necessary
			//
			if ((useP1 && useP2) || (useQ1 && useQ2)) {
//				cout << "NO need to calculate the X-tangent" << endl;
			} else if ((useP1 && useQ2) && !tooClose) {
//				cout << "Calculating the X-tangent: P1Q2" << endl;
				double a = lineC2CLen / 2;
				double alpha = acos(this->circleRad / a);
//				cout << "alpha " << alpha / M_PI * 180 << endl;
				double theta = lineC2CTheta - alpha;
//				cout << "theta " << theta / M_PI * 180 << endl;

				double dX = this->circleRad * cos(theta);
				double dY = this->circleRad * sin(theta);
//				cout << "dX " << dX << endl;
//				cout << "dY " << dY << endl;
				pose1[run] = Pose2D(cSrc.getX() + dX, cSrc.getY() + dY, 0.0);
				pose2[run] = Pose2D(cDest.getX() - dX, cDest.getY() - dY, 0.0);
			} else if ((useQ1 && useP2) && !tooClose) {
//				cout << "Calculating the X-tangent: Q1P2" << endl;
				double a = lineC2CLen / 2;
				double alpha = acos(this->circleRad / a);
//				cout << "alpha " << alpha / M_PI * 180 << endl;
				double theta = M_PI / 2 - lineC2CTheta - alpha;
//				cout << "theta " << theta / M_PI * 180 << endl;

				double dX = this->circleRad * cos(theta);
				double dY = this->circleRad * sin(theta);
//				cout << "dX " << dX << endl;
//				cout << "dY " << dY << endl;
				pose1[run] = Pose2D(cSrc.getX() + dY, cSrc.getY() + dX, 0.0);
				pose2[run] = Pose2D(cDest.getX() - dY, cDest.getY() - dX, 0.0);
			} else {
//				cout << "No solution possible!!\n" << endl;
				dists[run] = numeric_limits<double>::max();
				continue;
			}

			//
			// 4c) Set connection Line
			//
			double connectionLineTheta = atan2(
					pose2[run].getY() - pose1[run].getY(),
					pose2[run].getX() - pose1[run].getX());
			double connectionLineLen = hypot(
					pose2[run].getX() - pose1[run].getX(),
					pose2[run].getY() - pose1[run].getY());
			pose1[run].setRollPitchYawRad(0, 0, connectionLineTheta);
			pose2[run].setRollPitchYawRad(0, 0, connectionLineTheta);
//			cout << "source " << pose1[run].toString() << endl;
//			cout << "target " << pose2[run].toString() << endl;

			//
			// 5) Calc Dist travelled
			//
			dist = 0;
			// First Turn
			double angleDiff1 = (connectionLineTheta - currPose.getYawRad())
					* side[i];
			while (angleDiff1 < 0) {
				angleDiff1 = angleDiff1 + 2 * M_PI;
			}
			while (angleDiff1 > 2 * M_PI) {
				angleDiff1 = angleDiff1 - 2 * M_PI;
			}
			double d = this->circleRad * angleDiff1;
			dist += d;
//			cout << "Source Turn   : " << d << "m <= "
//					<< currPose.getYawRad() / M_PI * 180 << " -> "
//					<< connectionLineTheta / M_PI * 180 << " = "
//					<< angleDiff1 / M_PI * 180 << endl;

			// Straight line
			d = connectionLineLen;
			dist += d;
			//print.info(  "Straight part : " + d + "m "  );
			// Second Turn
			double angleDiff2 = (destPose.getYawRad() - connectionLineTheta)
					* side[j];
			while (angleDiff2 < 0) {
				angleDiff2 = angleDiff2 + 2 * M_PI;
			}
			while (angleDiff2 > 2 * M_PI) {
				angleDiff2 = angleDiff2 - 2 * M_PI;
			}
			d = this->circleRad * angleDiff2;
			dist += d;
//			cout << "Target Turn   : " << d << "m <= "
//					<< connectionLineTheta / M_PI * 180 << " -> "
//					<< destPose.getYawRad() / M_PI * 180 << " = "
//					<< angleDiff2 / M_PI * 180 << endl;

//			cout << "-------------------------" << endl;
//			cout << "Total         : " << dist << "m" << endl;

			//
			// 6) Calculate if the dist travelled is sufficient to adjust the altitude as well
			//
			double pitchRequired = asin(
					(destPose.getZ() - currPose.getZ()) / dist);

			if (pitchRequired > this->maxPitch) {
//				cout << "The required pitch to change the altitude is " << pitchRequired / M_PI * 180 << "deg" << endl;
//				cout << "Distance to travel not sufficient to change the altitude" << endl;
				continue;
			}

			if (dist < minDist) {
				minDist = dist;
				minDistI = run;
				minPose1 = pose1[minDistI];
				minPose2 = pose2[minDistI];
				minC1 = cSrc;
				minC2 = cDest;
				minSide1 = side[i];
				minSide2 = side[j];
				minPitch = pitchRequired;
			}

		}
	}

	/* ************************************************************************
	 *
	 * Now calculate the way points along the shortest path
	 *
	 */
	this->path.clear();
	if (minDistI == -1) {
		cerr << "No feasible path found! " << endl;
		return this->path;
	}

	// Add current pose for heading calculation. NOTE: This way point is removed at the end
	path.push_back(currPose);

	// Calc the limits
	double angle1 = (minPose1.getYawRad() - currPose.getYawRad()) * minSide1;
	while (angle1 < 0) {
		angle1 += 2 * M_PI;
	}
	double straightDist = minPose1.positionDistance(minPose2);
	double angle2 = (destPose.getYawRad() - minPose2.getYawRad()) * minSide2;
	while (angle2 < 0) {
		angle2 += 2 * M_PI;
	}

	angle1_report = angle1 / M_PI * 180;
	angle2_report = angle2 / M_PI * 180;

	//double dist1 = angle1 * this->circleRad;
	//double dist2 = angle2 * this->circleRad;
	//cout << endl << "Shortest Dubins path: " << endl;
	//cout << "\tFirst circle on " << (minSide1 == this->right ? "right" : "left") << endl;
	//cout << "\tSecond circle on " << (minSide2 == this->right ? "right" : "left") << endl;
	//cout << "\tPath length " << minDist << endl;
	//cout << fixed << setprecision(2) << "First circle distance (angle) : " << dist1 << "m (" << angle1 / M_PI * 180
	//		<< "deg)" << endl;
	//cout << "Straight path distance        : " << straightDist << "m" << endl;
	//cout << "Second circle distance (angle): " << dist2 << "m (" << angle2 / M_PI * 180 << ")deg" << endl;

	//cout << "An altitude change of " << destPose.getZ() - currPose.getZ()
	//		<< "m is required. This is achieved with a pitch of " << minPitch / M_PI * 180. << "deg" << endl;

	Pose3D pRel;

	double heading = currPose.getYawRad();
	double distance = 0;
	double Z0 = currPose.getZ();

	// Circle 1 path
	minC1.setRollPitchYawRad(
			0,
			0,
			atan2(currPose.getY() - minC1.getY(),
					currPose.getX() - minC1.getX()));
	for (double tT = dropAngle; tT < angle1; tT += dropAngle) {
		heading += dropAngle * minSide1;
		while (heading > M_PI)
			heading -= 2 * M_PI;
		while (heading < -M_PI)
			heading += 2 * M_PI;

		pRel.setIdentity();
		pRel.setPosition(this->circleRad * cos(tT * minSide1),
				this->circleRad * sin(tT * minSide1), 0);
		pRel = minC1.compose(pRel);

		distance += dropAngle * this->circleRad;
		//pRel.setZ(Z0 + sin(minPitch) * distance);
		pRel.setZ( destPose.getZ() );
		path.push_back(pRel);
	}

	// Straight line path
	for (double i = 0; i < straightDist; i += dropDist) {
		pRel.setIdentity();
		pRel.setPosition(i, 0, 0);
		pRel = minPose1.compose(pRel);

		distance += dropDist;
//		pRel.setZ(Z0 + sin(minPitch) * distance);
		pRel.setZ( destPose.getZ() );
		path.push_back(pRel);
	}

	// Circle 2 path
	minC2.setRollPitchYawRad(
			0,
			0,
			atan2((*(path.end() - 1)).getY() - minC2.getY(),
					(*(path.end() - 1)).getX() - minC2.getX()));
	for (double tT = dropAngle; tT < angle2; tT += dropAngle) {
		heading += dropAngle * minSide2;
		while (heading > M_PI)
			heading -= 2 * M_PI;
		while (heading < -M_PI)
			heading += 2 * M_PI;

		pRel.setIdentity();
		pRel.setPosition(this->circleRad * cos(tT * minSide2),
				this->circleRad * sin(tT * minSide2), 0);
		pRel = minC2.compose(pRel);

		distance += dropAngle * this->circleRad;
//		pRel.setZ(Z0 + sin(minPitch) * distance);
		pRel.setZ( destPose.getZ() );
		path.push_back(pRel);
	}

	// Add the destination pose for heading calculation. This will be removed at the end
	path.push_back(destPose);
	// Set pitch and heading
	for (unsigned int i = 1; i < path.size() - 1; i++) {
		path.at(i).setRollPitchYawRad(
				0,
				minPitch,
				atan2(path.at(i + 1).getY() - path.at(i - 1).getY(),
						path.at(i + 1).getX() - path.at(i - 1).getX()));
	}

	path.erase(path.begin());
//	path.erase(path.end());

	this->pathLength = minDist;
	return this->path;
}

