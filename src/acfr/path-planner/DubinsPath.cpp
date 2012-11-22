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

/** **************************************************
 *  Constructor
 */
DubinsPath::DubinsPath(double _circRad, double _dropDist, double _dropAngle) :
		minSide1(0), minSide2(0), minDist(0), minDistI(0), circleRad(_circRad), dropDist(
				_dropDist), dropAngle(_dropAngle) {

	this->side[0] = right;
	this->side[1] = left;
	this->sideStr[0] = "right";
	this->sideStr[1] = "left";

}

/** **************************************************
 *  Calculates the shortest feasible Dubin's path.
 */
vector<Pose3D> DubinsPath::calcPath(Pose3D currPose, Pose3D destPose) {

	int circleSide1;
	int circleSide2;
	Pose2D c1;
	Pose2D c2;
	Pose2D p1;
	Pose2D q1;
	Pose2D p2;
	Pose2D q2;
	bool useP1 = false;
	bool useQ1 = false;
	bool useP2 = false;
	bool useQ2 = false;

	Pose2D pose1[4];
	Pose2D pose2[4];
	//Line2D connectionLine;

	double dists[4];

	// Reset for this run
	this->minDist = numeric_limits<double>::max();
	this->minDistI = -1;

	Pose2D targRel = currPose.transformFrom(destPose.getPosition());

	int run = 0;
	bool tooClose = false;
	//
	//  1) Source Circle
	//
	for (int i = 0; i < 2; i++) {
//		cout << "\n\ni = " << i << endl;

		circleSide1 = this->side[i];
		double deltaX = this->circleRad * cos(M_PI / 2 - currPose.getYawRad())
				* -circleSide1;
		double deltaY = this->circleRad * sin(M_PI / 2 - currPose.getYawRad())
				* +circleSide1;
		c1.set(currPose.getX() + deltaX, currPose.getY() + deltaY,
				currPose.getYawRad());

//		cout << "c1: " << c1.toString() << endl;

		//
		// 2) Target Circle
		//
		for (int j = 0; j < 2; j++) {
			run = 2 * (i) + j;
//			cout << "\tRun" << run + 1 << " " << sideStr[i] << "-" << sideStr[j]
//					<< endl;

			tooClose = false;

			circleSide2 = side[j];
			double deltaX = this->circleRad
					* cos(M_PI / 2 - destPose.getYawRad()) * -circleSide2;
			double deltaY = this->circleRad
					* sin(M_PI / 2 - destPose.getYawRad()) * +circleSide2;
			c2.set(destPose.getX() + deltaX, destPose.getY() + deltaY,
					destPose.getYawRad());

//			cout << "\tc2: " << c2.toString() << endl;

			// Check for overlap
			double dist = c1.positionDistance(c2);
			if (dist < 2 * this->circleRad) {
//				cout << "Circles overlap! No X-tangent solution possible."
//						<< endl;
				tooClose = true;
			}

			//
			// 3) Angle of line between circle centers
			//
			double lineC2CTheta = atan2(c2.getY() - c1.getY(),
					c2.getX() - c1.getX());
			double lineC2CLen = hypot(c2.getX() - c1.getX(),
					c2.getY() - c1.getY());
//			cout << "lineC2CTheta " << lineC2CTheta / M_PI * 180
//					<< ", lineC2CLen " << lineC2CLen << endl;

			//
			// 4) Calculate rotation on source and target side
			//      and parallel tangents
			//
			// The perpendicular points
			double dX = this->circleRad * sin(lineC2CTheta);
			double dY = this->circleRad * cos(lineC2CTheta);
			p1.set(c1.getX() + dX, c1.getY() - dY, lineC2CTheta + M_PI / 2);
			q1.set(c1.getX() - dX, c1.getY() + dY, lineC2CTheta + M_PI / 2);
			p2.set(c2.getX() + dX, c2.getY() - dY, lineC2CTheta + M_PI / 2);
			q2.set(c2.getX() - dX, c2.getY() + dY, lineC2CTheta + M_PI / 2);
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
			c1p1 = (p1.getX() - c1.getX()), (p1.getY() - c1.getY());
			Vector2D c1q1;
			c1q1 = (q1.getX() - c1.getX()), (q1.getY() - c1.getY());
			double detc1p1p2 = c1p1[0] * p1p2[1] - c1p1[1] * p1p2[0];
			double detc1p1q2 = c1p1[0] * p1q2[1] - c1p1[1] * p1q2[0];
			double detc1q1p2 = c1q1[0] * q1p2[1] - c1q1[1] * q1p2[0];
			double detc1q1q2 = c1q1[0] * q1q2[1] - c1q1[1] * q1q2[0];
			// Determinants for target circle
			Vector2D p2p1;
			p2p1 = (p1 - p2).getX(), (p1 - p2).getY();
			Vector2D q2p1;
			q2p1 = (p1 - q2).getX(), (p1 - q2).getY();
			Vector2D p2q1;
			p2q1 = (q1 - p2).getX(), (q1 - p2).getY();
			Vector2D q2q1;
			q2q1 = (q1 - q2).getX(), (q1 - q2).getY();
			Vector2D p2c2;
			p2c2 = (c2.getX() - p2.getX()), (c2.getY() - p2.getY());
			Vector2D q2c2;
			q2c2 = (c2.getX() - q2.getX()), (c2.getY() - q2.getY());
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
				pose1[run] = Pose2D(c1.getX() + dX, c1.getY() + dY, 0.0);
				pose2[run] = Pose2D(c2.getX() - dX, c2.getY() - dY, 0.0);
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
				pose1[run] = Pose2D(c1.getX() + dY, c1.getY() + dX, 0.0);
				pose2[run] = Pose2D(c2.getX() - dY, c2.getY() - dX, 0.0);
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
			pose1[run].setThetaRad(connectionLineTheta);
			pose2[run].setThetaRad(connectionLineTheta);
//			cout << "source " << pose1[run].toString() << endl;
//			cout << "target " << pose2[run].toString() << endl;

			////
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
			if (dist < minDist) {
				minDist = dist;
				minDistI = run;
				minPose1 = pose1[minDistI];
				minPose2 = pose2[minDistI];
				minC1 = c1;
				minC2 = c2;
				minSide1 = side[i];
				minSide2 = side[j];
			}

		}
	}





	/* ************************************************************************
	 *
	 * Now calculate the way points along the shortest path
	 *
	 */
	this->path.clear();
	if( minDistI == -1 ) {
		cerr << "No feasible path found! " << endl;
		return this->path;
	}

//	cout << "Shortest path was at run " << minDistI << endl;
//	cout << "pathLength " << minDist << endl;

	double dx;
	double dy;
	double dt;
	double x;
	double y;
	double t;

	// Calc the limits
	double angle1 = (minPose1.getYawRad() - currPose.getYawRad()) * minSide1;
	while (angle1 < 0) {
		angle1 += 2 * M_PI;
	}
//	while (angle1 > M_PI) {
//		angle1 -= 2 * M_PI;
//	}
	double straightDist = minPose1.positionDistance(minPose2);
	double angle2 = (destPose.getYawRad() - minPose2.getYawRad()) * minSide2;
	while (angle2 < 0) {
		angle2 += 2 * M_PI;
	}
//	while (angle2 > M_PI) {
//		angle2 -= 2 * M_PI;
//	}

	// Circle 1 path
	dx = minC1.getX();
	dy = minC1.getY();
	dt = currPose.getYawRad() - M_PI / 2;
	double T1[4][4] = { { cos(dt), -sin(dt), 0, dx },
			{ sin(dt), cos(dt), 0, dy }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
	for (double tT = 0; tT < angle1; tT += dropAngle) {
		x = this->circleRad * cos(tT) * minSide1;
		y = this->circleRad * sin(tT);
		t = tT;
		double p[] = { x, y, t, 1 };
		Pose2D pRel;
		pRel.setX(
				T1[0][0] * p[0] + T1[0][1] * p[1] + T1[0][2] * p[2]
						+ T1[0][3] * p[3]);
		pRel.setY(
				T1[1][0] * p[0] + T1[1][1] * p[1] + T1[1][2] * p[2]
						+ T1[1][3] * p[3]);
		pRel.setThetaRad(dt + M_PI / 2 + tT * minSide1); //pRel.setTheta( T1[2][0]*p[0]+T1[2][1]*p[1]+T1[2][2]*p[2]+T1[2][3]*p[3] );
		path.push_back(pRel); //path.insert( pathIter++, pRel );
	}

	// Straight line path
	dx = minPose1.getX();
	dy = minPose1.getY();
	dt = minPose1.getYawRad();
	double T2[4][4] = { { cos(dt), -sin(dt), 0, dx },
			{ sin(dt), cos(dt), 0, dy }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
	for (double i = 0; i < straightDist; i += dropDist) {
		x = i;
		y = 0;
		t = dt;
		double p[] = { x, y, t, 1 };
		Pose2D pRel;
		pRel.setX(
				T2[0][0] * p[0] + T2[0][1] * p[1] + T2[0][2] * p[2]
						+ T2[0][3] * p[3]);
		pRel.setY(
				T2[1][0] * p[0] + T2[1][1] * p[1] + T2[1][2] * p[2]
						+ T2[1][3] * p[3]);
		//pRel.setTheta( T2[2][0]*p[0]+T2[2][1]*p[1]+T2[2][2]*p[2]+T2[2][3]*p[3] );
		pRel.setThetaRad(dt);
		path.push_back(pRel); //path.insert( pathIter++, pRel );
	}


	// Circle 2 path
	dx = minC2.getX();
	dy = minC2.getY();
	dt = minPose2.getYawRad() - M_PI / 2;
	double T3[4][4] = { { cos(dt), -sin(dt), 0, dx },
			{ sin(dt), cos(dt), 0, dy }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
	for (double tT = 0; tT < angle2; tT += dropAngle) {
		x = this->circleRad * cos(tT) * minSide2;
		y = this->circleRad * sin(tT);
		t = tT;
		double p[] = { x, y, t, 1 };
		Pose2D pRel;
		pRel.setX(
				T3[0][0] * p[0] + T3[0][1] * p[1] + T3[0][2] * p[2]
						+ T3[0][3] * p[3]);
		pRel.setY(
				T3[1][0] * p[0] + T3[1][1] * p[1] + T3[1][2] * p[2]
						+ T3[1][3] * p[3]);
		pRel.setThetaRad(dt + M_PI / 2 + tT * minSide2); //pRel.setTheta( T3[2][0]*p[0]+T3[2][1]*p[1]+T3[2][2]*p[2]+T3[2][3]*p[3] );
		path.push_back(pRel); //path.insert( pathIter, pRel );
	}


	return this->path;
}

