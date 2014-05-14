/*
 * CoveragePath.cpp
 *
 *  Created on: Mar 10, 2014
 *      Author: navid
 */

#include "CoveragePath.h"

bool CoveragePath::calcPath(void) {

	SpiralInwardPath * spiral;
	ZamboniePath * zambonie;

	cout << endl << "====================" << endl;
	cout << "CoveragePath" << endl;
	cout << "Start Pose = " << position.getX() << ", " << position.getY() << ", " << position.getZ() << endl;
	cout << "Length/Width = " << length << "/" << width << endl;
	cout << "Direction = " << (direction == DIRECTION_CCW ? "Counter " : "") << "Clockwise" << endl;
	cout << "Offset   = " << this->pathOffset << endl;
	cout << "TurnRad  = " << this->turnRadius << endl;
	cout << "DropDist = " << this->dropDist << endl;
	cout << "DropAngl = " << this->dropAngle / M_PI * 180 << endl;


	bool error = false;
	bool doZambonie = false;
	bool doSpiral = false;
	if( this->length <= 0 || this->width <= 0 || this->pathOffset <= 0 ) {
		cerr << "Error: Invalid width/length/offset" << endl;
		error = true;
	}
	// minimum width/length to perform zambonie
	double minZambonieLength = 2*turnRadius;
	double minZambonieWidth = 4 * turnRadius + 2 * pathOffset + 2 * centerOverlap;
	if( this->length < minZambonieLength ) {
		cerr << "Error: Area LENGTH too short. "
				<< "Min l=2*turnRadius=" << minZambonieLength << "m" << endl;
		error = true;
	}
	if (this->width < minZambonieWidth ) {
		cerr << "Error: Area WIDTH too short. "
				<< "Min w=4*turnRadius+2*pathOffset+2*centerOverlap="
				<< minZambonieWidth << "m" << endl;
		error = true;
	}
	doZambonie = true;
	// minimum width/length to perform spiral
	if( length > (minZambonieLength + 2*pathOffset) && width > (minZambonieWidth + 2*pathOffset)) {
		doSpiral = true;
	}
	if( error ) {
		cerr << "Fix this and try again. Exiting..." << endl;
		return false;
	}


	path.clear();
	list<waypoint> tmpPath;
	if(doSpiral) {
		int numLoops = min(
				ceil((length - minZambonieLength) / (2*pathOffset)),
				ceil((width - minZambonieWidth)/(2*pathOffset))
				);
		cout << endl << "Performing " << numLoops <<
				" loops of inward spirals first " << endl;

		spiral = new SpiralInwardPath( position, heading, direction, turnRadius,
				dropDist, dropAngle, pathOffset, width, length );
		spiral->setNumLoops(numLoops);
		if( spiral->calcPath() ) {
			tmpPath = spiral->getPath();
			path.insert(path.begin(), tmpPath.begin(), tmpPath.end());
			spiral->printPath();

			// now set the new start position for the Zambonie
			position.setX( position.getX() + numLoops * pathOffset );
			position.setY( position.getY() -direction * numLoops * pathOffset );
			length -= numLoops * 2*pathOffset;
			width  -= numLoops * 2*pathOffset;
		}
	}

	if(doZambonie ) {
		zambonie = new ZamboniePath( position, heading, direction, turnRadius,
				dropDist, dropAngle, pathOffset, width, length );
		if( zambonie->calcPath() ) {
			tmpPath = zambonie->getPath();
			list<waypoint>::iterator it = path.begin();
			if( path.size() > 0 )
				it = path.end();
			path.insert(it, tmpPath.begin(), tmpPath.end());

			zambonie->printPath();
		}

	}



	return (path.size() > 0);
}

