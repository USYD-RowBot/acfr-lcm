// to compile: g++ test.cpp mission.cpp ZamboniePath.cpp GotoPath.cpp LegPath.cpp SpiralPath.cpp SpiralInwardPath.cpp  -o test `pkg-config --cflags --libs libxml++-2.6`


#include "mission.hpp"
#include "CoveragePath.h"

int main(int argc, char ** argv)
{

	MissionPrimitive * mp = NULL;

	mp = new ZamboniePath();

	mp->setTurnRadius(7.5);
	mp->setLength( 35 );
	mp->setWidth( 40 );
	mp->setDropDist( 5.0 );
	mp->setDropAngleFromDropDist();
	mp->setPathOffset( 1.0 );
	mp->setDirectionCW();

	mp->setPosition( 0, 0, 0 );
	mp->setHeadingDeg(0);

	mp->calcPath();
	mp->printPath();

    return 1;
}
