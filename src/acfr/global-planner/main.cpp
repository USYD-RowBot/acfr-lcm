#include "LegPath.h"


int main(int argc, char **argv) {
    
	LegPath p;

	p.setPosition( 0, 0, 0 );
	p.setHeadingDeg(45.);
	p.setLength(100);
	p.setDropDist(5.0);
	p.setPitchDeg(5.);
	p.calcPath();

    return 0;
}
