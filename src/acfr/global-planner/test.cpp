// to compile: g++ test.cpp mission.cpp ZamboniePath.cpp GotoPath.cpp LegPath.cpp SpiralPath.cpp SpiralInwardPath.cpp  -o test `pkg-config --cflags --libs libxml++-2.6`


#include "mission.hpp"

int main()
{
    Mission mis;
    mis.load("./test.xml");
    mis.dumpMatlab();
    
    return 1;
}
