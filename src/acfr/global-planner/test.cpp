// to compile: g++ test.cpp mission.cpp ZamboniePath.cpp GotoPath.cpp LegPath.cpp SpiralPath.cpp SpiralInwardPath.cpp  -o test `pkg-config --cflags --libs libxml++-2.6`


#include "mission.hpp"

int main(int argc, char ** argv)
{
    Mission mis;
    mis.load((string)argv[1]);
    mis.dumpMatlab((string)argv[2]);
    
    return 1;
}
