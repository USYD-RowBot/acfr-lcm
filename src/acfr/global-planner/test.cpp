#include "mission.hpp"

int main()
{
    Mission mis;
    mis.load("./test.xml");
    mis.dumpMatlab();
    
    return 1;
}
