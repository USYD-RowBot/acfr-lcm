%Module missionXML

%Include list.sip


class WaypointSimple {
%TypeHeaderCode
#include "waypoint.hpp"
%End
public:
    WaypointSimple(double x, double y, double z);
    double x;
    double y;
    double z;
};

class Mission {
%TypeHeaderCode
#include "mission.hpp"
%End

public:
    Mission();

    ~Mission();
    
    int load(char *_filename);
    double getOriginLat();
    double getOriginLon();
    list<WaypointSimple> dumpSimple();
    
    
};
