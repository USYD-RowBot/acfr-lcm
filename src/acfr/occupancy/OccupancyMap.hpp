/*
 * OccupancyMap.hpp
 *
 *  Created on: Mar 23, 2012
 *      Author: s4155021
 */

#ifndef OCCUPANCYMAP_HPP_
#define OCCUPANCYMAP_HPP_

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <libgen.h>

#include <small/linalg.hh>
#include <small/Pose3D.hh>
#include <lcm/lcm-cpp.hpp>
#include "perls-common/lcm_util.h"
#include "perls-common/timestamp.h"
//#include "perls-lcmtypes/senlcm_oas_t.h"
#include "perls-lcmtypes++/senlcm/micron_ping_t.hpp"
#include "perls-lcmtypes++/senlcm/rdi_pd5_t.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_occmap_t.hpp"


using namespace std;
using namespace SMALL;

#define DTOR M_PI / 180
#define RTOD 180 / M_PI

class OccMapCell {
public:
	Vector3D loc;
	double votes;
	double timestamp;

	OccMapCell()
				: votes( 0 ), timestamp(0) {
	}
	OccMapCell( Vector3D & l, int v, double t )
				: loc(l), votes(v), timestamp(t) {
	}

	std::string toString( void ) const {
		char numStr[32];
		sprintf( numStr, "%.4f", this->votes );
		//std::string s = this->pose.transposed().toString() + " >> (" + (string)numStr + ")";
		std::string s = this->loc.transposed().toString() + "\t" + (string)numStr + "";
		return s;
	}
};

typedef vector<OccMapCell> OccMap;
typedef vector<OccMapCell>::iterator OccMapIterator;

typedef struct STATE {

    Pose3D pose;
    Pose3D posePrev;
    std::string vehicle_name;
    double time;
        
	double MAP_HALF_TIME; // [s]
	double MAP_HIT_VAL;
	double MAP_MAX_VAL; // hits
	double MAP_MIN_VAL; // below this a cell is invalid
	double voxelSize; // [m]
	double mapSize[3]; // [m]

	Matrix44 OAS_T; // sensor-robot-com transformation
	double OAS_MIN_RANGE; // [m]
	double OAS_MAX_RANGE; // [m]
	double OAS_CONEANGLE;

	Matrix44 RDI_T; // sensor-robot-com transformation
	double RDI_MIN_RANGE; // [m]
	double RDI_MAX_RANGE; // [m]
	double RDI_CONEANGLE;
	
	Matrix44 MICRON_T; // sensor-robot-com transformation

	OccMap map;
	OccMapIterator iter;

	lcm::LCM lcm;

} state_t ;

#endif /* OCCUPANCYMAP_HPP_ */
