#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "perls-common/units.h"

// convert latitude in radians to earth rate in rads/sec
double
lat_to_earth_rate (const double latitude_rad)
{
    return -15.04107 * sin (latitude_rad) * UNITS_DEGREE_TO_RADIAN / (60.*60.);
}


