#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "perls-common/bot_util.h"
#include "perls-common/error.h"
#include "perls-common/units.h"

#include "navigator.h"

// convert latitude in radians to earth rate in rads/sec
double
lat_to_earth_rate (const double latitude_rad)
{
    return -15.04107 * sin (latitude_rad) * UNITS_DEGREE_TO_RADIAN / (60.*60.);
}


//------------------------------------------------------------------------------
// Push the most recent predict time onto the delayed state utime buffer
void
push_utime_delayed_states (int64_t *utime_ds, int64_t utime) {
    
    for (int i=MAX_DELAYED_STATES ; i>0 ; i--) {
        utime_ds[i] = utime_ds[i-1];
    }
    utime_ds[0] = utime;
}

void
remove_utime_delayed_states (int64_t *utime_ds, int index) {
    for (int i=index ; i<MAX_DELAYED_STATES ; i++) {
        utime_ds[i] = utime_ds[i+1];
    }
    utime_ds[MAX_DELAYED_STATES] = 0;
}
