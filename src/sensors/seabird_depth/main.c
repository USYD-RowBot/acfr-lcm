/* 	Seabird Depth LCM module
	uses code lifted from the original vehicle code, SIO_seabird_depth.c
	and uses ecopuck as a basis
	
	17/5/2011
	Lachlan Toohey
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include <bot_param/param_client.h>

#include "perls-common/units.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-lcmtypes/senlcm_seabird_depth_t.h"


// Values taken from SIO_seabird.h from vehicle code.
#define SEABIRD_AUTORUN_CMD   "AUTORUN=Y\r\n"
#define SEABIRD_START_CMD   "START\r\n"
#define SEABIRD_STOP_CMD     "STOP\r\n" /* sample and send one measurement */
#define SEABIRD_NAVG_CMD     "NAVG=1\r\n" /* number of depth values, average 16Hz */
#define SEABIRD_OUTPUTFORMAT_CMD "OUTPUTFORMAT=3\r\n" /* output in metres of salt water */
#define SEABIRD_DECIMALS_CMD "DECIMALS=3\r\n" /* number of decimals */
#define SEABIRD_LAT_CMD      "LATITUDE=%i\r\n" /* Latitude command - should this somehow be variable/configurable? */

// parse routine lifted from SIO_seabird_depth.h from vehicle code
static int parseSeabirdDepth(char *buf, int buf_len, senlcm_seabird_depth_t *seabird_depth) { 
    double depth;

    if (1 == sscanf(buf,"%lf", &depth))
    {
        seabird_depth->depth = depth;
        return 1;
    }
    else
    {
        return 0;
    }
}

static int myopts(generic_sensor_driver_t *gsd) {
    getopt_add_description (gsd->gopt, "Seabird Depth driver.");
    getopt_add_int(gsd->gopt, 'l', "latitude", "22", "Approximate Latitude for Seabird Depth");
    return 0;
}

int main (int argc, char *argv[]) {
		
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_canonical (gsd, '\r','\n');
    gsd_launch (gsd);

    int latitude = getopt_get_int(gsd->gopt, "latitude");

    // Configuration taken from SIO_seabird_depth.c from vehicle code
    gsd_write(gsd, SEABIRD_STOP_CMD, strlen(SEABIRD_STOP_CMD));
    gsd_write(gsd, SEABIRD_NAVG_CMD, strlen(SEABIRD_NAVG_CMD));
    gsd_write(gsd, SEABIRD_OUTPUTFORMAT_CMD, strlen(SEABIRD_OUTPUTFORMAT_CMD));
    gsd_write(gsd, SEABIRD_DECIMALS_CMD, strlen(SEABIRD_DECIMALS_CMD));

    char buffer[256];

    snprintf(buffer, 256, SEABIRD_LAT_CMD, latitude);

    gsd_write(gsd, buffer, strlen(buffer));

    gsd_write(gsd, SEABIRD_AUTORUN_CMD, strlen(SEABIRD_AUTORUN_CMD));
    gsd_write(gsd, SEABIRD_START_CMD, strlen(SEABIRD_START_CMD));

    gsd_flush (gsd);
    gsd_reset_stats (gsd);
    
    // loop to collect data, parse and send it on its way
    while(!gsd->done) {
    	char buf[256];
        int64_t timestamp;
        int len = gsd_read (gsd, buf, 256, &timestamp);
        
        senlcm_seabird_depth_t seabird_depth;
        seabird_depth.utime = timestamp;
        if(parseSeabirdDepth(buf, len, &seabird_depth)) {
            senlcm_seabird_depth_t_publish (gsd->lcm, gsd->channel, &seabird_depth);
            gsd_update_stats (gsd, 1);
        }
        else
            gsd_update_stats (gsd, -1);
    }
}

		
    
    
    

