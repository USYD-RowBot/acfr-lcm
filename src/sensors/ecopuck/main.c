/* 	Ecopuck LCM module
	uses code lifted from the original vehicle code, SIO_ecopuck.c
	
	30/9/10
	Christian Lees
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>


#include "perls-common/units.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-lcmtypes/senlcm_ecopuck_t.h"

#define INVALID_CHLOROPHYLL  -10.0   /* return value if bad read       */
#define INVALID_CDOM         -10.0   /* return value if bad read       */
#define INVALID_TURBIDITY    -10.0   /* return value if bad read       */
#define INVALID_TEMPERATURE  -10.0        /* return value if bad read */
#define ECOPUCK_LOG_FILE       "RAW"       /* extension of raw log data file */
#define ECOPUCK_START_CMD      "$run\n"
#define ECOPUCK_STOP_CMD		"!!!!!"
#define ECOPUCK_CHL_SF    0.0127 /* scale factor (ug/l)/ count */
#define ECOPUCK_CHL_DC    50    /* dark counts */
#define ECOPUCK_NTU_SF    0.000003805 /* scale factor (m^-1 sr^-1)/count */
#define ECOPUCK_NTU_DC    54     /* dark count */
#define ECOPUCK_CDOM_SF   0.1002 /* scale factor ppb/count */
#define ECOPUCK_CDOM_DC   47     /* dark count */


// parse routine lifted from SIO_ecopuck.h from vehicle code
static int parseEcopuck(char *buf, int buf_len, senlcm_ecopuck_t *ecopuck) { 
	int chl_counts;
	int trb_counts;
	int cdom_counts;
	int term_counts;

	if (4 == sscanf(buf,"%*i/%*i/%*i %*i:%*i:%*i %*i %i %*i %i %*i %i %i\n", &trb_counts,&chl_counts,&cdom_counts,&term_counts)) { 
	  	ecopuck->chlorophyll = ((float)(chl_counts-ECOPUCK_CHL_DC))*ECOPUCK_CHL_SF;
	 	ecopuck->turbidity = ((float)(trb_counts-ECOPUCK_NTU_DC))*ECOPUCK_NTU_SF;
	    // FIX if using triplet
	  	ecopuck->cdom = ((float)(cdom_counts-ECOPUCK_CDOM_DC))*ECOPUCK_CDOM_SF;

		// Thermistor is not fitted to the ecopuck version of the triplet sensor.
	  	ecopuck->temperature = INVALID_TEMPERATURE;
      	return 1;
	}
	else
		return 0;
}

static int myopts(generic_sensor_driver_t *gsd) {
    getopt_add_description (gsd->gopt, "Ecopuck driver.");
    return 0;
}

int main (int argc, char *argv[]) {
		
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_canonical (gsd, '\r','\n');
    gsd_launch (gsd);
    
    gsd_flush (gsd);
    gsd_reset_stats (gsd);
    
    // loop to collect data, parse and send it on its way
    while(!gsd->done) {
    	char buf[256];
        int64_t timestamp;
        int len = gsd_read (gsd, buf, 256, &timestamp);
        
        senlcm_ecopuck_t ecopuck;
   		ecopuck.utime = timestamp;
		if(parseEcopuck(buf, len, &ecopuck)) {
			senlcm_ecopuck_t_publish (gsd->lcm, gsd->channel, &ecopuck);
            gsd_update_stats (gsd, 1);
        }
        else
            gsd_update_stats (gsd, -1);
	}
}

		
    
    
    

