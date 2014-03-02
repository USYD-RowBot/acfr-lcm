/*	YSI sonde LCM driver
	Christian Lees
	ACFR
	4/5/11
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include "perls-common/nmea.h"
#include "perls-common/generic_sensor_driver.h"
#include "perls-lcmtypes/senlcm_ysi_t.h"

static int myopts(generic_sensor_driver_t *gsd) {
    getopt_add_description (gsd->gopt, "YSI driver.");
    return 0;
}

// parse the nmea string from the YSI sensor
static int parseYsi(char *buf, int buf_len, senlcm_ysi_t *ysi) {
	int i=1;
	char code[16], value[16];
	int valid = 0;

	// set the YSI data structure to non values to start with
	ysi->temperature = -1000.0;
	ysi->depth = -1000.0;
	ysi->turbidity = -1000.0;
	ysi->chlorophyl = -1000.0;
	ysi->conductivity = -1000.0;
	
	while(nmea_arg(buf, i++, code) != 0) {
		//printf("code = %s, ", code);
		// get the associated value
		if(nmea_arg(buf, i++, value) == 1) {
			valid = 1;		// have at least one good value
			//printf("value = %s\n", value);
			switch(atoi(code)) {
				case 1:		// Temp in C
					ysi->temperature = atof(value);
					break;
				case 6:		// Cond in mS/cm
					ysi->conductivity = atof(value);
					break;
				case 22:	// depth in m
					ysi->depth = atof(value);
					break;
				case 12:	// salinity in ppm
					ysi->salinity = atof(value);
					break;
				case 204:	// turbidity in NTU
					ysi->turbidity = atof(value);
					break;
				case 193:	// chlorophyl in ug/l
					ysi->chlorophyl = atof(value);
					break;
				case 211:	// disolved oxygen in %
					ysi->oxygen = atof(value);
					break;
				case 28:	// battery voltage in V
					ysi->battery = atof(value);
					break;	
			}
		}
		else
		    break;
	}
	return valid;
}

int main (int argc, char *argv[]) {
		
    generic_sensor_driver_t *gsd = gsd_create (argc, argv, NULL, myopts);
    gsd_canonical (gsd, '\r','\n');
    gsd_launch (gsd);
    
    gsd_flush (gsd);
    gsd_reset_stats (gsd);
    char buf[256];
    senlcm_ysi_t ysi;
    
    sprintf( buf, "%c", 0x0B );
    gsd_write(gsd, buf, 1); // esc char (0x0B == \e)
    gsd_write(gsd, "nmea", 4); // put it in nmea mode
    
    // loop to collect data, parse and send it on its way
    while(!gsd->done) {
        int64_t timestamp;
        memset(buf, 0, sizeof(buf));
        int len = gsd_read (gsd, buf, 256, &timestamp);
        
		ysi.utime = timestamp;
		if(parseYsi(buf, len, &ysi)) {
			senlcm_ysi_t_publish (gsd->lcm, gsd->channel, &ysi);
				gsd_update_stats (gsd, 1);
        }
        else {
            gsd_update_stats (gsd, -1);
        }
	}

    return 0;
}
