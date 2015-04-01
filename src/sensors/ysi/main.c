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
#include <sys/select.h>
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>
#include <libgen.h>
#include <bot_param/param_client.h>

#include "perls-common/timestamp.h"
//#include "perls-common/serial.h"
#include "perls-common/nmea.h"
#include "acfr-common/sensor.h"

#include "perls-lcmtypes/senlcm_ysi_t.h"

//enum {io_socket, io_serial};

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


int program_ysi(acfr_sensor_t *s)
{
	// Set noncanonical mode
	acfr_sensor_noncanonical(s, 1, 0);
	
	// Send an escape and wait for the prompt
	char buf;
	sprintf(&buf, "%c\r\n", 27 );
    acfr_sensor_write(s, &buf, 1); 
    printf("Sent escape character. ");
	
	do
	{
        printf("Waiting for command prompt.");
		acfr_sensor_read(s, &buf, 1);
        printf("Read in character %c.\n", buf);
	}
	while (buf != '#');
	
	// Put it in NMEA mode
    acfr_sensor_write(s, "nmea\r\n", 6); // put it in nmea mode
	
	// Set canonical mode
    acfr_sensor_canonical(s, '\n', '\r');

	return 1;
}

int program_exit;
int broken_pipe;
void
signal_handler(int sig_num)
{
   // do a safe exit
    if(sig_num == SIGPIPE)
        broken_pipe = 1;
    else
        program_exit = 1;
}

int main (int argc, char *argv[]) {
		
    // install the signal handler
    program_exit = 0;
    broken_pipe = 0;
    signal(SIGINT, signal_handler);
    //signal(SIGPIPE, signal_handler);
	
	//Initalise LCM object - specReading
    lcm_t *lcm = lcm_create(NULL);

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));
    
    acfr_sensor_t *sensor = acfr_sensor_create(lcm, rootkey);
    if(sensor == NULL)
        return 0;
 	
	char buf[256];
    senlcm_ysi_t ysi;
   	
	program_ysi(sensor);
	int programmed = 1;

	fd_set rfds;	
    // loop to collect data, parse and send it on its way
    while(!program_exit) {
        // check for broken pipes, if it is broken make sure it is closed and then reopen it
		if(broken_pipe)
		{
            sensor->port_open = 0;
            programmed = 0;
            fprintf(stderr, "Pipe broken\n");
            continue;
        }
    	
		if(!programmed)
        {
            fprintf(stderr, "reprogramming the device\n");
            program_ysi(sensor);
            programmed = 1;
            continue;
        }	

		memset(buf, 0, sizeof(buf));
		
		FD_ZERO(&rfds);
        FD_SET(sensor->fd, &rfds);
	
		struct timeval tv;
		tv.tv_sec = 1;
		tv.tv_usec = 0;
	    
	    int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
        {
			int len;			
            ysi.utime = timestamp_now();
            len = acfr_sensor_read(sensor, buf, 256);
            if(len > 0)
				if(parseYsi(buf, len, &ysi))
					senlcm_ysi_t_publish (lcm, "YSI", &ysi);
					
        }
        else
        {
            // timeout, check the connection
            fprintf(stderr, "Timeout: Checking connection\n");
            //acfr_sensor_write(sensor, "\n", 1);
        }
	}

    acfr_sensor_destroy(sensor);
    
    return 0;
}
