/*	Seapath320 Binary Format 3 to ship status
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
#include "perls-common/nmea.h"
#include "acfr-common/sensor.h"

#include "perls-lcmtypes/acfrlcm_ship_status_t.h"

static float read_swapped_float(char *buf) {
    char swapped[4];

    swapped[0] = buf[3];
    swapped[1] = buf[2];
    swapped[2] = buf[1];
    swapped[3] = buf[0];

    return *(float *)swapped;
}

// 2^31 == 1.0, twos complement
// multiply the output by 180 or pi to get deg or radians
// we use doubles here for the extra precision
static double fixed_double(uint32_t original) {

    uint64_t exponent = 1023;
    uint64_t sign = 0;

    double output;
    uint64_t *output_bits = (uint64_t *)&output;

    // check the sign bit
    // convert to positive if necessary - makes it simpler
    // to not have to deal with twos complement for the conversion
    if (original & 0x80000000) {
        sign = 1;
        original = -original;
    }

    uint64_t significand = original;

    int i;

    // get the exponent and significand
    // basically shift until the top place bit is a 1
    for (i=31;i>=0;--i) {
        if ((1 << 31) & significand) {
            // we have found where we want to be...
            // put all in place
            (*output_bits) = ((sign << 63) & 0x8000000000000000) + ((exponent << 52) & 0x7FF0000000000000) + ((significand << 21) & 0x000FFFFFFFFFFFFF);
            break;
        } else {
            significand <<= 1;
            exponent -= 1;
        }
    }

    return output;
}

static int parseShipStatus(char *buf, int buf_len, acfrlcm_ship_status_t *status) {
    int valid = 0;

    // quick check that this packet matches expected fields
    if (buf[0] != 'q' || buf[1] != 49)
      return 0;

    //status->north_velocity = read_swapped_float(buf + 23);
    //status->east_velocity = read_swapped_float(buf + 27);
    //status->down_velocity = read_swapped_float(buf + 31);

    status->roll = read_swapped_float(buf + 35);
    status->pitch = read_swapped_float(buf + 39);
    status->heading = read_swapped_float(buf + 43);

    int32_t raw_lat = ntohl(*(uint32_t *)(buf + 7));
    int32_t raw_lon = ntohl(*(uint32_t *)(buf + 11));

    status->latitude = 180.0 * fixed_double(raw_lat);
    status->longitude = 180.0 * fixed_double(raw_lon);

    status->ship_id = (int8_t)1;
    status->name = (char *)"FALKOR";

    uint8_t sum = 0;
    int i = 0;
    for (i = 2;i<51;++i) {
        sum += buf[i];
    }

    if (sum == buf[51]) {
        valid = 1;
    }

    return valid;
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
    signal(SIGPIPE, signal_handler);
	
	//Initalise LCM object - specReading
    lcm_t *lcm = lcm_create(NULL);

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));
    
    acfr_sensor_t *sensor = acfr_sensor_create(lcm, rootkey);
    if(sensor == NULL)
        return 0;

    char key[128];
    sprintf(key, "%s.channel", rootkey);
    char *channel = bot_param_get_str_or_fail(sensor->param, key);
    sprintf(key, "%s.ship_name", rootkey);
    char *ship_name = bot_param_get_str_or_fail(sensor->param, key);
    sprintf(key, "%s.ship_id", rootkey);
    int8_t ship_id = bot_param_get_int_or_fail(sensor->param, key);
 	
    char buf[52];
    acfrlcm_ship_status_t status;

    status.name = ship_name;
    status.ship_id = ship_id;


    fd_set rfds;
    // loop to collect data, parse and send it on its way
    while(!program_exit) {
        // check for broken pipes, if it is broken make sure it is closed and then reopen it
	if(broken_pipe) {
            sensor->port_open = 0;
            fprintf(stderr, "Pipe broken\n");
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
            status.utime = timestamp_now();
            len = acfr_sensor_read(sensor, buf, 52);
            if(len == 52) {
	        if (parseShipStatus(buf, len, &status))
                    acfrlcm_ship_status_t_publish (lcm, channel, &status);
            } else {
              fprintf(stderr, "Message incorrect size!");
            }
					
        }
        else
        {
            // timeout, check the connection
            fprintf(stderr, "Timeout: Checking connection\n");
        }
    }

    acfr_sensor_destroy(sensor);
    
    return 0;
}
