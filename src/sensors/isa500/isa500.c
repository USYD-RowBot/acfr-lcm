#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

#include <bot_param/param_client.h>

#include "acfr-common/timestamp.h"
#include "acfr-common/nmea.h"
#include "acfr-common/sensor.h"
#include "acfr-common/units.h"
#include "acfr-common/dfs.h"

#include "perls-lcmtypes/senlcm_isa500_t.h"



static int
parse_buf (const char *buf, int len, senlcm_isa500_t *isa)
{
	char msg_type[8];
	nmea_argc(buf, 0, msg_type);
	int ret= 0;
	if(strstr(msg_type, "HPR"))
	{
		// RPH message
		ret += nmea_argf(buf, 1, &isa->heading);
		ret += nmea_argf(buf, 1, &isa->pitch);
		ret += nmea_argf(buf, 1, &isa->roll);
		
		if(ret == 3)
			return 1;
	}
		if(strstr(msg_type, "ADS"))
	{
		// distance message
		ret += nmea_argf(buf, 1, &isa->distance);
		
		if(ret == 1)
			return 1;
	}
	
	return 0;
	
}


void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, char **vehicle_name)
{
    int opt;

    const char *default_name = "DEFAULT";
    *vehicle_name = malloc(strlen(default_name)+1);
    strcpy(*vehicle_name, default_name);
    
    int n;
    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            n = strlen((char *)optarg);
            free(*vehicle_name);
            *vehicle_name = malloc(n);
            strcpy(*vehicle_name, (char *)optarg);
            break;
         }
    }
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

int main (int argc, char *argv[])
{

    // install the signal handler
    program_exit = 0;
    broken_pipe = 0;
    signal(SIGINT, signal_handler);
    //signal(SIGPIPE, signal_handler);

    //Initalise LCM object - specReading
    lcm_t *lcm = lcm_create(NULL);

    char *vehicle_name;
    char channel_name[200];
    parse_args(argc, argv, &vehicle_name);

    snprintf(channel_name, sizeof(channel_name), "%s.ISA500", vehicle_name);

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    acfr_sensor_t *sensor = acfr_sensor_create(lcm, rootkey);
    if(sensor == NULL)
        return 0;

    acfr_sensor_canonical(sensor, '\n', '\r');

    fd_set rfds;
    char buf[256];
    senlcm_isa500_t isa500;
    
    // loop to collect data, parse and send it on its way
    while(!program_exit)
    {
        // check for broken pipes, if it is broken make sure it is closed and then reopen it
        if(broken_pipe)
        {
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
            isa500.utime = timestamp_now();
            len = acfr_sensor_read(sensor, buf, 256);
            if(len > 0)
                if (parse_buf(buf, len, &isa500))
			senlcm_isa500_t_publish (lcm, channel_name, &isa500);

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
