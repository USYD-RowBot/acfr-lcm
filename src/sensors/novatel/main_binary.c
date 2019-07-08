#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <pthread.h>
#include <math.h>
#include <libgen.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <bot_param/param_client.h>


#include "perls-lcmtypes/senlcm_novatel_t.h"
#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"

#include "novatel.h"

#define DTOR M_PI/180
#define LEAP_SECONDS 16

static char *novatel_status[] =
{
    "INS Inactive\0",
    "INS Aligning\0",
    "INS Solution Bad\0",
    "INS Solution Good\0",
    "Reserved\0",
    "Reserved\0",
    "Bad INS/GPS Agreement\0",
    "INS Alignment Complete\0",
    "Undefined"
};

static char *bestpos_status[] = 
{
	"Solution computed\0",
	"Insufficient observations\0",
	"No convergence\0",
	"Singularity at parameters matrix\0",
 	"Covariance trace exceeds maximum (trace > 1000 m)\0",
 	"Test distance exceeded (maximum of 3 rejections if distance >10 km)\0",
 	"Not yet converged from cold start\0",
 	"Height or velocity limits exceeded\0",
 	"Variance exceeds limits\0",
 	"Residuals are too large Large residuals make position unreliable\0",
 	"Other\0" 	
};


typedef struct
{
	lcm_t *lcm;
	acfr_sensor_t *sensor;

    double lat_std;
    double lon_std;
    double gps_sog;
    double gps_tog;
    double gps_vz;
    double gps_heading;
    double gps_pitch;
    double gps_heading_std;
    double gps_pitch_std;
    double heading_offset;
    bool flip_roll_pitch;
    int64_t timestamp;
    int64_t gps_time;
    bool have_imu;
    
} state_t;

int program_novatel(state_t *state, int rate, char *com_port)
{
    char msg[256];

    printf("Using com port %s\n", com_port);
    sprintf(msg, "unlogall %s\r\n", com_port);
    acfr_sensor_write(state->sensor, msg, strlen(msg));

	if(state->have_imu)
	{
    	sprintf(msg, "log inspvab ontime %f\r\n", 1.0/(double)rate);
    	acfr_sensor_write(state->sensor, msg, strlen(msg));
	}
	else
	{
		sprintf(msg, "log heading2b onnew \r\n");
    	acfr_sensor_write(state->sensor, msg, strlen(msg));
		sprintf(msg, "log bestvelb ontime %f\r\n", 1.0/(double)rate);
    	acfr_sensor_write(state->sensor, msg, strlen(msg));		
	}
	
    sprintf(msg, "log bestposb ontime %f\r\n", 1.0/(double)rate);
    acfr_sensor_write(state->sensor, msg, strlen(msg));

    return 1;
}

int parse_inspvab(state_t *state, char *d, char *channel_name)
{
    // Reference: Novatel Span OEM Firmware Manual 5.2.23 INSPVA p.147
    senlcm_novatel_t nov;
    memset(&nov, 0, sizeof(senlcm_novatel_t));
    nov.utime = state->timestamp;
    nov.gps_time = state->gps_time;
    nov.latitude = *(double *)&d[12] * DTOR;
    nov.longitude = *(double *)&d[20] * DTOR;
    nov.roll = *(double *)&d[60] * DTOR;
    nov.pitch = *(double *)&d[68] * DTOR;
    nov.heading = *(double *)&d[76] * DTOR;
    nov.height = *(double *)&d[28];
    nov.north_velocity = *(double *)&d[36];
    nov.east_velocity = *(double *)&d[44];
    nov.up_velocity = *(double *)&d[52];
    //int status = *(int *)&d[84];
    //printf("Status: %d\n", status);
    char *ns = malloc(strlen(novatel_status[*(int *)&d[84]]) + 1);
    memset(ns, 0, strlen(novatel_status[*(int *)&d[84]]) + 1);
    strncpy(ns, novatel_status[*(int *)&d[84]], strlen(novatel_status[*(int *)&d[84]]));
    nov.status = ns; //novatel_status[*(int *)&d[84]];
    nov.latitude_sd = state->lat_std;
    nov.longitude_sd = state->lon_std;

    senlcm_novatel_t_publish(state->lcm, channel_name, &nov);

    return 1;
}

// We only extract the standard deviations out of this message
int parse_bestposb(state_t *state, char *d, char *channel_name)
{
    state->lat_std = *(float *)&d[40];
    state->lon_std = *(float *)&d[44];
    
    if(!state->have_imu)
    {
		senlcm_novatel_t nov;
		memset(&nov, 0, sizeof(senlcm_novatel_t));
		nov.utime = state->timestamp;
		nov.gps_time = state->gps_time;
		nov.latitude = *(double *)&d[8] * DTOR;
		nov.longitude = *(double *)&d[16] * DTOR;
		if(state->flip_roll_pitch)
		{	
		    nov.roll = 0;
		    nov.pitch = state->gps_pitch * DTOR;
		}
	        else
		{	
		    nov.pitch = 0;
		    nov.roll = state->gps_pitch * DTOR;
		}
		nov.heading = fmod((state->gps_heading + state->heading_offset), 360) * DTOR;
		nov.height = *(double *)&d[24];
		nov.north_velocity = cos(state->gps_tog*DTOR)*state->gps_sog;
		nov.east_velocity = sin(state->gps_tog*DTOR)*state->gps_sog;
		nov.up_velocity = state->gps_vz;
		int status = *(int *)&d[0];
		if(status  > 10)
			status = 10;
		//printf("Status %d\n", status);
		char *ns = malloc(strlen(bestpos_status[status]) + 1);
		memset(ns, 0, strlen(bestpos_status[status]) + 1);
		strncpy(ns, bestpos_status[status], strlen(bestpos_status[status]));
		nov.status = ns; 
		nov.latitude_sd = state->lat_std;
		nov.longitude_sd = state->lon_std;

		senlcm_novatel_t_publish(state->lcm, "NOVATEL", &nov);
	}

    return 1;
}

int parse_bestvelb(state_t *state, char *d)
{
	state->gps_sog = *(double *)&d[16];
	state->gps_tog = *(double *)&d[24];
	state->gps_vz = *(double *)&d[32];

	return 1;
}

int parse_heading2b(state_t *state, char *d)
{
	state->gps_heading = *(float *)&d[12];
	state->gps_pitch = *(float *)&d[16];
	state->gps_heading_std = *(float *)&d[24];
	state->gps_pitch_std = *(float *)&d[28];

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

void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, char **channel_name)
{
    int opt;

    const char *default_name = "DEFAULT";
    *channel_name = malloc(strlen(default_name)+1);
    strcpy(*channel_name, default_name);
    
    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            free(*channel_name);
            *channel_name = malloc(200);
            snprintf(*channel_name, 200, "%s.NOVATEL", (char *)optarg);
            break;
         }
    }
}

int
main (int argc, char *argv[])
{
    // install the signal handler
    program_exit = 0;
    broken_pipe = 0;
    signal(SIGINT, signal_handler);
    signal(SIGPIPE, signal_handler);

	state_t state;
    //Initalise LCM object - specReading
    state.lcm = lcm_create(NULL);

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    char *channel_name;
    parse_args(argc, argv, &channel_name);

    state.sensor = acfr_sensor_create(state.lcm, rootkey);
    if(state.sensor == NULL)
        return 0;

    acfr_sensor_canonical(state.sensor, '\r', '\n');

    // read some additional config
    char key[64];
    sprintf(key, "%s.rate", rootkey);
    int rate = bot_param_get_int_or_fail(state.sensor->param, key);

    sprintf(key, "%s.com_port", rootkey);
    char *com_port = bot_param_get_str_or_fail(state.sensor->param, key);

	sprintf(key, "%s.have_imu", rootkey);
    state.have_imu = bot_param_get_boolean_or_fail(state.sensor->param, key);


    sprintf(key, "%s.flip_roll_pitch", rootkey);
    if(bot_param_has_key(state.sensor->param, key))
	state.flip_roll_pitch = bot_param_get_boolean_or_fail(state.sensor->param, key);
    else
	state.flip_roll_pitch = false;

    sprintf(key, "%s.heading_offset", rootkey);
    if(bot_param_has_key(state.sensor->param, key))
	state.heading_offset = bot_param_get_double_or_fail(state.sensor->param, key);
    else
	state.heading_offset = 0.0;

    program_novatel(&state, rate, com_port);

    acfr_sensor_noncanonical(state.sensor, 1, 0);

    fd_set rfds;
    char buf[512];
    int64_t timestamp;


    while(!program_exit)
    {
        // check for broken pipes, if it is broken make sure it is closed and then reopen it
        if(broken_pipe)
            state.sensor->port_open = 0;

        memset(buf, 0, sizeof(buf));

        FD_ZERO(&rfds);
        FD_SET(state.sensor->fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
        {
            timestamp = timestamp_now();

            // read a byte and check to see if it is a sync bytes
            do
            {
                acfr_sensor_read(state.sensor, &buf[0], 1);
            }
            while((buf[0] & 0xFF) != 0xAA);

            // read another two bytes and check
            acfr_sensor_read(state.sensor, &buf[1], 2);
            if(buf[1] == 0x44 && buf[2] == 0x12)
            {
                // we have a valid header, get one more byte to get the header length
                acfr_sensor_read(state.sensor, &buf[3], 1);
                unsigned char header_length = (unsigned char)buf[3];

                // read the rest of the header
                acfr_sensor_read(state.sensor, &buf[4], header_length - 4);

                // get the message type
                unsigned short message_id = *(unsigned short *)&buf[4];
                unsigned short message_length = *(unsigned short *)&buf[8];
                unsigned short gps_week = *(unsigned short *)&buf[14];
                unsigned int gps_msec = *(unsigned int *)&buf[16];

                // work out the GPS time
                int64_t gps_time = ((((int64_t)gps_week + 1024) * 604800) * 1000000) + ((int64_t)gps_msec * 1000) + (int64_t)(315964800000000) - LEAP_SECONDS * 1000000;

                // read the rest of the message including the checksum
                acfr_sensor_read(state.sensor, &buf[header_length], message_length + 4);

                // Check the CRC
                unsigned long crc = CalculateBlockCRC32(header_length + message_length, (unsigned char *)buf);
                if(crc != (*(unsigned long *)&buf[header_length + message_length]))
                    printf("CRC error: 0x%08lX, actual: 0x%08lX\n", crc, *(unsigned long *)&buf[header_length + message_length]);
                else
                {
                	state.timestamp = timestamp;
                	state.gps_time = gps_time;
                	switch(message_id)
                	{
                    	case 507:
                    		parse_inspvab(&state, &buf[header_length], channel_name);
							break;
                    	case 42:
                        	parse_bestposb(&state, &buf[header_length], channel_name);
                        	break;
                    	case 99:
                        	parse_bestvelb(&state, &buf[header_length]);
							break;
						case 1335:
                        	parse_heading2b(&state, &buf[header_length]);
							break;
					}
                }
            }
        }
        else
            fprintf(stderr, "Select timeout\n");
    }

    char msg[256];
    sprintf(msg, "unlogall %s\r\n", com_port);
    acfr_sensor_write(state.sensor, msg, strlen(msg));
    acfr_sensor_destroy(state.sensor);

    return 0;

}

