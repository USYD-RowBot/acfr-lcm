#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <signal.h>
#include <libgen.h>
#include <byteswap.h>

#include "acfr-common/timestamp.h"
#include "perls-lcmtypes/senlcm_xsens_t.h"
#include "acfr-common/sensor.h"
#include "acfr-common/units.h"
#include <lcm/lcm.h>

#define XSENS_COMMAND_SIZE 128
#define XSENS_READ_SIZE 256

#define update_rate 0.1

float tofloat(char *d)
{
	uint32_t n = bswap_32(*d);
	return *((float *)&n);
}	

int parse_xsens(lcm_t *lcm, char *channel_name, char *d, int len, int64_t timestamp)
{
	char *dp;
	if(d[3] == 0xFF)
		dp = &d[6];
	else
		dp = &d[4];
		
    senlcm_xsens_t xs;
    xs.utime = timestamp;
    xs.temp = tofloat(&dp[0]);
    xs.acc_x = tofloat(&dp[4]);
    xs.acc_y = tofloat(&dp[8]);
    xs.acc_z = tofloat(&dp[12]);
    xs.gyr_x = tofloat(&dp[16]);
    xs.gyr_y = tofloat(&dp[20]);
    xs.gyr_z = tofloat(&dp[24]);
    xs.mag_x = tofloat(&dp[28]);
    xs.mag_y = tofloat(&dp[32]);
    xs.mag_z = tofloat(&dp[36]);
    xs.roll = tofloat(&dp[40]);
    xs.pitch = tofloat(&dp[44]);
    xs.heading = tofloat(&dp[48]);
      
    senlcm_xsens_t_publish(lcm, channel_name, &xs);    

    return 1;

}

int xsens_form_message(char *in, int data_len, char mid, char *out)
{
	memset(out, 0, XSENS_COMMAND_SIZE);
    out[0] = 0xFA;
    out[1] = 0xFF;
    out[2] = mid & 0xFF;
    
    //Data size
    if(data_len < 254)
    {
    	out[4] = data_len & 0xFF;
    	memcpy(&out[5], in, data_len);
	}
	else
	{	// Extended data mode
		out[4] = 0xFF;
		out[5] = (data_len & 0xFF00) >> 8;
		out[6] = data_len & 0xFF;
		memcpy(&out[7], in, data_len);
	}
    
    // Header Length
    int header_len;
    if(data_len > 254)
    	header_len = 6;
	else 
		header_len = 4;
    
    for(int i=header_len; i<data_len+header_len; i++)
    	out[header_len + data_len] += out[i];

    return header_len + data_len + 1;
}

int xsens_read(acfr_sensor_t *s, char *in)
{
	int len = acfr_sensor_read_timeoutms(s, in, 4, 500);
	if(len < 4)
	{
		fprintf(stderr, "Failed to get data back from Xsens\n");
		return 0;
	}
	// read the rest of the data 
	unsigned int data_size = in[3];
	// Extended data mode
	if(data_size == 0xFF)
	{	
		len += acfr_sensor_read_timeoutms(s, &in[len], 2, 500);
		data_size = in[5] & 0xFF;
		data_size += (in[4] & 0xFF) << 8;
	}
	
	// Read the data and the checksum
	len += acfr_sensor_read_timeoutms(s, &in[len], data_size+1, 500);
	
	// Check the checksum
	unsigned char cs = 0;
	for(int i=1; i<len; i++)
		cs += in[i];
		
	if(cs != in[len-1])
	{
		fprintf(stderr, "Checksum error\n");
		return -1;
	}	

	return len;
}

int xsens_send_command(acfr_sensor_t *s, char *in, int data_len, char command)
{
	char out[XSENS_COMMAND_SIZE];
	
	int len;
	len = xsens_form_message(in, data_len, command, out);
	acfr_sensor_write(s, out, len);

	return xsens_read(s, in);	
}

int program_xsens(acfr_sensor_t *s)
{
    char data[128];
 	
	// Xsens wakeup
	xsens_send_command(s, data, 0, 0x3E);
 
 	// Go to config
    xsens_send_command(s, data, 0, 0x30);
    
    // Set output mode, Temp, Calibrated data, Orientation
    data[0] = 0x08;
    data[1] = 0x07;
    xsens_send_command(s, data, 2, 0xD0);
    
    // Output settings
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x0C;
    data[3] = 0x05;
    xsens_send_command(s, data, 4, 0xD2);
    
    // Set output period, 100Hz
    data[0] = 0x04;
    data[1] = 0x80;
    xsens_send_command(s, data, 2, 0x04);
    
    // Go to measurement mode
    xsens_send_command(s, data, 0, 0x10);

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
    else if (sig_num == SIGTERM)
        broken_pipe = 1;
    else if(sig_num == SIGINT)
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

int
main (int argc, char *argv[])
{

    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);

    char *vehicle_name;
    parse_args(argc, argv, &vehicle_name);

    char xsens_channel[128];

    snprintf(xsens_channel, 128, "%s.XSENS", vehicle_name);

    free(vehicle_name);

    //Initalise LCM object - specReading
    lcm_t *lcm = lcm_create(NULL);

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    acfr_sensor_t *sensor = acfr_sensor_create(lcm, rootkey);
    if(sensor == NULL)
        return 0;

    acfr_sensor_noncanonical(sensor, 1, 0);

    int len;
    char buf[XSENS_READ_SIZE];

    int64_t timestamp;

    program_xsens(sensor);

    fd_set rfds;

    while(!program_exit)
    {
 
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
            timestamp = timestamp_now();
            len = xsens_read(sensor, buf);
            if(len > 1)
            	parse_xsens(lcm, xsens_channel, buf, len, timestamp);
            else
                printf("Bad data_len\n");
        }
        
    }

    acfr_sensor_destroy(sensor);
    lcm_destroy(lcm);

    return 0;

}
