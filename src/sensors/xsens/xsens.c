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

float be_float(unsigned char *d)
{
	

	unsigned int n = *((unsigned int *)d);
	uint32_t n2 = bswap_32(n);
	return *((float *)&n2);
}	

unsigned short be_short(unsigned char *d)
{
	return (unsigned short)((d[0] << 8) + d[1]);
}


int parse_xsens(lcm_t *lcm, char *channel_name, unsigned char *d, int len, int64_t timestamp, senlcm_xsens_t *xs)
{
	unsigned char *dp;
	if(d[3] == 0xFF)
		dp = &d[6];
	else
		dp = &d[4];

	unsigned char sub_len, data[64];
	unsigned short id;
            xs->utime = timestamp;
	while((dp - d) < len)
	{
	    id = (*dp << 8) + *(dp + 1);
 	    sub_len = *(dp + 2);
	    memcpy(data, dp + 3, sub_len);
	    dp += 3 + sub_len;

//	    printf("ID: 0x%04X, length: %d\n", id & 0xFFFF, sub_len);
//	    for(int i=0; i<sub_len; i++)
//		printf("%02X ", data[i] & 0xFF);
//	    printf("\n");

            switch (id & 0xF800)
            {
	        case 0x0800:	//Temperature
		    if((id & 0x00F0) == 0x0010)
			xs->temp = be_float(data);
		    break;
	        case 0x1000:	// Timestamp
		    if((id & 0x00F0) == 0x0020)
			xs->count = (int)be_short(data);
		    break;
	        case 0x2000:	// Orientation
		    if((id & 0x00F0) == 0x0030)
		    {
			xs->roll = be_float(data);
			xs->pitch = be_float(data+4);
			xs->heading = be_float(data+8);
		    }
		    break;
	        case 0x4000:	// Acceleration
		    if((id & 0x00F0) == 0x0020)
		    {
			xs->acc_x = be_float(data);
			xs->acc_y = be_float(data+4);
			xs->acc_z = be_float(data+8);
		    }
		    break;			
	        case 0x8000:	// Angular vel
		    if((id & 0x00F0) == 0x0020)
		    {
			xs->gyr_x = be_float(data);
			xs->gyr_y = be_float(data+4);
			xs->gyr_z = be_float(data+8);
		    }
		    break;			
	        case 0xC000:	// Magnetic
		    if((id & 0x00F0) == 0x0020)
		    {
			xs->mag_x = be_float(data);
			xs->mag_y = be_float(data+4);
			xs->mag_z = be_float(data+8);
		    }
		    break;			
	        //default:
		//    printf("Unwanted or unknown packet\n");
		//    break;
            }

	}
      
        senlcm_xsens_t_publish(lcm, channel_name, xs);    

    return 1;

}

int xsens_form_message(unsigned char *in, int data_len, unsigned char mid, unsigned char *out)
{
	memset(out, 0, XSENS_COMMAND_SIZE);
    out[0] = 0xFA;
    out[1] = 0xFF;
    out[2] = mid & 0xFF;
    
    //Data size
    if(data_len < 254)
    {
    	out[3] = data_len & 0xFF;
    	memcpy(&out[4], in, data_len);
	}
	else
	{	// Extended data mode
		out[3] = 0xFF;
		out[4] = (data_len & 0xFF00) >> 8;
		out[5] = data_len & 0xFF;
		memcpy(&out[6], in, data_len);
	}
    
    // Header Length
    int header_len;
    if(data_len > 254)
    	header_len = 6;
	else 
		header_len = 4;

    unsigned char cs = 0 ; 
    for(int i=1; i<data_len+header_len; i++)
	cs += out[i];    	

    out[header_len + data_len] += 0xFF - cs + 1;

    return header_len + data_len + 1;
}

int xsens_read(acfr_sensor_t *s, unsigned char *in)
{
	int len;
	in[0] = 0;
	do
	{
	    len = acfr_sensor_read_timeoutms(s, (char *)&in[0], 1, 500);
	    if(in[0] == 0xFA)
	        len += acfr_sensor_read_timeoutms(s, (char *)&in[1], 1, 500);
	    //printf("0x%02X, ", in[0] & 0xFF);
	} while(in[0] != 0xFA && in[1] == 0xFF);
	//printf("\n");
	len += acfr_sensor_read_timeoutms(s, (char *)&in[len], 2, 500);
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
		len += acfr_sensor_read_timeoutms(s, (char *)&in[len], 2, 500);
		data_size = in[5] & 0xFF;
		data_size += (in[4] & 0xFF) << 8;
	}
	
	// Read the data and the checksum
	len += acfr_sensor_read_timeoutms(s, (char *)&in[len], data_size+1, 500);
	
	// Check the checksum
	unsigned char cs = 0;
	for(int i=1; i<len-1; i++)
		cs += in[i];
/*
	printf("Data Recv: ");
	for(int i = 0;i<len; i++)
		printf("%02X ", in[i] & 0xFF);
	printf("\n");
*/
	if((0xFF - cs + 1) != in[len-1])
	{
		fprintf(stderr, "Checksum error\n");
		return -1;
	}	

	return len;
}

int xsens_send_command(acfr_sensor_t *s, unsigned char *in, int data_len, unsigned char command)
{
	unsigned char out[XSENS_COMMAND_SIZE];
	
	int len;
	len = xsens_form_message(in, data_len, command, out);
	acfr_sensor_write(s, (char *)out, len);
/*
	printf("Data Send: ");
	for(int i = 0;i<len; i++)
		printf("%02X ", out[i] & 0xFF);
	printf("\n");
*/	
	do {
	    xsens_read(s, in);
	} while (in[2] != command + 1);

	return 1;	
}


int program_xsens(acfr_sensor_t *s)
{
    unsigned char data[128];
 	

    tcflush(s->fd, TCIOFLUSH); 
//    xsens_send_command(s, data, 0, 0x40);
//printf("Reset\n");
 	// Go to config
//usleep(5e6);
    xsens_send_command(s, data, 0, 0x30);
printf("Entered config\n");
	unsigned char xsense_program[] = 
		{	0x08, 0x10, 0x00, 0x01,		// Temp, 1Hz
			0x10, 0x20, 0x07, 0xD0, 	// Packet count
			0x20, 0x30, 0x00, 0x64,		// Euler angles, 100Hz
			0x40, 0x20, 0x00, 0x64, 	// Acceleration, 100Hz
			0x80, 0x20, 0x00, 0x64, 	// Angular velocity, 100Hz
			0xC0, 0x20, 0x00, 0x64  	// Mag field, 100Hz			 
		};
	xsens_send_command(s, xsense_program, sizeof(xsense_program), 0xC0);
printf("Programmed\n");
    // Go to measurement mode
    xsens_send_command(s, data, 0, 0x10);
printf("Entered measure\n");
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
    unsigned char buf[XSENS_READ_SIZE];

    int64_t timestamp;
    program_xsens(sensor);

    fd_set rfds;
    senlcm_xsens_t xs;

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
            	parse_xsens(lcm, xsens_channel, buf, len, timestamp, &xs);
            else
                fprintf(stderr, "Bad data_len\n");
        }
        
    }

    acfr_sensor_destroy(sensor);
    lcm_destroy(lcm);

    return 0;

}
