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

// read a line in a canonical way on a socket
int readline(int fd, char *d, int len)
{
	int bytes = 0;
	int term = 0;
	while(bytes < len)
	{
		bytes += read(fd, &d[bytes], 1);
		if(d[bytes-1] == '\n')
		{
			term = 1;
			break;
		}
	}
	if(term)
		return bytes;
	else
		return -1;
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
    
    acfr_sensor_t sensor;
    acfr_sensor_load_config(lcm, &sensor, rootkey);
    acfr_sensor_open(&sensor);
/*                
    // Read the LCM config file
    BotParam *param;
	
	char key[64];
	
    param = bot_param_new_from_server (lcm, 1);
    
    

    // read the config file
    int io;
	sprintf(key, "%s.io", rootkey);
	char *io_str = bot_param_get_str_or_fail(param, key);
    if(!strcmp(io_str, "serial"))
        io = io_serial;
    else if(!strcmp(io_str, "socket"))
        io = io_socket;
    
    char *serial_dev, *parity;
    int baud;
    char *ip, *inet_port;
    
    if(io == io_serial)
    {
        sprintf(key, "%s.serial_dev", rootkey);
        serial_dev = bot_param_get_str_or_fail(param, key);

    	sprintf(key, "%s.baud", rootkey);
	    baud = bot_param_get_int_or_fail(param, key);

	    sprintf(key, "%s.parity", rootkey);
	    parity = bot_param_get_str_or_fail(param, key);
    }
    
    if(io == io_socket)
    {
        sprintf(key, "%s.ip", rootkey);
        ip = bot_param_get_str_or_fail(param, key);

        sprintf(key, "%s.port", rootkey);
        inet_port = bot_param_get_str_or_fail(param, key);
    }

    // Open either the serial port or the socket
    struct addrinfo hints, *spec_addr;
    int fd;
    if(io == io_serial)
    {
        fd = serial_open(serial_dev, serial_translate_speed(baud), serial_translate_parity(parity), 1);
        if(fd < 0)
        {
            printf("Error opening port %s\n", serial_dev);
            return 0;
        }
    }        
    else if(io == io_socket)
    {
        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        getaddrinfo(ip, inet_port, &hints, &spec_addr);
    	fd = socket(spec_addr->ai_family, spec_addr->ai_socktype, spec_addr->ai_protocol);
        if(connect(fd, spec_addr->ai_addr, spec_addr->ai_addrlen) < 0) 
        {
	        printf("Could not connect to %s on port %s\n", ip, inet_port);
    		return 1;
        }
    
    }
*/   	
	char buf[256];
    senlcm_ysi_t ysi;
   	
	// Put the YSI in a known state, send and escape character and then wait for the prompt
	// then send a nmea command
    
	sprintf( buf, "%c", 27 );
    write(sensor.fd, buf, 1); 
	
	do
	{
		read(sensor.fd, buf, 1);
	}
	while (buf[0] != '#');
	
    write(sensor.fd, "nmea\r\n", 6); // put it in nmea mode
	
	fd_set rfds;	
    // loop to collect data, parse and send it on its way
    while(!program_exit) {
        // check for broken pipes, if it is broken make sure it is closed and then reopen it
        if(broken_pipe)
            if(!sensor.port_open)
                acfr_sensor_open(&sensor);
    
    
        memset(buf, 0, sizeof(buf));
		
		FD_ZERO(&rfds);
        FD_SET(sensor.fd, &rfds);
	
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
			if(sensor.io_type == io_serial)
				len = read(sensor.fd, buf, 256);
			else
				len = readline(sensor.fd, buf, 256);
            if(len > 0)
				if(parseYsi(buf, len, &ysi))
					senlcm_ysi_t_publish (lcm, "YSI", &ysi);
					
        }
	}

    return 0;
}
