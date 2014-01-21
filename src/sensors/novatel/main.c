/*
 *  Novatel GPS LCM driver
 *  Decodes the ASCII version of the INSPVA message
 *  This is a stop gap driver whilst we still need the old seabed_gui to work.
 *
 *  Christian Lees
 *  ACFR
 *  29/4/12
 */    


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
#include "perls-common/timestamp.h"


#ifndef BOT_CONF_DIR
#define DEFAULT_BOT_CONF_PATH "../config/master.cfg"
#else
#define DEFAULT_BOT_CONF_PATH BOT_CONF_DIR "/master.cfg"
#endif


// copied from Beej's guide to network programming
int recvtimeout(int s, char *buf, int len, int timeout) {
    fd_set fds;
    int n;
    struct timeval tv;
    // set up the file descriptor set
    FD_ZERO(&fds);
    FD_SET(s, &fds);
    // set up the struct timeval for the timeout
    tv.tv_sec = timeout;
    tv.tv_usec = 0;
    // wait until timeout or data received
    
	n = select(s+1, &fds, NULL, NULL, &tv);
	if (n == 0) return -2; // timeout!
	if (n == -1) return -1; // error
	// data must be here, so do a normal recv()
	return recv(s, buf, len, 0);
}

int program_exit;
void signal_handler(int sig_num) 
{
    // do a safe exit
    program_exit = 1;
}

int main(int argc, char *argv[]) 
{
	// install the signal handler
	program_exit = 0;
	signal(SIGINT, signal_handler);

    // read the config file
    BotParam *cfg;
	char rootkey[64];
	char key[64];
	
	char *path = getenv ("BOT_CONF_PATH");
    if (!path) 
        path = DEFAULT_BOT_CONF_PATH;
    cfg = bot_param_new_from_file(path);
    if(cfg == NULL) {
        printf("cound not open config file\n");
        return 0;
    }
    
    sprintf(rootkey, "sensors.%s", basename(argv[0]));
      
    char *gps_ip;
    sprintf(key, "%s.IP", rootkey);
	gps_ip = bot_param_get_str_or_fail(cfg, key);
	
	char *gps_port;
    sprintf(key, "%s.port", rootkey);
	gps_port = bot_param_get_str_or_fail(cfg, key);
	
	// Connect to the GPS
    struct addrinfo hints, *gps_addr;
	int gps_fd;
	
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;

	getaddrinfo(gps_ip, gps_port, &hints, &gps_addr);
	gps_fd = socket(gps_addr->ai_family, gps_addr->ai_socktype, gps_addr->ai_protocol);
 
    int connected = 0;
    char data[256];   
    struct timeval tv;
	fd_set rfds; 
	int ret;

    lcm_t *lcm = lcm_create(NULL);
	
	// main loop
	while(!program_exit) {
        if(!connected)
        {
            tv.tv_sec = 2;
    	    tv.tv_usec = 0;
            FD_ZERO(&rfds);
            FD_SET(gps_fd, &rfds);
            int ret = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
            if(ret != -1)
            {
    	        if(connect(gps_fd, gps_addr->ai_addr, gps_addr->ai_addrlen) < 0) 
	    	    {
                    perror("GPS connect");
                    continue;
                }
                else 
                {
                    printf("GPS: got a connection\n");
                    connected = 1;
                }
            }
            else
            {
                printf("Select timeout\n");
                continue;
            }
        }
        
        senlcm_novatel_t nov;
        char status[128];
        char init_char[64];
        double secs_of_week;
        int gps_week;
        int count;
        
        // now receive and process the GPS messages
        count = 0;
        do
        {
            ret = recvtimeout(gps_fd, &data[count], 1, 1); // 5s timeout
            count += ret;
        } while(data[count - ret] != '\n');
        
        nov.utime = timestamp_now();
        
        
        if(data[0] == '<') 
        {
            count = sscanf(data,"%c %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %s",
     	            init_char, &gps_week, &secs_of_week, &nov.latitude, &nov.longitude, &nov.height,
                        &nov.north_velocity, &nov.east_velocity, &nov.up_velocity, &nov.roll ,&nov.pitch, &nov.heading,
		                status);
		
		    nov.status = status;
            //time_t time_int = secs_of_week + ((gps_week + (10*52)) * GPS_SECS_IN_WEEK);
            //struct tm* tptr=gmtime(&time_int);
            
            if(count!=13)
	            printf("Invalid novatel message, count = %d\n", count);
	        else
	            senlcm_novatel_t_publish(lcm, "NOVATEL", &nov);
        }
    }
}        



