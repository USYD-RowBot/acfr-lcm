/*
 *  Novatel GPS LCM driver
 *  Decodes the ASCII version of the INSPVA message
 *  This is a stop gap driver whilst we still need the old seabed_gui to work.
 *
 *  Christian Lees
 *  ACFR
 *  29/4/12
 */    

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
#include "perls-common/timestamp.h"


#ifndef BOT_CONF_DIR
#define DEFAULT_BOT_CONF_PATH "../config/master.cfg"
#else
#define DEFAULT_BOT_CONF_PATH BOT_CONF_DIR "/master.cfg"
#endif

#define DTOR M_PI/180

int readline(int fd, char *buf, int max_len)
{
    int i=0;
    do
    {
        if(recv(fd, &buf[i++], 1, 0) == -1)
		{
			printf("BREAK\n");
            break;
		}
    } while((buf[i-1] != '\n'));
    return i;
}

int chop_string(char *data, char **tokens)
{
    char *token;
    int i = 0;
    
    token = strtok(data, " ,:");
    while(token != NULL) 
    {
        tokens[i++] = token;            
        token = strtok(NULL, " ,:");            
    }
    return i;        
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
    BotParam *param;
	char rootkey[64];
	char key[64];

    lcm_t *lcm = lcm_create(NULL);
    param = bot_param_new_from_server (lcm, 1);
	    
    sprintf(rootkey, "sensors.%s", basename(argv[0]));
      
    char *gps_ip;
    sprintf(key, "%s.IP", rootkey);
	gps_ip = bot_param_get_str_or_fail(param, key);
	
	char *gps_port;
    sprintf(key, "%s.port", rootkey);
	gps_port = bot_param_get_str_or_fail(param, key);
	
	// Connect to the GPS
    struct addrinfo hints, *gps_addr;
	int gps_fd;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	getaddrinfo(gps_ip, gps_port, &hints, &gps_addr);
	gps_fd = socket(gps_addr->ai_family, gps_addr->ai_socktype, gps_addr->ai_protocol);
 
	struct timeval tv;

    int connected = 0;
    char data[256];   
    //struct timeval tv;
	fd_set rfds; 
	int ret;
	
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
				    tv.tv_sec = 1;  // 1 Secs Timeout 
    				tv.tv_usec = 1000;  // Not init'ing this can cause strange errors
				    setsockopt(gps_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

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
        
        // now receive and process the GPS messages

		memset(data, 0, sizeof(data));
  		readline(gps_fd, data, 256);      
        nov.utime = timestamp_now();
		
		char *tok[64];
		ret = chop_string(data, tok);
	    if(ret > 1)
	    {
	        if((tok[0][0] == '<') && (ret == 13))
	        {
	            nov.latitude = atof(tok[3]) * DTOR;
			    nov.longitude = atof(tok[4]) * DTOR;
			    nov.roll = atof(tok[9]) * DTOR;
			    nov.pitch = atof(tok[10]) * DTOR;
			    nov.heading = atof(tok[11]) * DTOR;
			    nov.height = atof(tok[5]);
			    nov.north_velocity = atof(tok[6]);
			    nov.east_velocity = atof(tok[7]);
			    nov.up_velocity = atof(tok[8]);
//			    nov.time = atof(tok[]);
			    
	            
                senlcm_novatel_t_publish(lcm, "NOVATEL", &nov);
            }
        }

    }
}        



