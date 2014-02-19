#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <signal.h>
#include <libgen.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netdb.h>
#include <bot_param/param_client.h>

#include "evologics.h"

#include "perls-common/serial.h"
#include "perls-common/units.h"
#include "perls-common/timestamp.h"

#include "perls-lcmtypes/perllcm_heartbeat_t.h"



#define MAX_BUF_LEN 1024

void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;

    if(state->ping_counter == state->ping_period)
    {
        state->ping_counter = 0;
        send_ping(2, state);
    }
    else
        state->ping_counter++;
}
        
void status_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_status_t *as, void *u)
{
    printf("assembling a status message\n");
    state_t *state = (state_t *)u;
    char *channel = {"STATUS"};
    int data_size = sizeof(acfrlcm_auv_status_t) + strlen(channel) + 2;
    char *data = malloc(data_size);
    printf("assembling a status message, data size %d\n", data_size);
    data[0] = LCM_AUV_STATUS;
    data[1] = strlen(channel);
    memcpy(&data[2], channel, strlen(channel));
    memcpy(&data[strlen(channel) + 2], as, sizeof(acfrlcm_auv_status_t));
    send_evologics_data(data, data_size, 2, state);
    free(data);
}
        


int program_exit;
void 
signal_handler(int sigNum) 
{
    // do a safe exit
    program_exit = 1;
}

int main (int argc, char *argv[]) {
		
	state_t state;
    
    // install the signal handler
	program_exit = 0;
    signal(SIGINT, signal_handler);   
    
    
    // read the config file
    BotParam *param;
	char rootkey[64];
	char key[64];
	
    state.lcm = lcm_create(NULL);
    param = bot_param_new_from_server (state.lcm, 1);
    
    sprintf(rootkey, "sensors.%s", basename(argv[0]));
    
    // read the config file
    char *io_str;
	sprintf(key, "%s.io", rootkey);
	io_str = bot_param_get_str_or_fail(param, key);
    if(!strcmp(io_str, "serial"))
        state.io = io_serial;
    else if(!strcmp(io_str, "socket"))
        state.io = io_socket;
    
    char *serial_dev;
    char *inet_port;
    char *ip;
    int baud;
    char *parity;
    
    if(state.io == io_serial)
    {
        sprintf(key, "%s.serial_dev", rootkey);
        serial_dev = bot_param_get_str_or_fail(param, key);

    	sprintf(key, "%s.baud", rootkey);
	    baud = bot_param_get_int_or_fail(param, key);

	    sprintf(key, "%s.parity", rootkey);
	    parity = bot_param_get_str_or_fail(param, key);
    }
    
    if(state.io == io_socket)
    {
        sprintf(key, "%s.ip", rootkey);
        ip = bot_param_get_str_or_fail(param, key);

        sprintf(key, "%s.port", rootkey);
        inet_port = bot_param_get_str_or_fail(param, key);
    }
    
    sprintf(key, "%s.ping_period", rootkey);
    state.ping_period = bot_param_get_int_or_fail(param, key);

    
    
    int lcm_fd = lcm_get_fileno(state.lcm);    
    
    state.ping_counter = 0;
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    acfrlcm_auv_status_t_subscribe(state.lcm, "AUV_STATUS", &status_handler, &state);
    
    // open the ports
    struct addrinfo hints, *evo_addr;
    if(state.io == io_serial)
    {
        state.fd = serial_open(serial_dev, serial_translate_speed(baud), serial_translate_parity(parity), 1);
        if(state.fd < 0)
        {
            printf("Error opening port %s\n", serial_dev);
            return 0;
        }
        serial_set_canonical(state.fd, '\r', '\n');
    }        
    else if(state.io == io_socket)
    {
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        getaddrinfo(ip, inet_port, &hints, &evo_addr);
    	state.fd = socket(evo_addr->ai_family, evo_addr->ai_socktype, evo_addr->ai_protocol);
        if(connect(state.fd, evo_addr->ai_addr, evo_addr->ai_addrlen) < 0) 
        {
	        printf("Could not connect to %s on port %s\n", ip, inet_port);
    		return 1;
        }
        
        struct timeval tv;
        tv.tv_sec = 2;  // 1 Secs Timeout 
        tv.tv_usec = 0;  // Not init'ing this can cause strange errors
        setsockopt(state.fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
    
    }

    fd_set rfds;
    char buf[MAX_BUF_LEN];
    int64_t timestamp;
    int len;
       
    // loop to collect data, parse and send it on its way
    while(!program_exit) 
    {
        FD_ZERO(&rfds);
        FD_SET(lcm_fd, &rfds);
        FD_SET(state.fd, &rfds);
	
		struct timeval tv;
		tv.tv_sec = 1;
		tv.tv_usec = 0;
	    
	    int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
        {
            if(FD_ISSET(lcm_fd, &rfds))
                lcm_handle(state.lcm);
            else
            {
                memset(buf, 0, MAX_BUF_LEN);
                if(state.io == io_socket)                    
                    len = readline(state.fd, buf, MAX_BUF_LEN);
                else
                    len = read(state.fd, buf, MAX_BUF_LEN);   
                timestamp = timestamp_now();
            
                parse_evologics_message(buf, len, &state, timestamp);
            }
	    }
    }
    
    // lets exit
    close(state.fd);
    
    return 0;
}
