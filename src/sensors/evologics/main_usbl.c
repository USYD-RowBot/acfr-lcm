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

}
       
void novatel_handler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_novatel_t *nv, void *u)
{
    state_t *state = (state_t *)u;
    memcpy(state->novatel, nv, sizeof(senlcm_novatel_t));

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
	state.term = '\n';
	state.novatel = malloc(sizeof(senlcm_novatel_t));
    
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
        
    char *inet_port;
    char *ip;
    
    sprintf(key, "%s.ip", rootkey);
    ip = bot_param_get_str_or_fail(param, key);

    sprintf(key, "%s.port", rootkey);
    inet_port = bot_param_get_str_or_fail(param, key);
    
    // Ping information
    sprintf(key, "%s.targets", rootkey);
    state.num_targets = bot_param_get_int_array(param, key, state.targets, 8);
    for(int i=0; i<state.num_targets; i++)
        state.ping_sem[i] = 1;
        
    sprintf(key, "%s.attitude_source", rootkey);
    char *att_source_str = bot_param_get_str_or_fail(param, key);
    if(!strcmp(att_source_str, "novatel"))
        state.attitude_source = NOVATEL;
    else if(!strcmp(att_source_str, "internal"))
        state.attitude_source = INTERNAL;
    else
    {
        fprintf(stderr, "Invalid attitude source\n");
        return 0;
    }   
    
    int lcm_fd = lcm_get_fileno(state.lcm);    
    
    state.ping_counter = 0;
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    senlcm_novatel_t_subscribe(state.lcm, "NOVATEL", &novatel_handler, &state);
    
    
    // open the ports
    struct addrinfo hints, *evo_addr;
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
    tv.tv_sec = 0;  // 1 Secs Timeout 
    tv.tv_usec = 1000;  // Not init'ing this can cause strange errors
    setsockopt(state.fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
    

    fd_set rfds;
    char buf[MAX_BUF_LEN];
    int64_t timestamp;
    int len;
    int ping_counter = 0;
    state.channel_ready = 1;
    
    // put the USBL in a known state
    send_evologics_command("+++ATZ1\n", NULL, 256, &state);
    
    while(!program_exit) 
    {
        FD_ZERO(&rfds);
        FD_SET(state.fd, &rfds);
        FD_SET(lcm_fd, &rfds);
	
		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 50000;
	    
	    int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        
        if(ret != 0)
        {
           if(FD_ISSET(lcm_fd, &rfds))
                lcm_handle(state.lcm);
            else
            {
                // data to be read       
                memset(buf, 0, MAX_BUF_LEN);
                len = readline(state.fd, buf, MAX_BUF_LEN);
                timestamp = timestamp_now();
                
                // parsing the meaasge will also set all the channel control flags
                parse_evologics_message(buf, len, &state, timestamp);
            }
        }    
        
        if(state.channel_ready)
        {
            // send a ping
            send_ping(ping_counter++, &state);
            if(ping_counter == state.num_targets)
                ping_counter = 0;
        }
    }
    
    // lets exit
    close(state.fd);
    
    return 0;
}
