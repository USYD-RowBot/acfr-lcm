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
#include <bot_param/param_client.h>

#include "evologics.h"

#include "perls-common/serial.h"
#include "perls-common/units.h"
#include "perls-common/timestamp.h"

#include "perls-lcmtypes/perllcm_heartbeat_t.h"



#define MAX_BUF_LEN 1024
/*
void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;

}
*/      
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
	state.term = '\n';
    
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
      
    // read the initial config
    sprintf(key, "%s.device", rootkey);
    char *device = bot_param_get_str_or_fail(param, key);

	sprintf(key, "%s.baud", rootkey);
    int baud = bot_param_get_int_or_fail(param, key);

    sprintf(key, "%s.parity", rootkey);
    char *parity = bot_param_get_str_or_fail(param, key);  
      
        
    // Open the serial port
    state.fd = serial_open(device, serial_translate_speed(baud), serial_translate_parity(parity), 1);    
    serial_set_canonical(state.fd, '\r', '\n');
    
    int lcm_fd = lcm_get_fileno(state.lcm);    
    
//    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    acfrlcm_auv_status_t_subscribe(state.lcm, "AUV_STATUS", &status_handler, &state);

    fd_set rfds;
    char buf[MAX_BUF_LEN];
    int64_t timestamp;
    int len;
    int ping_counter = 0;
    state.channel_ready = 1;
    
    // put the modem in a known state
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
                len = read(state.fd, buf, MAX_BUF_LEN);
                timestamp = timestamp_now();
                
                // parsing the meaasge will also set all the channel control flags
                parse_evologics_message(buf, len, &state, timestamp);
            }
        }    
        
        
    }
    
    // lets exit
    close(state.fd);
    
    return 0;
}
