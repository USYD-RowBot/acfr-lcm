/* ACFR PSU driver
 * 1/8 brick 
 *
 * Christian Lees
 * ACFR
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <libgen.h>
#include <sys/ioctl.h>
#include <bot_param/param_client.h>
#include "acfr-common/sensor.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/senlcm_acfr_psu_t.h"

#define MAX_PSUS 8

typedef struct 
{
    lcm_t *lcm;
    acfr_sensor_t *sensor;
    int num_psu;
    int psu_addrs[MAX_PSUS];
} state_t;

#define RS485_SEALEVEL

int set_rts(int fd, int level)
{
    int status;
    if (ioctl(fd, TIOCMGET, &status) == -1)
    {
        perror("setRTS(): TIOCMGET");
        return 0;
    }
    if (level)
        status |= TIOCM_RTS;
    else
        status &= ~TIOCM_RTS;
    if (ioctl(fd, TIOCMSET, &status) == -1)
    {
        perror("setRTS(): TIOCMSET");
        return 0;
    }
    return 1;
}

int RS485_write(acfr_sensor_t *sensor, char *d)
{
#ifdef RS485_SEALEVEL
    // set the RTS pin to enable sending
//	set_rts(sensor->fd, 1);
#endif

    int ret = acfr_sensor_write(sensor, d, strlen(d));

#ifdef RS485_SEALEVEL
    // clear the RTS pin to enable receiving
//    set_rts(sensor->fd, 0);
#endif

    return ret;
}

   
int chop_string(char *data, char *delim, char **tokens)
{
    char *token;
    int i = 0;

    token = strtok(data, delim);
    while(token != NULL)
    {
        tokens[i++] = token;
        token = strtok(NULL, delim);
    }
    return i;
}

void parse_psu_response(state_t *state, char *d, int len)
{
	char addr_str[3] = {0};
    char *cmd_char;
    int addr;
    char *tokens[8];
    
    memcpy(addr_str, &d[1], 2);
    addr = atoi(addr_str);
    cmd_char = &d[3];
    
    if(*cmd_char == 'S' || *cmd_char == 's')
    {
    	if(chop_string( d, ", ", tokens) == 4)
    	{
	    	senlcm_acfr_psu_t psu;
	    	psu.utime = timestamp_now();
	    	psu.address = addr;
	    	psu.voltage = atof(tokens[1]);
	    	psu.current = atof(tokens[2]);
	    	psu.temperature = atof(tokens[3]);
	    	senlcm_acfr_psu_t_publish(state->lcm, "PSU", &psu);
    	}
    }
}
    	 

void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    char msg[16];
    char resp[32];
    int ret;
	
	
	for(int i=0; i<state->num_psu; i++)
	{
		memset(msg, 0, sizeof(msg));
		sprintf(msg, "#%02dS\r", state->psu_addrs[i]);
		RS485_write(state->sensor, msg);
		
		// Wait for a response with a timeout
		memset(resp, 0, sizeof(resp));
		ret = acfr_sensor_read_timeoutms(state->sensor, resp, sizeof(resp), 100);
		if(ret > 0)
			parse_psu_response(state, resp, ret);
	}
        
        
		    
}


int program_exit;
void
signal_handler(int sig_num)
{
    // do a safe exit
    program_exit = 1;
}

int main (int argc, char *argv[])
{

    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);

    state_t state;
    
    //Initalise LCM object
    state.lcm = lcm_create(NULL);
    

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    state.sensor = acfr_sensor_create(state.lcm, rootkey);
    if(state.sensor == NULL)
        return 0;

	// Read the PSU addresses we are interested in
	char key[64];
	sprintf(key, "%s.addrs", rootkey);
	state.num_psu = bot_param_get_int_array(state.sensor->param, key, state.psu_addrs, MAX_PSUS);
	
    // Set canonical mode
    acfr_sensor_canonical(state.sensor, '\r', '\n');
  
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);

    
    while (!program_exit)
    {
    	lcm_handle_timeout(state.lcm, 1000);
    }

    acfr_sensor_destroy(state.sensor);
    return 1;
}

