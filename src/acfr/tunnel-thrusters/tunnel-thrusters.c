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
#include "perls-lcmtypes/acfrlcm_tunnel_thruster_command_t.h"
#include "perls-lcmtypes/acfrlcm_tunnel_thruster_power_t.h"


#define TUNNEL_MAX 2047
#define TUNNEL_MIN -2048

#define COMMAND_TIMEOUT_THRUST 500
#define COMMAND_TIMEOUT 10

typedef struct 
{
    lcm_t *lcm;
    acfr_sensor_t *sensor;
    acfrlcm_tunnel_thruster_command_t tc;
    int addrs[4];
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

int parse_thruster_response(state_t *state, char *d, int len)
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
    	if(chop_string(d, ", ", tokens) == 5)
    	{
	    	acfrlcm_tunnel_thruster_power_t ttp;
	    	ttp.utime = timestamp_now();
	    	ttp.addr = addr;
	    	ttp.voltage = atof(tokens[2]);
	    	ttp.current = atof(tokens[3]);
	    	ttp.temperature = atof(tokens[4]);
	    	acfrlcm_tunnel_thruster_power_t_publish(state->lcm, "TUNNEL_THRUSTER_POWER", &ttp);
	    	
	    	return 1;
    	}
    }
    return 0;
}

// Send a command and wait for a response and parse
int tunnel_write_respond(state_t *state, char *d, int timeout)
{
    printf("****Sending data: %s\n", d);
    int ret = RS485_write(state->sensor, d);
    
    char buf[64];
    memset(buf, 0, sizeof(buf));
    int bytes;
    if(ret > 0)
    {
        bytes =  acfr_sensor_read_timeoutms(state->sensor, buf, sizeof(buf), timeout);
        printf("****Got %d bytes, response %s\n", bytes, buf);
        if(bytes > 0)
            return parse_thruster_response(state, buf, bytes);
        else
            printf("RX timeout\n");
    }
    else
        return -2;
        
    return 0;                
}

    	 

void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    char msg[16];
	
	for(int i=0; i<4; i++)
	{
		memset(msg, 0, sizeof(msg));
		sprintf(msg, "#%02dS\r", state->addrs[i]);
		tunnel_write_respond(state, msg, COMMAND_TIMEOUT);
		
	}
        
        
		    
}

int send_tunnel_commands(state_t *state)
{
	// Send the thrust values to the controllers
	char msg[16];
	
	sprintf(msg, "#%02uT%d\r", state->addrs[0], state->tc.fore_horiz);
	tunnel_write_respond(state, msg, COMMAND_TIMEOUT_THRUST);
	
	sprintf(msg, "#%02uT%d\r", state->addrs[1], state->tc.fore_vert);
	tunnel_write_respond(state, msg, COMMAND_TIMEOUT_THRUST);

	sprintf(msg, "#%02uT%d\r", state->addrs[2], state->tc.aft_horiz);
	tunnel_write_respond(state, msg, COMMAND_TIMEOUT_THRUST);
	
	sprintf(msg, "#%02uT%d\r", state->addrs[3], state->tc.aft_vert);
	tunnel_write_respond(state, msg, COMMAND_TIMEOUT_THRUST);
	
	return 1;
}	

void tunnel_command_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_tunnel_thruster_command_t *tc, void *u)
{
    state_t *state = (state_t *)u;


 // Copy the commands and set the limits
    
    state->tc.utime = tc->utime;
    
    if(tc->fore_horiz > TUNNEL_MAX)
    	state->tc.fore_horiz = TUNNEL_MAX;
	else if (tc->fore_horiz < TUNNEL_MIN)
    	state->tc.fore_horiz = TUNNEL_MIN;
	else
		state->tc.fore_horiz = tc->fore_horiz;
		
    if(tc->fore_vert > TUNNEL_MAX)
    	state->tc.fore_vert = TUNNEL_MAX;
	else if (tc->fore_vert < TUNNEL_MIN)
    	state->tc.fore_vert = TUNNEL_MIN;
	else
		state->tc.fore_vert = tc->fore_vert;

	if(tc->aft_horiz > TUNNEL_MAX)
    	state->tc.aft_horiz = TUNNEL_MAX;
	else if (tc->aft_horiz < TUNNEL_MIN)
    	state->tc.aft_horiz = TUNNEL_MIN;
	else
		state->tc.aft_horiz = tc->aft_horiz;
		
    if(tc->aft_vert > TUNNEL_MAX)
    	state->tc.aft_vert = TUNNEL_MAX;
	else if (tc->aft_vert < TUNNEL_MIN)
    	state->tc.aft_vert = TUNNEL_MIN;
	else
		state->tc.aft_vert = tc->aft_vert;

    send_tunnel_commands(state);
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
    sprintf(rootkey, "acfr.%s", basename(argv[0]));

    state.sensor = acfr_sensor_create(state.lcm, rootkey);
    if(state.sensor == NULL)
        return 0;
        
	// Read the PSU addresses we are interested in
	char key[64];
	sprintf(key, "%s.addrs", rootkey);
	int num_thrusters = bot_param_get_int_array(state.sensor->param, key, state.addrs, 4);
	if(num_thrusters != 4)
	{
		printf("Wrong number of thruster addresses\n");
		acfr_sensor_destroy(state.sensor);
		return 0;
	}

    // Set canonical mode
    acfr_sensor_canonical(state.sensor, '\r', '\n');
  
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    acfrlcm_tunnel_thruster_command_t_subscribe(state.lcm, "TUNNEL_THRUSTER_COMMAND", &tunnel_command_handler, &state);
    
    while (!program_exit)
    {
    	lcm_handle_timeout(state.lcm, 1000);
    }

    acfr_sensor_destroy(state.sensor);
    return 1;
}

