/* ACFR Tunnel Thruster Twin Controller
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
#include "perls-lcmtypes/acfrlcm_auv_nga_motor_command_t.h"

// actual max values supported
//#define TUNNEL_MAX 2047
//#define TUNNEL_MIN -2048

// but with power issues...
// 1024 is really slow, nowhere near the limit
#define TUNNEL_MAX 1500
#define TUNNEL_MIN -1500

#define COMMAND_TIMEOUT_THRUST 10
#define COMMAND_TIMEOUT 10

typedef struct 
{
    lcm_t *lcm;
    acfr_sensor_t *sensor;
//    acfrlcm_tunnel_thruster_command_t tc;
    int64_t command_utime;
    int vert_fore;
    int vert_aft;
    int lat_fore;
    int lat_aft;
    int addrs[4];
    int64_t zero_time;
    int64_t last_zero_time;
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
    	if(chop_string(d, ", ", tokens) == 6)
    	{
	    	acfrlcm_tunnel_thruster_power_t ttp;
	    	ttp.utime = timestamp_now();
	    	ttp.addr = addr;
	    	ttp.voltage[0] = atof(tokens[1]);
	    	ttp.current[0] = atof(tokens[2]);
	    	ttp.voltage[1] = atof(tokens[3]);
	    	ttp.current[1] = atof(tokens[4]);
	    	ttp.temperature = atof(tokens[5]);
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
	
	for(int i=0; i<2; i++)
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
	
	sprintf(msg, "#%02uT1 %d\r", state->addrs[0], state->lat_fore);
	tunnel_write_respond(state, msg, COMMAND_TIMEOUT_THRUST);
	
	sprintf(msg, "#%02uT2 %d\r", state->addrs[0], state->vert_fore);
	tunnel_write_respond(state, msg, COMMAND_TIMEOUT_THRUST);

	// these are the opposite direction to the fore motors
	sprintf(msg, "#%02uT1 %d\r", state->addrs[1], -state->lat_aft);
	tunnel_write_respond(state, msg, COMMAND_TIMEOUT_THRUST);
	
	sprintf(msg, "#%02uT2 %d\r", state->addrs[1], -state->vert_aft);
	tunnel_write_respond(state, msg, COMMAND_TIMEOUT_THRUST);
	
	return 1;
}	

//void tunnel_command_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_tunnel_thruster_command_t *tc, void *u)
void nga_motor_command_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_nga_motor_command_t *mot, void *u)
{
    state_t *state = (state_t *)u;


 // Copy the commands and set the limits
    
    state->command_utime = mot->utime;
    
    if(mot->lat_fore > TUNNEL_MAX)
    	state->lat_fore = TUNNEL_MAX;
    else if (mot->lat_fore < TUNNEL_MIN)
    	state->lat_fore = TUNNEL_MIN;
    else
	state->lat_fore = mot->lat_fore;
		
    if(mot->vert_fore > TUNNEL_MAX)
    	state->vert_fore = TUNNEL_MAX;
    else if (mot->vert_fore < TUNNEL_MIN)
    	state->vert_fore = TUNNEL_MIN;
    else
	state->vert_fore = mot->vert_fore;

    if(mot->lat_aft > TUNNEL_MAX)
    	state->lat_aft = TUNNEL_MAX;
    else if (mot->lat_aft < TUNNEL_MIN)
    	state->lat_aft = TUNNEL_MIN;
    else
	state->lat_aft = mot->lat_aft;
		
    if(mot->vert_aft > TUNNEL_MAX)
    	state->vert_aft = TUNNEL_MAX;
    else if (mot->vert_aft < TUNNEL_MIN)
    	state->vert_aft = TUNNEL_MIN;
    else
        state->vert_aft = mot->vert_aft;

    if(state->vert_aft == 0 && state->vert_fore == 0 && state->lat_aft == 0 && state->lat_fore == 0)
    {
	int64_t time_now = timestamp_now();
	state->zero_time += time_now - state->last_zero_time;
	state->last_zero_time = time_now;
    }
    else
    {
	state->last_zero_time = timestamp_now();
	state->zero_time = 0;
    }

    if(state->zero_time < 10e6)
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
    int num_thrusters = bot_param_get_int_array(state.sensor->param, key, state.addrs, 2);
    if(num_thrusters != 2)
	{
		printf("Wrong number of thruster addresses\n");
		acfr_sensor_destroy(state.sensor);
		return 0;
	}

    // Set canonical mode
    acfr_sensor_canonical(state.sensor, '\r', '\n');
  
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    //acfrlcm_tunnel_thruster_command_t_subscribe(state.lcm, "TUNNEL_THRUSTER_COMMAND", &tunnel_command_handler, &state);
    acfrlcm_auv_nga_motor_command_t_subscribe(state.lcm, "NGA_MOTOR", &nga_motor_command_handler, &state);
    
    while (!program_exit)
    {
    	lcm_handle_timeout(state.lcm, 1000);
    }

    acfr_sensor_destroy(state.sensor);
    return 1;
}

