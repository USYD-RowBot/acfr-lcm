/* ACFR bluerov Thruster Controller
 *
 * Christian Reeks
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
#include "perls-lcmtypes/acfrlcm_auv_abluemination_t.h"
#include "perls-lcmtypes/acfrlcm_tunnel_thruster_power_t.h"
#include "perls-lcmtypes/acfrlcm_auv_nga_motor_command_t.h"

// actual max values supported
//#define TUNNEL_MAX 2047
//#define TUNNEL_MIN -2048

// but with power issues...
// 1024 is really slow, nowhere near the limit
#define TUNNEL_MAX 1750
#define TUNNEL_MIN -1750
#define MAX_DSHOT_VALUE 1000
#define COMMAND_TIMEOUT_THRUST 50
#define COMMAND_TIMEOUT 50

typedef struct 
{
    lcm_t *lcm;
    acfr_sensor_t *sensor;
    int64_t command_utime;
    int max_rpm;
    int port_top;
    int port_middle;
    int port_bottom;
    int stb_top;
    int stb_middle;
    int stb_bottom;
    int8_t pt_cw;
    int8_t pm_cw;
    int8_t pb_cw;
    int8_t st_cw;
    int8_t sm_cw;
    int8_t sb_cw;
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
    char *cmd_char;
    char *tokens[8];
    
    cmd_char = &d[3];
    
    if(*cmd_char == 'S' || *cmd_char == 's')
    {
    	if(chop_string(d, ", ", tokens) == 4)
    	{
	    	acfrlcm_tunnel_thruster_power_t ttp;
	    	ttp.utime = timestamp_now();
	    	ttp.addr = 1;
	    	ttp.voltage[0] = atof(tokens[1]);
	    	ttp.current[0] = atof(tokens[2]);
	    	ttp.temperature = atof(tokens[3]);
		char channel[100];
		snprintf(channel, 100, "NGA.ABLUEMINATION_POWER");
	    	acfrlcm_tunnel_thruster_power_t_publish(state->lcm, channel, &ttp);
	    	
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
	memset(msg, 0, sizeof(msg));
	sprintf(msg, "#01S\r");
	tunnel_write_respond(state, msg, COMMAND_TIMEOUT);	    
}

int send_tunnel_commands(state_t *state)
{
	// Send the thrust values to the controllers
	char msg[64];
	sprintf(msg, "#%02uT %d, %d, %d, %d, %d, %d\r", 01, state->port_top, state->port_middle, state->port_bottom, state->stb_top, state->stb_middle, state->stb_bottom);
	tunnel_write_respond(state, msg, COMMAND_TIMEOUT_THRUST);
	
	return 1;
}	

void nga_motor_command_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_nga_motor_command_t *mot, void *u)
{
    state_t *state = (state_t *)u;

    acfrlcm_auv_nga_motor_command_t mc;

    memcpy(&mc, mot, sizeof(acfrlcm_auv_nga_motor_command_t));
    
    state->command_utime = mot->utime;
    //Get a thrust value in percent form that does not exceed 100%
    double thrust_max_value = 32.5/100.0;
    double thrust_percentage = mot->tail_thruster/state->max_rpm;
    if (fabs(thrust_percentage) > 1.0)
        thrust_percentage = thrust_percentage/fabs(thrust_percentage);
    // Rudder percentage values
    double rudder_max_value = 40.0/100.0;
    double rudder_min_value = 22.5/100.0;
    double rudder_port = 0.0;
    double rudder_stb = 0.0;
    double rudder_percentage = mot->tail_rudder/(12*M_PI/180);
    if (fabs(rudder_percentage) > 1.0)
        rudder_percentage = rudder_percentage/fabs(rudder_percentage);

    if (rudder_percentage < 0.0){
        rudder_port = thrust_max_value-fabs(rudder_percentage)*(thrust_max_value-rudder_min_value);
        rudder_stb = thrust_max_value+fabs(rudder_percentage)*(rudder_max_value-thrust_max_value);
    }
    else if (rudder_percentage > 0.0){
        rudder_port = thrust_max_value+fabs(rudder_percentage)*(rudder_max_value-thrust_max_value);
        rudder_stb = thrust_max_value-fabs(rudder_percentage)*(thrust_max_value-rudder_min_value);
    }
    else {
        rudder_port = thrust_max_value;
        rudder_stb = thrust_max_value;
    }
    // Elevator percentage values
    double elevator_max_value = 37.5/100.0;
    double elevator_min_value = 25.0/100.0;
    double elevator_top = 0.0;
    double elevator_mid = 0.0;
    double elevator_btm = 0.0;
    double elevator_percentage = mot->tail_elevator/(12*M_PI/180);
    if (fabs(elevator_percentage) > 1.0)
        elevator_percentage = elevator_percentage/fabs(elevator_percentage);

    if (elevator_percentage < 0.0){
        elevator_top = thrust_max_value+fabs(elevator_percentage)*(elevator_max_value-thrust_max_value);
        elevator_mid = thrust_max_value;
        elevator_btm = thrust_max_value-fabs(elevator_percentage)*(thrust_max_value-elevator_min_value);
    }
    else if (elevator_percentage > 0.0){
        elevator_top = thrust_max_value-fabs(elevator_percentage)*(thrust_max_value-elevator_min_value);
        elevator_mid = thrust_max_value;
        elevator_btm = thrust_max_value+fabs(elevator_percentage)*(elevator_max_value-thrust_max_value);
    }
    else{
        elevator_top = thrust_max_value;
        elevator_mid = thrust_max_value;
        elevator_btm = thrust_max_value;
    }
    //naive reconciliation through averaging
    state->port_top     =   state->pt_cw*((rudder_port+  elevator_top)/2)*thrust_percentage*MAX_DSHOT_VALUE;
    state->port_middle  =   state->pm_cw*((rudder_port+  elevator_mid)/2)*thrust_percentage*MAX_DSHOT_VALUE;
    state->port_bottom  =   state->pb_cw*((rudder_port+  elevator_btm)/2)*thrust_percentage*MAX_DSHOT_VALUE;
    state->stb_top      =   state->st_cw*((rudder_stb+   elevator_top)/2)*thrust_percentage*MAX_DSHOT_VALUE;
    state->stb_middle   =   state->sm_cw*((rudder_stb+   elevator_mid)/2)*thrust_percentage*MAX_DSHOT_VALUE;
    state->stb_bottom   =   state->sb_cw*((rudder_stb+   elevator_btm)/2)*thrust_percentage*MAX_DSHOT_VALUE;

    acfrlcm_auv_abluemination_t bluey;
    bluey.utime = timestamp_now();
    bluey.port_top = state->port_top;
    bluey.port_middle = state->port_middle;
    bluey.port_bottom = state->port_bottom;
    bluey.stb_top = state->stb_top;
    bluey.stb_middle = state->stb_middle;
    bluey.stb_bottom = state->stb_bottom; 
    char channel[100];
    snprintf(channel, 100, "NGA.ABLUEMINATION_THRUST");
    acfrlcm_auv_abluemination_t_publish(state->lcm, channel, &bluey);

    if(state->port_top == 0 && state->port_middle == 0 && state->port_bottom == 0 && state->stb_top == 0 && state->stb_middle == 0 && state->stb_bottom == 0)
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
    {
        static int sent_last = 0;
    	if (sent_last == 1)
    	{
            send_tunnel_commands(state);
    	    sent_last = 0;
    	}
            else
    	{
    	    sent_last = 1;
    	}
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
    sprintf(rootkey, "acfr.%s", basename(argv[0]));

    state.sensor = acfr_sensor_create(state.lcm, rootkey);
    if(state.sensor == NULL)
        return 0;
        
    // Read the PSU addresses we are interested in
     char key[64];
 //    sprintf(key, "%s.addrs", rootkey);
 //    int num_thrusters = bot_param_get_int_array(state.sensor->param, key, state.addrs, 2);
 //    if(num_thrusters != 2)
	// {
	// 	printf("Wrong number of thruster addresses\n");
	// 	acfr_sensor_destroy(state.sensor);
	// 	return 0;
	// }

    sprintf(key, "%s.pt_cw", rootkey);
    state.pt_cw = bot_param_get_boolean_or_fail(state.sensor->param, key);
    sprintf(key, "%s.pm_cw", rootkey);
    state.pm_cw = bot_param_get_boolean_or_fail(state.sensor->param, key);
    sprintf(key, "%s.pb_cw", rootkey);
    state.pb_cw = bot_param_get_boolean_or_fail(state.sensor->param, key);
    sprintf(key, "%s.st_cw", rootkey);
    state.st_cw = bot_param_get_boolean_or_fail(state.sensor->param, key);
    sprintf(key, "%s.sm_cw", rootkey);
    state.sm_cw = bot_param_get_boolean_or_fail(state.sensor->param, key);
    sprintf(key, "%s.sb_cw", rootkey);
    state.sb_cw = bot_param_get_boolean_or_fail(state.sensor->param, key);

    state.max_rpm = bot_param_get_int_or_fail(state.sensor->param, "acfr.bluefin-tail.max_rpm");

    if (state.pt_cw == 0)
        state.pt_cw = -1;
    if (state.pm_cw == 0)
        state.pm_cw = -1;
    if (state.pb_cw == 0)
        state.pb_cw = -1;
    if (state.st_cw == 0)
        state.st_cw = -1;
    if (state.sm_cw == 0)
        state.sm_cw = -1;
    if (state.sb_cw == 0)
        state.sb_cw = -1;

    // Set canonical mode
    acfr_sensor_canonical(state.sensor, '\r', '\n');
  
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    acfrlcm_auv_nga_motor_command_t_subscribe(state.lcm, "NGA.NEXTGEN_MOTOR", &nga_motor_command_handler, &state);
    
    while (!program_exit)
    {
    	lcm_handle_timeout(state.lcm, 1000);
    }

    acfr_sensor_destroy(state.sensor);
    return 1;
}

