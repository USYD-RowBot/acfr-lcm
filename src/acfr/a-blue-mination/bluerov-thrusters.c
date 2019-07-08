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
#include <math.h>
#include <sys/ioctl.h>
#include <bot_param/param_client.h>
#include "acfr-common/sensor.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/acfrlcm_auv_abluemination_t.h"
#include "perls-lcmtypes/acfrlcm_tunnel_thruster_power_t.h"
#include "perls-lcmtypes/acfrlcm_auv_nga_motor_command_t.h"

#define MAX_DSHOT_VALUE 1000
#define COMMAND_TIMEOUT_THRUST 50
#define COMMAND_TIMEOUT 50

typedef struct 
{
    lcm_t *lcm;
    acfr_sensor_t *sensor;
    int64_t command_utime;
    int max_rpm;
    int16_t stb_bottom;
    int16_t stb_middle;
    int16_t stb_top;
    int16_t port_bottom;
    int16_t port_middle;
    int16_t port_top;
    int8_t pt_cw;
    int8_t pm_cw;
    int8_t pb_cw;
    int8_t st_cw;
    int8_t sm_cw;
    int8_t sb_cw;
    int64_t zero_time;
    int64_t last_zero_time;
} state_t;

int RS485_write(acfr_sensor_t *sensor, char *d)
{
    int ret = acfr_sensor_write(sensor, d, strlen(d));
    return ret;
}

int parse_thruster_response(state_t *state, char *d, int len)
{
    // ESC board sends back telemetry with format b'[*<values>\r]
    if(d[0] == '*')
    {
	    acfrlcm_tunnel_thruster_power_t ttp;
	    ttp.utime = timestamp_now();
	    ttp.addr = 1;
	    ttp.voltage[0] = (double)(*((int32_t *)&d[1]))/1000;
	    ttp.current[0] = (double)(*((int32_t *)&d[5]))/100000;
	    ttp.current[1] = 0.0;
	    ttp.voltage[1] = 0.0;
	    ttp.temperature = (double)(*((int32_t *)&d[9]))/1000;
	    acfrlcm_tunnel_thruster_power_t_publish(state->lcm, "NGA.ABLUEMINATION_POWER", &ttp);
	    return 1;
    }
    return 0;
}

// Send a command and wait for a response and parse
int tunnel_write_respond(state_t *state, char *d, int timeout)
{
    int ret =  RS485_write(state->sensor, d);
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

int send_tunnel_commands(state_t *state);
void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    // make telemetry request message #?\r
    char msg[16];
    msg[0] = '#';
    msg[1] = '?';
    msg[2] = '\r';
    //tunnel write respond makes the I2C hang and bad things happen
//	tunnel_write_respond(state, msg, 10);
//	printf("hb time=%d , cmd time=%d, diff = %d", hb->utime, state->command_utime, (hb->utime-state->command_utime)/10000); 

	if((hb->utime - state->command_utime)/10000 < COMMAND_TIMEOUT)
	{  
		send_tunnel_commands(state);
	}
	else
	{
		state->port_top = 0.0;
		state->port_middle = 0.0;
		state->port_bottom = 0.0;
		state->stb_top = 0.0;
		state->stb_middle = 0.0;
		state->stb_bottom = 0.0;
		send_tunnel_commands(state);
	}	
}

int send_tunnel_commands(state_t *state)
{
	// Send the thrust values to the controllers format b'[#!<values>\r]
	char msg[64];
	msg[0] = '#';
    msg[1] = '!';
	memcpy(&msg[2], &state->stb_bottom, 12);
	msg[14] = '\r';
//	printf("%d, %d, %d, %d, %d, %d\r", state->stb_bottom, state->stb_middle, state->stb_top, state->port_bottom, state->port_middle, state->port_top);
	acfr_sensor_write(state->sensor, msg, 15);
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
	acfr_sensor_write(state->sensor, msg, 15);	
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
    rudder_percentage *= state->invert_rudder;
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
    elevator_percentage *= state->invert_elevator;
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
    sprintf(key, "%s.invert_rudder", rootkey);
    state.sb_cw = bot_param_get_boolean_or_fail(state.sensor->param, key);
    sprintf(key, "%s.invert_elevator", rootkey);
    state.sb_cw = bot_param_get_boolean_or_fail(state.sensor->param, key);
    sprintf(key, "%s.max_rpm", rootkey);
    state.max_rpm = bot_param_get_int_or_fail(state.sensor->param, key);

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
    if (state.invert_rudder == 0)
        state.invert_rudder = -1;
    if (state.invert_elevator == 0)
        state.invert_elevator = -1;

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

