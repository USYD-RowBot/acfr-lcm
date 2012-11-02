/*
    Listens to the motor command message and sends it onto the main thruster
    and the servo controller.  Easliy expanded to support cannards.
    
    Christian Lees
    ACFR
    2/11/12
*/

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <libgen.h>

#include "perls-common/lcm_util.h"
#include "perls-common/serial.h"
#include "perls-common/timestamp.h"
#include <bot_param/param_client.h>
#include "perls-lcmtypes/acfrlcm_auv_iver_motor_command_t.h"

#define SERVO_RANGE 0

// gears are servo 19T, intermediat 40T, fin shaft 36T
// gear ratio is 0.5277
// servo range is 90 degrees
#define SERVO_GEAR_RATIO (1/0.5277)
#define MAX_FIN_ANGLE 20

#define REMOTE_TIMEOUT 2000000

typedef struct 
{
    // serial port stuff
    int servo_fd;
    int motor_fd;
    
    // lcm stuff param = bot_param_new_from_server (lcm, 1);
    lcm_t *lcm;
    
    int64_t remote_time;
    int remote;
    
} state_t;    

void
motor_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_iver_motor_command_t *mc, void *u) 
{
    state_t *state = (state_t *)u;
    
    // scale the motor commands from actual fin angles to value to send to the motors
    char rudder_top, rudder_bottom;
    char plane_port, plane_starboard;
    
    // we got a remote command, set the time and mode
    if(mc->source == ACFRLCM_AUV_IVER_MOTOR_COMMAND_T_REMOTE)
    {
        state->remote_time = mc->utime;
        state->remote = 1;
    }

    // check the remote time out and set remote to off if required    
    if((timestamp_now() - state->remote_time) > REMOTE_TIMEOUT)
        state->remote = 0;
    
    // if we got a auto cammand but we are still in remote mode the return
    if(mc->source == ACFRLCM_AUV_IVER_MOTOR_COMMAND_T_AUTO || state->remote)
        return;
    
    
    
    // limit check and scale
    if(mc->top > MAX_FIN_ANGLE)
        rudder_top = (char)(MAX_FIN_ANGLE * SERVO_GEAR_RATIO);
    else if(mc->top < -MAX_FIN_ANGLE)
        rudder_top = (char)(-MAX_FIN_ANGLE * SERVO_GEAR_RATIO);
    else
        rudder_top = (char)(mc->top * SERVO_GEAR_RATIO);
        
    if(mc->bottom > MAX_FIN_ANGLE)
        rudder_bottom = (char)(MAX_FIN_ANGLE * SERVO_GEAR_RATIO);
    else if(mc->bottom < -MAX_FIN_ANGLE)
        rudder_bottom = (char)(-MAX_FIN_ANGLE * SERVO_GEAR_RATIO);
    else
        rudder_bottom = (char)(mc->bottom * SERVO_GEAR_RATIO);
    
    if(mc->port > MAX_FIN_ANGLE)
        plane_port = (char)(MAX_FIN_ANGLE * SERVO_GEAR_RATIO);
    else if(mc->port < -MAX_FIN_ANGLE)
        plane_port = (char)(-MAX_FIN_ANGLE * SERVO_GEAR_RATIO);
    else
        plane_port = (char)(mc->port * SERVO_GEAR_RATIO);
    
    if(mc->starboard > MAX_FIN_ANGLE)
        plane_starboard = (char)(MAX_FIN_ANGLE * SERVO_GEAR_RATIO);
    else if(mc->starboard < -MAX_FIN_ANGLE)
        plane_starboard = (char)(-MAX_FIN_ANGLE * SERVO_GEAR_RATIO);
    else
        plane_starboard = (char)(mc->starboard * SERVO_GEAR_RATIO);
    
    
    
    char servo_command[3];
    servo_command[0] = 0xFF;
    
    servo_command[1] = 0x00 + SERVO_RANGE;
    servo_command[2] = rudder_top;
    write(state->servo_fd, servo_command, 3);
    
    servo_command[1] = 0x01 + SERVO_RANGE;
    servo_command[2] = rudder_bottom;
    write(state->servo_fd, servo_command, 3);
    
    servo_command[1] = 0x02 + SERVO_RANGE;
    servo_command[2] = plane_port;
    write(state->servo_fd, servo_command, 3);
    
    servo_command[1] = 0x03 + SERVO_RANGE;
    servo_command[2] = plane_starboard;
    write(state->servo_fd, servo_command, 3);

    // send the command to the thruster motor
    char motor_string[64];
    sprintf(motor_string, "MV a=0 A=1000 V=%d G\n", (int)(mc->main * 32212 / 60));
    write(state->motor_fd, motor_string, strlen(motor_string));
}

int program_exit;
void signal_handler(int sigNum) 
{
    // do a safe exit
    program_exit = 1;
}

int 
main(int argc, char **argv)
{
    state_t state;
    state.remote = 0;
 
    // install the signal handler
	program_exit = 0;
    signal(SIGINT, signal_handler);   
    
   	// lets start LCM
	state.lcm = lcm_create(NULL);
    	
    
    // read the config file
	char rootkey[64];
    char key[128];    
    BotParam *param = bot_param_new_from_server (state.lcm, 1);
    if(param == NULL)
        return 0;
    sprintf(rootkey, "acfr.%s", basename(argv[0]));

	
	// servo port info
	sprintf(key, "%s.servo_device", rootkey);
	char *servo_device = bot_param_get_str_or_fail(param, key);

	sprintf(key, "%s.servo_baud", rootkey);
	int servo_baud = bot_param_get_int_or_fail(param, key);

	sprintf(key, "%s.servo_parity", rootkey);
	char *servo_parity = bot_param_get_str_or_fail(param, key);
	
	// motor port info
	sprintf(key, "%s.motor_device", rootkey);
    char *motor_device = bot_param_get_str_or_fail(param, key);

	sprintf(key, "%s.motor_baud", rootkey);
    int motor_baud = bot_param_get_int_or_fail(param, key);

	sprintf(key, "%s.motor_parity", rootkey);
    char *motor_parity = bot_param_get_str_or_fail(param, key);
	
	// now lets open some ports
	state.servo_fd = serial_open(servo_device, serial_translate_speed(servo_baud), serial_translate_parity(servo_parity), 1);
	if(state.servo_fd < 1) {
		fprintf(stderr, "Could not open servo serial port %s\n", servo_device);
		return 1;
	}
	serial_set_ctsrts(state.servo_fd, 0);
	
	state.motor_fd = serial_open(motor_device, serial_translate_speed(motor_baud), serial_translate_parity(motor_parity), 1);
	if(state.motor_fd < 1) {
		fprintf(stderr, "Could not open motor serial port %s\n", servo_device);
		return 1;
	}
	serial_set_ctsrts(state.motor_fd, 0);
	
	acfrlcm_auv_iver_motor_command_t_subscribe(state.lcm, "IVER_MOTOR", &motor_handler, &state);
	
	// process
	while(!program_exit) 
	{
		struct timeval tv;
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		lcmu_handle_timeout(state.lcm, &tv);
	}
	
	close(state.servo_fd);
	close(state.motor_fd);
	
	return 0;
}
