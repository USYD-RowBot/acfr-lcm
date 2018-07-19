#include <signal.h>
#include <stdio.h>
#include <ctype.h>
#include <bot_param/param_client.h>


#include "perls-lcmtypes/acfrlcm_auv_sirius_motor_command_t.h"
#include "perls-lcmtypes/acfrlcm_auv_sirius_motor_status_t.h"
#include "perls-lcmtypes/acfrlcm_relay_status_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"	
#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"

#define MAX_MESSAGE_SIZE 128
#define MOTOR_TIMEOUT 10000000  // this is in microseconds

#define ANIMATICS_PRINT_STRING "PRINT(V,\" \",UIA,\" \",UJA,\" \",TEMP,#13,#10)\n"

typedef struct
{
	acfr_sensor_t *sensor;
    int64_t last_motor_command;
    int64_t last_motor_status;
    char thruster_channel;
    double max_rpm;
    int relay_number;
    bool enabled;

} state_t;

void
send_motor_command(acfr_sensor_t *sensor, double rpm)
{
    // send a command to the motors
    char animatics_str[MAX_MESSAGE_SIZE];
    int thr_ref, len;

    // from animatics user manual p46
    // the 32212 is for the motor, 32.5 is for the gear box, 60 is from RPM to Hz
    // compose animatics command string for velocity mode

    thr_ref = (int)(rpm * 32212.0 * 32.5 / 60.0);
    memset(animatics_str, 0, MAX_MESSAGE_SIZE);
    len = snprintf(animatics_str, MAX_MESSAGE_SIZE, "MV A=800 V=%i G t=0\n",thr_ref);
    acfr_sensor_write(sensor, animatics_str, strlen(animatics_str));
//printf("Sending %s", animatics_str);
//printf("%s", ANIMATICS_PRINT_STRING);
    usleep(1000);
    memset(animatics_str, 0, MAX_MESSAGE_SIZE);
    strncpy(animatics_str, ANIMATICS_PRINT_STRING, strlen(ANIMATICS_PRINT_STRING));
    //acfr_sensor_write(sensor, ANIMATICS_PRINT_STRING, strlen(ANIMATICS_PRINT_STRING));
    acfr_sensor_write(sensor, animatics_str, strlen(animatics_str));
//printf("Sending %s", animatics_str);

}

void
relay_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_relay_status_t *rs, void *u)
{
    state_t *state = (state_t *)u;
    if(rs->state_list[0] & (1 << state->relay_number))
    {
	if(!state->enabled)
	{
	    printf("Enabling motor\n");
	    usleep(4e6);
	}
	state->enabled = 1;
    }
    else
	state->enabled = 0;
}

void
motor_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_sirius_motor_command_t *mc, void *u)
{
    state_t *state = (state_t *)u;

    state->last_motor_command = mc->utime;
	
	double rpm;
	switch(state->thruster_channel)
	{
		case 'P':
		case 'p':
			rpm = mc->port;
			break;
		case 'S':
		case 's':
			rpm = mc->starboard;
			break;
		case 'V':
		case 'v':
			rpm = mc->vertical;
			break;
	}
//printf("RPM: %f\n", rpm);		
	// hard limit the rpm
	if(rpm > state->max_rpm)
		rpm  = state->max_rpm;
	else if(rpm < -state->max_rpm)
		rpm = -state->max_rpm;
//    if((timestamp_now() - state->last_motor_command) > 500000)			
    if(state->enabled)
    {
        send_motor_command(state->sensor, rpm);
    
        state->last_motor_command = mc->utime;
    }
}


void
heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    // this is used to make sure some process up stream doesn't die and leave us in a bad
    // state.  If a timeout occurs then we will shut the motors down
    state_t *state = (state_t *)u;

    int64_t time_diff = hb->utime - state->last_motor_command;
    if((time_diff > MOTOR_TIMEOUT) && state->enabled)
    {
	//printf("Zeroing\n");
        send_motor_command(state->sensor, 0);
    }
/*
    if((hb->utime - state->last_motor_status) > 3e6)
    {
	    printf("Flushing and sending break\n");
	    tcflush(state->sensor->fd, TCIOFLUSH);
	    tcsendbreak(state->sensor->fd, 0);
	    usleep(1e6);
	    acfr_sensor_write(state->sensor, "\n\n\n\n", 4);
	    state->last_motor_status = hb->utime;
    }
*/
}


void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    printf("  -k config file key               eg. acfr.animatics.port\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, char **vehicle_name, char **config_key)
{
    int opt;

    const char *default_name = "DEFAULT";
    *vehicle_name = malloc(strlen(default_name)+1);
    strcpy(*vehicle_name, default_name);
    bool got_key = 0;
    
    int n;
    while ((opt = getopt (argc, argv, "hn:k:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            n = strlen((char *)optarg);
            free(*vehicle_name);
            *vehicle_name = malloc(n + 1); // the null char
            strcpy(*vehicle_name, (char *)optarg);
            break;
        case 'k':
            n = strlen((char *)optarg);
            *config_key = malloc(n+1); // the null char!
            strcpy(*config_key, (char *)optarg);
            got_key = 1;        	
         }
    }
    if(!got_key)
    	print_help(0, argv);
    
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
    
    state_t state;
    
    char *vehicle_name;
    char *rootkey;
    parse_args(argc, argv, &vehicle_name, &rootkey);

    lcm_t *lcm = lcm_create(NULL);

    state.sensor = acfr_sensor_create(lcm, rootkey);
    if(state.sensor == NULL)
        return 0;

    acfr_sensor_canonical(state.sensor, '\n', '\r');
    
    char key[128];
    sprintf(key, "%s.thruster", rootkey);
    char *thruster_name = bot_param_get_str_or_fail(state.sensor->param, key);
    sprintf(key, "%s.max_rpm", rootkey);
    state.max_rpm = bot_param_get_double_or_fail(state.sensor->param, key);
    
    sprintf(key, "%s.relay_number", rootkey);
    state.relay_number = bot_param_get_int_or_fail(state.sensor->param, key) - 1;
    state.enabled = false;

    state.thruster_channel = thruster_name[0]; 
    
    char status_channel[128];
    snprintf(status_channel, 128, "%s.ANIMATICS_STATUS.%s", vehicle_name, thruster_name);
    
    char channel[128];
    snprintf(channel, 128, "%s.THRUSTER", vehicle_name);
    acfrlcm_auv_sirius_motor_command_t_subscribe(lcm, channel, &motor_handler, &state);
    
    snprintf(channel, 128, "%s.RELAY_STATUS", vehicle_name);
    acfrlcm_relay_status_t_subscribe(lcm, channel, &relay_handler, &state);

    perllcm_heartbeat_t_subscribe(lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    
    
    char buf[MAX_MESSAGE_SIZE];
    double omega_raw, current_raw, voltage_raw, temp_raw;
    fd_set rfds;
    int lcm_fd = lcm_get_fileno(lcm);
   
    tcflush(state.sensor->fd, TCIOFLUSH);

    // listen to LCM and the serial port
    while(!program_exit)
    {
        FD_ZERO(&rfds);
        FD_SET(lcm_fd, &rfds);
        FD_SET(state.sensor->fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(FD_ISSET(state.sensor->fd, &rfds))
        {
	    memset(buf, 0, MAX_MESSAGE_SIZE);
            ret = acfr_sensor_read_timeoutms(state.sensor, buf, MAX_MESSAGE_SIZE, 10);
            if(ret > 0)
            {
//                printf("Received: %s", buf);
		    ret = sscanf(buf,"%lf %lf %lf %lf", &omega_raw, &current_raw, &voltage_raw, &temp_raw);
                if(ret == 4)
                {
                    // form the LCM message and publish it
                    state.last_motor_status = timestamp_now();
		    acfrlcm_auv_sirius_motor_status_t status;
                    status.utime = timestamp_now();
		    status.thruster_num = 0;
                    status.rpm = omega_raw / 32212.0 / 32.5 * 60.0;
                    status.voltage = voltage_raw / 10.0;
                    status.current = current_raw / 100.0;
		    status.temp = temp_raw;
                    acfrlcm_auv_sirius_motor_status_t_publish(lcm, status_channel, &status);
                }
            }
	    else
	    {
		// Comms timeout
	    }

		}
        else
            lcm_handle(lcm);


    }
    
    acfr_sensor_destroy(state.sensor);
    
    return 1;
}
