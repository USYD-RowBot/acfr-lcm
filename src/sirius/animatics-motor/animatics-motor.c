#include <signal.h>
#include <stdio.h>
#include <ctype.h>
#include <bot_param/param_client.h>


#include "perls-lcmtypes/acfrlcm_auv_sirius_motor_command_t.h"
#include "perls-lcmtypes/acfrlcm_auv_sirius_motor_status_t.h"
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
    char thruster_channel;

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
    len = sprintf(animatics_str, "MV A=800 V=%i G t=0\n",thr_ref);
    acfr_sensor_write(sensor, animatics_str, len);

    usleep(5000);
    acfr_sensor_write(sensor, ANIMATICS_PRINT_STRING, strlen(ANIMATICS_PRINT_STRING));

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
			
    send_motor_command(state->sensor, rpm);
}


void
heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    // this is used to make sure some process up stream doesn't die and leave us in a bad
    // state.  If a timeout occurs then we will shut the motors down
    state_t *state = (state_t *)u;

    int64_t time_diff = hb->utime - state->last_motor_command;
    if(time_diff > MOTOR_TIMEOUT)
    {
        send_motor_command(state->sensor, 0);
    }
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
            *vehicle_name = malloc(n);
            strcpy(*vehicle_name, (char *)optarg);
            break;
        case 'k':
 			n = strlen((char *)optarg);
            *config_key = malloc(n);
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
    char sensor_channel[100];
    snprintf(sensor_channel, 100, "%s.4319", vehicle_name);

    lcm_t *lcm = lcm_create(NULL);
    

    state.sensor = acfr_sensor_create(lcm, rootkey);
    if(state.sensor == NULL)
        return 0;

    acfr_sensor_canonical(state.sensor, '\r', '\n');
    
    char key[128];
    sprintf(key, "%s.thruster", rootkey);
    char *thruster_name = bot_param_get_str_or_fail(state.sensor->param, key);
    
    state.thruster_channel = thruster_name[0]; 
    
    char status_channel[128];
    snprintf(status_channel, 128, "%s.ANIMATICS_STATUS.%s", vehicle_name, thruster_name);
    
    char buf[MAX_MESSAGE_SIZE];
    double omega_raw, current_raw, voltage_raw, temp_raw;
    fd_set rfds;
    int lcm_fd = lcm_get_fileno(lcm);
    
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
        	ret = acfr_sensor_read(state.sensor, buf, MAX_MESSAGE_SIZE);
            if(ret > 0)
            {
                ret = sscanf(buf,"%lf %lf %lf %lf", &omega_raw, &current_raw, &voltage_raw, &temp_raw);
                if(ret == 4)
                {
                    // form the LCM message and publish it
                    acfrlcm_auv_sirius_motor_status_t status;
                    status.utime = timestamp_now();

                    status.rpm = omega_raw / 32212.0 / 32.5 * 60.0;
                    status.voltage = voltage_raw / 10.0;
                    status.current = current_raw / 100.0;

                    acfrlcm_auv_sirius_motor_status_t_publish(lcm, status_channel, &status);
                }
            }
		}
        else
            lcm_handle(lcm);


    }
    
    acfr_sensor_destroy(state.sensor);
    
    return 1;
}
