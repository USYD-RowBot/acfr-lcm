/* Bluefin tail driver
 *
 * Christian Lees
 * ACFR
 * 27/06/2016
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <libgen.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <error.h>
#include <bot_param/param_client.h>
#include "acfr-common/sensor.h"
#include "acfr-common/units.h"
#include "acfr-common/lcm_util.h"

#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/acfrlcm_auv_nga_motor_command_t.h"
#include "perls-lcmtypes/acfrlcm_auv_bluefin_tail_command_t.h"
#include "perls-lcmtypes/acfrlcm_auv_bluefin_tail_status_t.h"


#define RS485_WE(fd, x) set_rts(fd, x)


#define BF_RE_MAX 12*DTOR
#define BF_RE_MIN -12*DTOR
#define COMMAND_TIMEOUT 10000000



typedef struct 
{
    lcm_t *lcm;
    acfr_sensor_t *sensor;
    bool enabled;
//    acfrlcm_auv_bluefin_tail_command_t bf_command;
    acfrlcm_auv_bluefin_tail_status_t bf_status;
    int64_t command_utime;
    int64_t zero_time;
    int64_t last_zero_time;
    int thruster;
    double rudder;
    double elevator;
    int max_rpm;

    bool error_main;
    bool error_rudder;
    bool error_elevator; 
    bool homed;   
} state_t;

int bluefin_write_respond(state_t *state, char *d, int timeout);

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
 
    
   
int parse_bluefin_message(state_t *state, char *d, int len)
{
    // Break the returned message down
    char addr_str[3] = {0};
    char cmd_str[2];
    int addr;
    
    memcpy(addr_str, &d[1], 2);
    addr = atoi(addr_str);
    memcpy(cmd_str, &d[3], 2);
    
    printf("Got data: %d %s\n", addr, d);
    
    // Main thruster reponses
    if(addr == 1)
    {   
        // Thruster RPM command response
        if(d[3] == 'M' && d[4] == 'V')
        {
            if(d[6] == '1')
                state->error_main = false;
            else
                state->error_main = true;
            
            return 1;
        }
        
        // Sensor command response
        if(d[3] == 'Q' && d[4] == '1')
        {
            char *tok[16];
            int ret = chop_string(d, " ", tok);
            if(ret == 6)
            {
                state->bf_status.tail_utime = timestamp_now();    
                state->bf_status.comp1 = atoi(tok[2]);
                state->bf_status.comp2 = atoi(tok[3]);
                state->bf_status.leak = atoi(tok[4]);
                state->bf_status.tail_temp = atoi(tok[5]);
                
                return 1;
            }
        }

        if(d[3] == 'Q' && d[4] == '0')
        {
            char *tok[16];
            int ret = chop_string(d, " ", tok);
            if(ret == 4)
            {
                state->bf_status.current_rpm = atoi(tok[2]);
                state->bf_status.target_rpm = atoi(tok[3]);
                
                return 1;
            }            
        }
    }
    // Rudder reponses
    if(addr == 2)
    {
        if(d[3] == 'M' && d[4] == 'P')
        {
            if(d[6] == '1')
                state->error_rudder = false;
            else
                state->error_rudder = true;
                
            return 1;
        }

        if(d[3] == 'H' && d[4] == 'M')
        {
            if(d[6] == '1')
                state->homed = true;
            else
                state->homed = false;
                
            return 1;
        }
    }

    // Elevator reponses
    if(addr == 3)
    {
        if(d[3] == 'M' && d[4] == 'P')
        {
            if(d[6] == '1')
                state->error_elevator = false;
            else
                state->error_elevator = true;
            
            return 1;
        }

        if(d[3] == 'H' && d[4] == 'M')
        {
            if(d[6] == '1')
                state->homed = true;
            else
                state->homed = false;
                
            return 1;
        }

    }
    
    
    // PSU responses
    if(addr == 4)
    {
        // Enable
        if(d[3] == 'E')
        {
            if(d[4] == '1' && d[6] == 'O' && d[7] == 'K')
            {
                state->enabled = true;
                printf("PSU enabled\n");
                // At this point the tail will send three lines of data
                char buf[64];
                for(int i = 0; i<4; i++)
                {
                    int ret = acfr_sensor_read_timeout(state->sensor, buf, sizeof(buf), 1);
                    if(ret > 0)
                    {
                        printf("%s", buf);
                    }
                }
                usleep(2e6);
                
                bluefin_write_respond(state, "#02AO\n", 2);
                bluefin_write_respond(state, "#02HM\n", 10);
                bluefin_write_respond(state, "#03AO\n", 2);
                bluefin_write_respond(state, "#03HM\n", 10);
                usleep(1e6);
            }
            else if(d[4] == '0' && d[6] == 'O' && d[7] == 'K')
            {
                state->enabled = false;
                printf("PSU disabled\n");
            }
        }
        // Sensor
        if(d[3] == 'S')
        {
            char *tok[16];
            int ret = chop_string(d, " ", tok);
            if(ret == 5)
            {
                state->bf_status.utime = timestamp_now();
                state->enabled = atoi(tok[1]);    
                state->bf_status.voltage = atof(tok[2]);
                state->bf_status.current = atof(tok[3]);
                state->bf_status.psu_temp = atof(tok[4]);
                
                acfrlcm_auv_bluefin_tail_status_t_publish(state->lcm, "NGA.BLUEFIN_STATUS", &state->bf_status);
                return 1;
            }
        }
    }

    
    return 0;
}

int bluefin_write(acfr_sensor_t *sensor, char *d)
{
    int ret = acfr_sensor_write(sensor, d, strlen(d));

    return ret;
}

// Send a command and wait for a response and parse
int bluefin_write_respond(state_t *state, char *d, int timeout)
{
    //printf("****Sending data: %s\n", d);
    int ret = acfr_sensor_write(state->sensor, d, strlen(d));
    
    //usleep(10000);
    
    char buf[64];
    memset(buf, 0, sizeof(buf));
    int bytes;
    if(ret > 0)
    {
        bytes =  acfr_sensor_read_timeout(state->sensor, buf, sizeof(buf), timeout);
        //printf("****Got %d bytes, response %s\n", bytes, buf);
        if(bytes > 0)
            return parse_bluefin_message(state, buf, bytes);
        else
            printf("RX timeout\n");
    }
    else
        return -2;
        
    return 0;                
}


int program_exit;
void
signal_handler(int sig_num)
{
    // do a safe exit
    program_exit = 1;
}

int send_bluefin_tail_commands(state_t *state)
{
    char msg[32];
    bool commanded;
    int ret;
    int retry;

    //printf("Sending values: %d, %f, %f\n", state->bf_command.main, state->bf_command.rudder * RTOD, state->bf_command.elevator * RTOD);
    //printf("Sending values: %d, %f, %f\n", state->thruster, state->rudder * RTOD, state->elevator * RTOD);
    
    
    if(!state->enabled)
    {
        bluefin_write_respond(state, "#04E1\n", 1);
    }
    
    // Main
    commanded = false;
    retry = 0;
    while(!commanded && retry++ < 5)
    {
        sprintf(msg, "#01MV %d\n", state->thruster);
        printf("Sending, attemp %d: %s\n", retry, msg);
        ret = bluefin_write_respond(state, msg, 1);
        if(ret == 1 && state->error_main)
        {
            printf("Failed, reset, retry\n");
            bluefin_write_respond(state, "#01AO\n", 1);
      //      usleep(10000);
        }
        else
            commanded = true;
    }
    
    // Rudder
    commanded = false;
    retry = 0;
    while(!commanded && retry++ < 5)
    {
        sprintf(msg, "#02MP %2.1f\n", state->rudder * RTOD);
        ret = bluefin_write_respond(state, msg, 1);
        if(ret == 1 && state->error_rudder)
        {
            printf("Failed, reset, retry\n");
            bluefin_write_respond(state, "#02AO\n", 1);
       //     usleep(10000);
        }
        else
            commanded = true;
    }
    
    // Elevator
    commanded = false;
    retry = 0;
    while(!commanded && retry++ < 5)

    {
        sprintf(msg, "#03MP %2.1f\n", state->elevator * RTOD);
        ret = bluefin_write_respond(state, msg, 1);
        if(ret == 1 && state->error_elevator)
        {
            printf("Failed, reset, retry\n");
            bluefin_write_respond(state, "#03AO\n", 1);
        //    usleep(10000);
        }
        else
            commanded = true;
    }
    
    return 1;
    
}

void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    
       // bluefin_write_respond(state, "#04S\n", 1);
    // On the heart beat we will get the ouput voltage and current if the thruster is enabled
    //if(state->enabled)
    {
        // fill status message
        bluefin_write_respond(state, "#01Q0\n", 1);
        bluefin_write_respond(state, "#01Q1\n", 1);
        bluefin_write_respond(state, "#04S\n", 1);

        
        // If we are enabled then we send the tail commands every second for the timeout period
        // after receiving a command
        if((hb->utime - state->command_utime) < COMMAND_TIMEOUT)
           send_bluefin_tail_commands(state);
    }            
}

void nga_motor_command_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_nga_motor_command_t *mot, void *u)
{
    state_t *state = (state_t *)u;

    
    // Copy the commands and set the limits
    
    state->command_utime = mot->utime;
    if(mot->tail_thruster > state->max_rpm)
        state->thruster = state->max_rpm;
    else if(mot->tail_thruster < -state->max_rpm)
        state->thruster = -state->max_rpm;
    else
        state->thruster = mot->tail_thruster;
        
    if(mot->tail_rudder > BF_RE_MAX)
        state->rudder = BF_RE_MAX;
    else if(mot->tail_rudder < BF_RE_MIN)
        state->rudder = BF_RE_MIN;
    else
        state->rudder = mot->tail_rudder;

    if(mot->tail_elevator > BF_RE_MAX)
        state->elevator = BF_RE_MAX;
    else if(mot->tail_elevator < BF_RE_MIN)
        state->elevator = BF_RE_MIN;
    else
        state->elevator = mot->tail_elevator;
    

    // Check for a zero condition, as the auv-control program continuously sends data this is how we will
    // check for an idle condition to shut the power supply down.
    if(fabs(state->rudder) < 1e-6 && fabs(state->elevator) < 1e-6 && fabs(state->thruster) < 1e-6)
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
        send_bluefin_tail_commands(state);
    else
        state->enabled = false;
}

int main (int argc, char *argv[])
{

    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);

    state_t state;
    memset(&state, 0, sizeof(state_t));
//    memset(&state.bf_command, 0, sizeof(acfrlcm_auv_bluefin_tail_command_t));
    memset(&state.bf_status, 0, sizeof(acfrlcm_auv_bluefin_tail_status_t));
    state.enabled = false;
    
    //Initalise LCM object
    state.lcm = lcm_create(NULL);
    

    char rootkey[64];
    sprintf(rootkey, "acfr.%s", basename(argv[0]));

    state.sensor = acfr_sensor_create(state.lcm, rootkey);
    if(state.sensor == NULL)
        return 0;

    // Read the max RPM value from the config file
    char key[64];
    sprintf(key, "%s.max_rpm", rootkey);
    state.max_rpm = bot_param_get_int_or_fail(state.sensor->param, key);

    // Set canonical mode
    acfr_sensor_canonical(state.sensor, '\r', '\n');
/*
	#define TIOCGRS485      0x542E
	#define TIOCSRS485      0x542F
	struct serial_rs485 rs485conf;

	// Enable RS485 mode: 
	rs485conf.flags |= SER_RS485_ENABLED;

	// Set logical level for RTS pin equal to 1 when sending: 
	rs485conf.flags |= SER_RS485_RTS_ON_SEND;

	// or, set logical level for RTS pin equal to 0 after sending: 
	rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);

	// Set rts delay before send, if needed: 
	//rs485conf.delay_rts_before_send = ...;

	// Set rts delay after send, if needed: 
	//rs485conf.delay_rts_after_send = ...;

	// Set this flag if you want to receive data even whilst sending data 
	rs485conf.flags &= ~(SER_RS485_RX_DURING_TX);

	if (ioctl (state.sensor->fd, TIOCSRS485, &rs485conf) < 0) {
		// Error handling. See errno. 
		printf("error setting rd485 mode\n");
		perror("RS485 RTS");
	}
*/
    char buf[128];
  

    // get the bluefin version numbers and print them out, we need to get three lines back
    /*bluefin_write(state.sensor, "#00!?\r\n" );
    int line_count = 0;
    for(int i = 0; i<3; i++)
    {
        int ret = acfr_sensor_read_timeout(state.sensor, buf, sizeof(buf), 1);
        if(ret > 0)
        {
            line_count++;
            printf("%s", buf);
        }
    }
    if(line_count != 3)
    {
        fprintf(stderr, "The Bluefin tail did not return the correct number of items\n");
        acfr_sensor_destroy(state.sensor);
        return 0;
    }
*/
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    //acfrlcm_auv_bluefin_tail_command_t_subscribe(state.lcm, "BLUEFIN_COMMAND", &bluefin_command_handler, &state);
    acfrlcm_auv_nga_motor_command_t_subscribe(state.lcm, "NGA.NEXTGEN_MOTOR", &nga_motor_command_handler, &state);


    // the main loop is for reading data from the serial port that isn't handled after a write and the LCM messages
    fd_set rfds;
    int lcm_fd = lcm_get_fileno(state.lcm);
    int bytes;
    
    while (!program_exit)
    {
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        FD_ZERO(&rfds);
        FD_SET(lcm_fd, &rfds);
        FD_SET(state.sensor->fd, &rfds);
        
        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
        {
            if(FD_ISSET(lcm_fd, &rfds))
                lcm_handle(state.lcm);
            else
            {
		memset(buf, 0, sizeof(buf));
                bytes = acfr_sensor_read_timeout(state.sensor, buf, sizeof(buf), 1);
                if(bytes > 1)
                     parse_bluefin_message(&state, buf, bytes);
             }
        }
    }

    acfr_sensor_destroy(state.sensor);
    return 1;
}

