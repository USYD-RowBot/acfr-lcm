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
#include <math.h>
#include <bot_param/param_client.h>
#include "acfr-common/sensor.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/senlcm_acfr_psu_type_1_t.h"
#include "perls-lcmtypes/senlcm_acfr_psu_type_2_t.h"

#define MAX_PSUS 8

typedef struct 
{
    lcm_t *lcm;
    acfr_sensor_t *sensor;
    int num_psu;
    int psu_addrs[MAX_PSUS];
    int psu_type[MAX_PSUS];
    int psu;
    char *channel_name;
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

double l112f(char *d)
{
    uint16_t data = strtol(d, NULL, 16);
    int8_t exponent = data >> 11;
    // extract mantissa as LS 11 bits
    int16_t mantissa = data & 0x7ff;
    // sign extend exponent from 5 to 8 bits
    if( exponent > 0x0F ) exponent |= 0xE0;
    // sign extend mantissa from 11 to 16 bits
    if( mantissa > 0x03FF ) mantissa |= 0xF800;
    return mantissa * pow(2,exponent);
}
void parse_psu_response(state_t *state, char *d, int len)
{
    char addr_str[3] = {0};
    char *cmd_char;
    int addr;
    char *tokens[16];
    char pub_channel[80];
    
    memcpy(addr_str, &d[1], 2);
    addr = atoi(addr_str);
    int psu_type = -1;
    for(int i = 0; i<MAX_PSUS; i++)
        if(state->psu_addrs[i] == addr)
            psu_type = state->psu_type[i];
    if(psu_type == -1)
    {
        fprintf(stderr, "Unknown PSU type for address %d.\nSupplied message: %*s", addr, len, d);
	return;
    }

    //if (addr >= 0 && addr <= MAX_PSUS)
    // Get the index to determine the type
    {
    	cmd_char = &d[3];
    	if(*cmd_char == 'S' || *cmd_char == 's')
    	{
            int num_tokens = chop_string( d, ", ", tokens);
	    if(psu_type == 1)
    		if(num_tokens == 8)
    		{
			
    		    senlcm_acfr_psu_type_1_t psu;
	    	    psu.utime = timestamp_now();
	    	    psu.address = addr;
	    	    psu.voltage = atof(tokens[1]);
	    	    psu.current = atof(tokens[2]);
	    	    psu.temperature = atof(tokens[3]);
	            psu.voltage_max = atof(tokens[4]);
		    psu.current_max = atof(tokens[5]);
		    psu.voltage_min = atof(tokens[6]);
		    psu.current_min = atof(tokens[7]);
		    strcpy(pub_channel, state->channel_name);
		    strcat(strcat(pub_channel, "_"), addr_str);
		    if (fabs(psu.voltage_max) >1e-3  && fabs(psu.voltage_min) >1e-3 && fabs(psu.current_max) >1e-3 && fabs(psu.current_min) >1e-3 )
				senlcm_acfr_psu_type_1_t_publish(state->lcm, pub_channel, &psu);
		}
	    if(psu_type == 2)
	    {
    		if(num_tokens == 9)
		{
		    senlcm_acfr_psu_type_2_t psu;
	    	    psu.utime = timestamp_now();
	    	    psu.address = addr;
	    	    psu.voltage_in[0] = l112f(tokens[1]);
	    	    psu.voltage_out[0] = l112f(tokens[2]);
	    	    psu.current_out[0] = l112f(tokens[3]);
	    	    psu.temperature[0] = l112f(tokens[4]);
	    	    psu.voltage_in[1] = l112f(tokens[5]);
	    	    psu.voltage_out[1] = l112f(tokens[6]);
	    	    psu.current_out[1] = l112f(tokens[7]);
	    	    psu.temperature[1] = l112f(tokens[8]);
		    strcpy(pub_channel, state->channel_name);
		    strcat(strcat(pub_channel, "_"), addr_str);
		    senlcm_acfr_psu_type_2_t_publish(state->lcm, pub_channel, &psu);
		}
	        else
		{
			fprintf(stderr, "Couldn't decode PSU Type 2 message: %*s", len, d);
		}
	    }
    	}
	else
	{
	    fprintf(stderr, "Sentinel value 'S' not found in: %*s", len, d);
	}
    }	
}	
    	 

void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    char msg[16];
    char resp[75];
    int ret;	
	for(int i=0; i<state->num_psu; i++)
	{
		memset(msg, 0, sizeof(msg));
		sprintf(msg, "#%02dS\r", state->psu_addrs[i]);
		RS485_write(state->sensor, msg);
		// Wait for a response with a timeout
		usleep(10e3);
		memset(resp, 0, sizeof(resp));
		ret = acfr_sensor_read_timeoutms(state->sensor, resp, sizeof(resp), 1000/(state->num_psu));
		if(ret > 0)
		    parse_psu_response(state, resp, ret);
		else
		    fprintf(stderr, "No response from %d\n", state->psu_addrs[i]);
		usleep(10e3);
	}			    
}

void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, char **channel_name)
{
    int opt;

    const char *default_name = "DEFAULT";
    *channel_name = malloc(strlen(default_name)+1);
    strcpy(*channel_name, default_name);
    
    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            free(*channel_name);
            *channel_name = malloc(200);
            snprintf(*channel_name, 200, "%s.PSU", (char *)optarg);
            break;
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

    parse_args(argc, argv, &state.channel_name);
    
    //Initalise LCM object
    state.lcm = lcm_create(NULL);
    

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    state.sensor = acfr_sensor_create(state.lcm, rootkey);
    if(state.sensor == NULL)
    {
        return 0;
    }

    // Read the PSU addresses we are interested in
    char key[64];
    sprintf(key, "%s.addrs", rootkey);
    state.num_psu = bot_param_get_int_array(state.sensor->param, key, state.psu_addrs, MAX_PSUS);
    sprintf(key, "%s.type", rootkey);
    state.num_psu = bot_param_get_int_array(state.sensor->param, key, state.psu_type, MAX_PSUS);
	
    // Set canonical mode
    acfr_sensor_canonical(state.sensor, '\r', '\n');
  
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    state.psu = 0;
    
    while (!program_exit)
    {
    	lcm_handle_timeout(state.lcm, 1000);
    }

    acfr_sensor_destroy(state.sensor);
    return 1;
}

