#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <aiousb/aiousb.h>
#include "aiocommon.h"
#include <lcm/lcm.h>

#include "acfr-common/timestamp.h"

#include "perls-lcmtypes/acfrlcm_relay_command_t.h"
#include "perls-lcmtypes/acfrlcm_relay_status_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

AIOUSB_BOOL find_idio( AIOUSBDevice *dev ) { 
    if( (dev->ProductID >= USB_IIRO_16 && dev->ProductID <= USB_IIRO_4 ) ||
        (dev->ProductID >= USB_IDIO_16 && dev->ProductID <= USB_IDIO_4 )) {
        /* found a USB-IIRO or USB-IDIO family device
         * as this sample supports both
         */
        return AIOUSB_TRUE;
    } else {
        return AIOUSB_FALSE;
    }
}

typedef struct
{
    lcm_t *lcm;
	char *vehicle_name;					// for vehicle prefix on LCM channels
	char command_channel[128];
	char status_channel[128];
	struct opts options;
} state_t;

// handles incoming LCM relay requests, sends requests to the board via tcp
void relay_cmd_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_relay_command_t *rc, void *u)
{
    state_t *state = (state_t *)u;
    
    // Read the current relay state
    unsigned int values;
    DIO_ReadAll(state->options.index, &values);
    if(!rc->relay_request)
    	values |= 1 << (rc->relay_number - 1);
    else
	values &= ~(1 << (rc->relay_number - 1));
		
    DIO_WriteAll(state->options.index, &values);
}


void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
     
    acfrlcm_relay_status_t status_msg;
    status_msg.utime = timestamp_now();
    unsigned int values;
    DIO_ReadAll(state->options.index, &values);
//    DIO_Read8(state->options.index, 0, (unsigned char *)&status_msg.state_list[1]);
    status_msg.state_list[0] = ~(values & 0xFF);
    status_msg.state_list[1] = ~((values & 0xFF00) >> 8);
    status_msg.state_list[2] = 0;
    status_msg.state_list[3] = 0;
    // publish status message
    acfrlcm_relay_status_t_publish(state->lcm, state->status_channel, &status_msg);
    
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
parse_args (int argc, char **argv, char **vehicle_name)
{
    int opt;

    const char *default_name = "DEFAULT";
    *vehicle_name = malloc(strlen(default_name)+1);
    strcpy(*vehicle_name, default_name);
    
    int n;
    while ((opt = getopt (argc, argv, "hn:")) != -1)
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
         }
    }
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
    parse_args(argc, argv, &vehicle_name);
    snprintf(state.command_channel, 128, "%s.RELAY_CONTROL", vehicle_name);
	snprintf(state.status_channel, 128, "%s.RELAY_STATUS", vehicle_name);

	AIORET_TYPE retval;
	state.options = AIO_OPTIONS;
	int *indices, num_devices = 0;
	int timeout = 1000;
	AIODeviceQuery *devq;
	
	retval = AIOUSB_Init(); /**< Call AIOUSB_Init() first */

    AIOUSB_FindDevices( &indices, &num_devices, find_idio );
    if ( num_devices <= 0 ) {
        fprintf(stderr,"Unable to find a USB-IDIO device\n");
        exit(1);
    }
    
    if ( ( retval = aio_supply_default_command_line_settings(&state.options)) != AIOUSB_SUCCESS )
        exit(retval);

	devq = NewAIODeviceQuery( state.options.index );
    if (!devq ) {
        fprintf(stderr,"Unable to query device at index %d \n", state.options.index );
        exit(1);
    }


    AIOUSB_Reset( state.options.index );
    AIOUSB_SetCommTimeout( state.options.index, timeout );


    state.lcm = lcm_create(NULL);
    acfrlcm_relay_command_t_subscribe(state.lcm, state.command_channel, &relay_cmd_handler, &state);
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    
    while (!program_exit)
    {
    	lcm_handle_timeout(state.lcm, 1000);
	}
	
	return 1;
}

