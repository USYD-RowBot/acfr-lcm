#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <libgen.h>
#include <modbus/modbus.h>
#include <sys/ioctl.h>
#include <bot_param/param_client.h>
#include "acfr-common/sensor.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/senlcm_keller_depth_t.h"

typedef struct 
{
    lcm_t *lcm;
    modbus_t *mb;  
    char *channel_name;  
} state_t;


void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    uint16_t reg[2];
    int rc;
    senlcm_keller_depth_t kd;
    
    kd.utime = timestamp_now();
    rc = modbus_read_registers(state->mb, 0x0002, 2, reg);
    if(rc == -1)
    {
    	fprintf(stderr, "Read pressure: %s\n", modbus_strerror(errno));
    }
    else
    	kd.pressure = (double)modbus_get_float(reg);
    	
    
    rc = modbus_read_registers(state->mb, 0x0002, 2, reg);
    if(rc == -1)
    {
    	fprintf(stderr, "Read temperature: %s\n", modbus_strerror(errno));
    }
    else
    	kd.temperature = (double)modbus_get_float(reg);
    
    
    senlcm_keller_depth_t_publish(state->lcm, state->channel_name, &kd);	
    
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
            snprintf(*channel_name, 200, "%s.KELLER_DEPTH", (char *)optarg);
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
    BotParam *param = bot_param_new_from_server (state.lcm, 1);
    
    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));
    
    char key[64];
    sprintf(key, "%s.serial_dev", rootkey);
    char *serial_dev = bot_param_get_str_or_fail(param, key);

    sprintf(key, "%s.baud", rootkey);
    int baud = bot_param_get_int_or_fail(param, key);
    
    sprintf(key, "%s.address", rootkey);
    int address = bot_param_get_int_or_fail(param, key);

	state.mb = modbus_new_rtu(serial_dev, baud, 8, 'N', 1);
	modbus_set_slave(state.mb, address);
	modbus_connect(state.mb);
	
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);
    
    while (!program_exit)
    {
    	lcm_handle_timeout(state.lcm, 1000);
    }

    return 1;
}


