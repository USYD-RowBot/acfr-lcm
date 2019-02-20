#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <libgen.h>

#include "perls-lcmtypes/senlcm_micron_sounder_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"

#define MAX_OAS 4

typedef struct 
{
    lcm_t *lcm;
    int num_oas;
    int oas_ports[MAX_OAS];
    char *channel_name;
    acfr_sensor_t *sensors[MAX_OAS];
} state_t;

void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    static int beat_count = 1;
    static int sensor_count = 0;

    char msg[32];
    char resp[75];
    char pub_channel[80];
    char value[16];
    senlcm_micron_sounder_t micron;
    if ( beat_count++ % 5 == 0)
    {
        memset(msg, 0, sizeof(msg));
        sprintf(msg, "Z");
        acfr_sensor_write(state->sensors[sensor_count], msg, strlen(msg));
        // Wait for a response with a timeout
        memset(resp, 0, sizeof(resp));
        int ret = acfr_sensor_read_timeoutms(state->sensors[sensor_count], resp, sizeof(resp), 1e3/(state->num_oas));
        if(ret > 0){
            // parse out the range value leaving the m off then end
            micron.utime = timestamp_now();
            memset(value, 0, sizeof(value));
            //printf("%s", resp);
            strncpy(value, resp, strlen(resp) - 1);
            micron.altitude = atof(value);
            strcpy(pub_channel, state->channel_name);
            memset(msg, 0, sizeof(msg));
            sprintf(msg, "_%d", state->oas_ports[sensor_count]);
            strcat(pub_channel, msg);
            senlcm_micron_sounder_t_publish(state->lcm, pub_channel, &micron);
        }
        beat_count = 1;
        if (sensor_count++ == (state->num_oas-1))
            sensor_count =0;
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
            snprintf(*channel_name, 200, "%s.MICRON_SOUNDER", (char *)optarg);
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

int
main (int argc, char *argv[])
{
    state_t state;
    //memset(&state, 0, sizeof(state_t));

    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);

    parse_args(argc, argv, &state.channel_name);

    // read the config file
    BotParam *param;
    char rootkey[64];
    char key[64];

    state.lcm = lcm_create(NULL);
    param = bot_param_new_from_server (state.lcm, 1);

    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    // read the config file
    char *io_str;
    int io;
    sprintf(key, "%s.io", rootkey);
    io_str = bot_param_get_str_or_fail(param, key);
    if(!strcmp(io_str, "serial"))
        io = io_serial;
    else if(!strcmp(io_str, "tcp"))
        io = io_tcp;
    else
        return -1;

    char **serial_devs;
    char **inet_ports;
    char *ip;
    int baud;
    char *parity;

    sprintf(key, "%s.serial_ports", rootkey);
    state.num_oas = bot_param_get_int_array(param, key, state.oas_ports, MAX_OAS);

    if(io == io_serial)
    {
        sprintf(key, "%s.serial_dev", rootkey);
        serial_devs = bot_param_get_str_array_alloc(param, key);

        sprintf(key, "%s.baud", rootkey);
        baud = bot_param_get_int_or_fail(param, key);

        sprintf(key, "%s.parity", rootkey);
        parity = bot_param_get_str_or_fail(param, key);
    }

    if(io == io_tcp)
    {
        sprintf(key, "%s.ip", rootkey);
        ip = bot_param_get_str_or_fail(param, key);

        sprintf(key, "%s.ports", rootkey);
        inet_ports = bot_param_get_str_array_alloc(param, key);
    }

    int lcm_fd = lcm_get_fileno(state.lcm);

    printf("oas devs found = %d\n" , state.num_oas);
    for(int i=0; i<state.num_oas; i++)
    {
        if(io == io_tcp)
        {
            printf("*");
            state.sensors[i]->io_type = io;
            state.sensors[i]->ip = ip;
            state.sensors[i]->inet_port = inet_ports[i];
        }
        else
        {
            printf("#");
            state.sensors[i]->io_type = io;
            state.sensors[i]->serial_dev = serial_devs[i];
            state.sensors[i]->baud = baud;
            state.sensors[i]->parity = parity;
        }
        acfr_sensor_open(state.sensors[i]);
        acfr_sensor_canonical(state.sensors[i], '\r', '\n');
    }
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_10HZ", &heartbeat_handler, &state);

    while (!program_exit)
    {
        lcm_handle_timeout(state.lcm, 1000);
    }
    
    for(int i=0; i < state.num_oas ; i++)
    {
        acfr_sensor_destroy(state.sensors[i]);
    }
    
    return 1;
}





