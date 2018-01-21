#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <libgen.h>
#include <math.h>
#include <termios.h>

#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"
#include <bot_param/param_client.h>
#include "perls-lcmtypes/acfrlcm_auv_spektrum_control_command_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

#define BUFSIZE 100


typedef struct {
    char vehicle_name[BUFSIZE];
    char channel_name[BUFSIZE];

    int channels;

    char send_next;

    lcm_t *lcm;
    acfrlcm_auv_spektrum_control_command_t sc;
} state_t;

void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;

    state->send_next = 1;
}

// Parse the 16 bytes that come from the RC controller
static int
parse_rc(char *buf, state_t *state)
{

    unsigned short channel_value;
    char channel_id;
    short channel_values[state->channels];

    for(int i=0; i<state->channels; i++)
    {
        channel_id = (buf[i*2] & 0xF8) >> 3;
        channel_value = ((buf[i*2] & 0x07) << 8) | (buf[(i*2)+1] & 0xFF);

        //printf("%i-%i ", (int)channel_id, (int)channel_value);
        printf("%x-%x ", (unsigned char)buf[i*2], (unsigned char)buf[i*2+1]);

        if (channel_id < state->channels)
        {
            channel_values[(int)channel_id] = channel_value;
        }
    }
    printf("\n");

    acfrlcm_auv_spektrum_control_command_t sc;
    sc.utime = timestamp_now();
    sc.channels = state->channels;
    sc.values = channel_values;

    if (state->send_next)
    {
        state->send_next = 0;
        acfrlcm_auv_spektrum_control_command_t_publish(state->lcm, state->channel_name, &sc);
    }

    return 1;
}

int main_exit;
int broken_pipe;
void signal_handler(int sig_num)
{
    // do a safe exit
    if(sig_num == SIGPIPE)
        broken_pipe = 1;
    else
        main_exit = 1;
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
parse_args (int argc, char **argv, state_t *state)
{
    int opt;

    const char *def = "DEFAULT";
    strncpy(state->vehicle_name, def, BUFSIZE);
    snprintf(state->channel_name, BUFSIZE, "SPEKTRUM_CONTROL.%s", state->vehicle_name);

    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            strncpy(state->vehicle_name, (char *)optarg, BUFSIZE);
            snprintf(state->channel_name, BUFSIZE, "%s.SPEKTRUM_CONTROL", state->vehicle_name);
            break;
         }
    }
}

static void *
lcm_thread (void *context)
{
    state_t *state = (state_t *) context;
    printf("LCM thread starting\n");
    while (!main_exit)
    {
        lcm_handle_timeout(state->lcm, 1000);
    }

    return 0;
}

void realign(acfr_sensor_t *sensor)
{
    // this is fragile - and may fail arbitrarily
    // but we need to find the start of the packet
    // based on decoding with an oscilloscope
    // the first two bytes of the message is 0xE1 0xA2
    
    fd_set rfds;
    unsigned char buf[16];
    uint8_t aligned = 0;

    printf("Aligning to message.\n");
    while(!aligned)
    {
        if(broken_pipe)
        {
            sensor->port_open = 0;
            fprintf(stderr, "Pipe broken\n");
            continue;
        }

        memset(buf, 0, sizeof(buf));

        tcflush(sensor->fd, TCIOFLUSH);
        tcflush(sensor->fd, TCIOFLUSH);

        FD_ZERO(&rfds);
        FD_SET(sensor->fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 2000;

        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
        {
            perror("Select failure: ");
        }
        else if(ret != 0)
        {
            int len;
            // deliberately don't align with expected packet size of 16 bytes
            int bytes = 16;
            len = acfr_sensor_read(sensor, buf, bytes);
            printf("%i\n", len);
            if(len == bytes && buf[1] == 0xa2)
            {
                aligned = 1;
            }
            else
            {
                fprintf(stderr, "Wrong number of bytes (%i)\n", len);
            }
        }
        else
        {
            fprintf(stderr, "Timeout: Checking connection\n");
        }
    }

    printf("Aligned packet.\n");
}

int
main(int argc, char **argv)
{
    // install the signal handler
    main_exit = 0;
    broken_pipe = 0;
    signal(SIGINT, signal_handler);
    //signal(SIGPIPE, signal_handler);

    state_t state;

    parse_args(argc, argv, &state);

    state.lcm = lcm_create(NULL);
    BotParam *param = bot_param_new_from_server (state.lcm, 1);

    pthread_t tid;
    pthread_create(&tid, NULL, lcm_thread, &state);
    pthread_detach(tid);

	perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_10HZ", &heartbeat_handler, &state);

    if(param == NULL)
        return 0;

    // Read the config params
    char rootkey[64];
    char key[64];
    sprintf(rootkey, "acfr.%s", basename(argv[0]));

    sprintf(key, "%s.channels", rootkey);
    state.channels = bot_param_get_int_or_fail(param, key);

    acfr_sensor_t *sensor = acfr_sensor_create(state.lcm, rootkey);

    if(sensor == NULL)
        return 0;

    acfr_sensor_noncanonical(sensor, 1, 0);

    // Create the UDP listener
    realign(sensor);

    fd_set rfds;
    unsigned char buf[16];

    while(!main_exit)
    {
        if(broken_pipe)
        {
            sensor->port_open = 0;
            fprintf(stderr, "Pipe broken\n");
            continue;
        }
        memset(buf, 0, sizeof(buf));

        FD_ZERO(&rfds);
        FD_SET(sensor->fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
        {
            perror("Select failure: ");
        }
        else if(ret != 0)
        {
            int len;
            int bytes = 16;
            len = acfr_sensor_read(sensor, buf, bytes);
            if(len == bytes)
            {
                parse_rc(buf, &state);
            }
            else
            {
                fprintf(stderr, "Wrong number of bytes (%i)\n", len);
            }
        }
        else
        {
            fprintf(stderr, "Timeout: Checking connection\n");
        }
    }
    acfr_sensor_destroy(sensor);
    pthread_join(tid, NULL);

    return 0;
}

