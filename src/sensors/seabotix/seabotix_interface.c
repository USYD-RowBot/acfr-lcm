#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>
#include <libgen.h>

#include "acfr-common/timestamp.h"
#include "perls-lcmtypes/senlcm_seabotix_sensors_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "acfr-common/sensor.h"


// Data conversion
#define d2f(x) *(float *)x
#define d2current(x) ((float)x / 10.0)
#define d2speed(x) (short)((short)(x - 0x80) * -1 * 4500 / 102)


typedef struct
{
    lcm_t *lcm;
    acfr_sensor_t *sensor;
} state_t;


unsigned char seabotix_checksum(unsigned char *message, long size)
{

    int sum = 0;

    for (int i=0; i<size; i++)   // disregard last element (should be 0 anyway)
    {
        sum = sum + message[i];
    }

    unsigned char chksm = (unsigned char)(sum & 0xFF);
    //message[size-1] = chksm;

    return chksm;
}

// Sned a request for the sensors every second
void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;

    // send an MT-26 message
    unsigned char message[] = {0x14, 0x04, 0x25, 0x00};
    message[3] = seabotix_checksum(message, 3);
    acfr_sensor_write(state->sensor, (char *)message, 4);
}

int parse_mt25(const unsigned char *d, lcm_t *lcm, int64_t timestamp)
{
    senlcm_seabotix_sensors_t rov;
    memset(&rov, 0, sizeof(senlcm_seabotix_sensors_t));

    rov.utime = timestamp;
    rov.heading = d2f(&d[8]);
    rov.depth = d2f(&d[12]);
    rov.pitch = d2f(&d[16]);
    rov.roll = d2f(&d[20]);
    rov.turns = d2f(&d[24]);
    rov.temperature_internal = d2f(&d[28]);
    rov.temperature_external = d2f(&d[32]);

    rov.PF_faults = d[36];
    rov.PF_current = d2current(d[37]);
    rov.PF_speed = d2speed(d[38]);
    rov.PF_temperature = d[39];

    rov.PA_faults = d[40];
    rov.PA_current = d2current(d[41]);
    rov.PA_speed = d2speed(d[42]);
    rov.PA_temperature = d[43];

    rov.PV_faults = d[44];
    rov.PV_current = d2current(d[45]);
    rov.PV_speed = d2speed(d[46]);
    rov.PV_temperature = d[47];

    rov.SF_faults = d[48];
    rov.SF_current = d2current(d[49]);
    rov.SF_speed = d2speed(d[50]);
    rov.SF_temperature = d[51];

    rov.SA_faults = d[52];
    rov.SA_current = d2current(d[53]);
    rov.SA_speed = d2speed(d[54]);
    rov.SA_temperature = d[55];

    rov.SV_faults = d[56];
    rov.SV_current = d2current(d[57]);
    rov.SV_speed = d2speed(d[58]);
    rov.SV_temperature = d[59];

    senlcm_seabotix_sensors_t_publish(lcm, "SEABOTIX_STATS", &rov);

    return 1;
}


int program_exit;
int broken_pipe;
void
signal_handler(int sig_num)
{
    printf("Got a signal\n");
    // do a safe exit
    if(sig_num == SIGPIPE)
        broken_pipe = 1;
    else if (sig_num == SIGTERM)
        broken_pipe = 1;
    else if(sig_num == SIGINT)
        program_exit = 1;
}

int
main (int argc, char *argv[])
{
    // install the signal handler
    program_exit = 0;
    broken_pipe = 0;

    struct sigaction sa;
    sa.sa_flags = 0;
    sigemptyset (&sa.sa_mask);
    sigaddset(&sa.sa_mask, SIGPIPE);
    sigaddset(&sa.sa_mask, SIGHUP);
    sigaddset(&sa.sa_mask, SIGTERM);
    sigaddset(&sa.sa_mask, SIGINT);
    sa.sa_handler = SIG_IGN;
    if (sigaction(SIGPIPE, &sa, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }
    sa.sa_handler = signal_handler;
    sigaction(SIGHUP, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);


    //Initalise LCM object
    state_t state;
    state.lcm = lcm_create(NULL);

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    acfr_sensor_t *sensor = acfr_sensor_create(state.lcm, rootkey);
    if(sensor == NULL)
        return 0;

    acfr_sensor_noncanonical(state.sensor, 1, 0);

    // LCM subscriptions
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);


    int len;
    char buf[256];

    int64_t timestamp;

    fd_set rfds;
    int lcm_fd = lcm_get_fileno(state.lcm);

    while(!program_exit)
    {
        // check for broken pipes, if it is broken make sure it is closed and then reopen it
        if(broken_pipe)
        {
            //sensor->port_open = 0;
            fprintf(stderr, "Pipe broken\n");
            continue;
        }

        memset(buf, 0, sizeof(buf));

        FD_ZERO(&rfds);
        FD_SET(sensor->fd, &rfds);
        FD_SET(lcm_fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
        {
            if(FD_ISSET(lcm_fd, &rfds))
                lcm_handle(state.lcm);
            else if(FD_ISSET(state.sensor->fd, &rfds))
            {
                // read a whole packet
                timestamp = timestamp_now();
                len = acfr_sensor_read(sensor, buf, sizeof(buf));

                // check the length
                if(len != buf[1])
                {
                    fprintf(stderr, "Incorrect packet length\n");
                    continue;
                }

                // check the checksum
                unsigned char checksum = seabotix_checksum((unsigned char *)buf, len - 1);
                if(checksum != buf[len - 1])
                {
                    fprintf(stderr, "Check sum error, expected 0x%02X, got 0x%02X\n", checksum & 0xFF, buf[len - 1] & 0xFF);
                    continue;
                }

                // parse the message
                switch(buf[0])
                {
                case 0x25:
                    parse_mt25((unsigned char *)buf, state.lcm, timestamp);
                    break;
                default:
                    printf("Unsupported message 0x%02X\n", buf[0] & 0xFF);
                    break;
                }
            }
        }
    }

    acfr_sensor_destroy(state.sensor);
    lcm_destroy(state.lcm);

    return 0;

}



