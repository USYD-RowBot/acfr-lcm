#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <pthread.h>
#include <math.h>
#include <libgen.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <bot_param/param_client.h>


#include "perls-lcmtypes/senlcm_novatel_t.h"
#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"

#include "novatel.h"

#define DTOR M_PI/180
#define LEAP_SECONDS 16

static char *novatel_status[] =
{
    "INS Inactive",
    "INS Aligning",
    "INS Solution Bad",
    "INS Solution Good",
    "Reserved",
    "Reserved",
    "Bad INS/GPS Agreement",
    "INS Alignment Complete"
};


int program_novatel(acfr_sensor_t *s, int rate, char *com_port)
{
    char msg[256];

    printf("Using com port %s\n", com_port);
    sprintf(msg, "unlogall %s\r\n", com_port);
    acfr_sensor_write(s, msg, strlen(msg));

    sprintf(msg, "log inspvab ontime %f\r\n", 1.0/(double)rate);
    acfr_sensor_write(s, msg, strlen(msg));

    sprintf(msg, "log bestposb ontime %f\r\n", 1.0/(double)rate);
    acfr_sensor_write(s, msg, strlen(msg));

    return 1;
}

int parse_inspvab(lcm_t *lcm, char *d, int64_t timestamp, int64_t gps_time, float lat_std, float lon_std)
{
    senlcm_novatel_t nov;
    memset(&nov, 0, sizeof(senlcm_novatel_t));
    nov.utime = timestamp;
    nov.gps_time = gps_time;
    nov.latitude = *(double *)&d[12] * DTOR;
    nov.longitude = *(double *)&d[20] * DTOR;
    nov.roll = *(double *)&d[60] * DTOR;
    nov.pitch = *(double *)&d[68] * DTOR;
    nov.heading = *(double *)&d[76] * DTOR;
    nov.height = *(double *)&d[28];
    nov.north_velocity = *(double *)&d[36];
    nov.east_velocity = *(double *)&d[44];
    nov.up_velocity = *(double *)&d[52];
    nov.status = novatel_status[*(int *)&d[84]];
    nov.latitude_sd = lat_std;
    nov.longitude_sd = lon_std;

    senlcm_novatel_t_publish(lcm, "NOVATEL", &nov);

    return 1;
}

// We only extract the standard deviations out of this message
int parse_bestposb(char *d, float *lat_std, float *lon_std)
{
    *lat_std = *(float *)&d[40];
    *lon_std = *(float *)&d[44];

    return 1;
}

int program_exit;
int broken_pipe;
void
signal_handler(int sig_num)
{
    // do a safe exit
    if(sig_num == SIGPIPE)
        broken_pipe = 1;
    else
        program_exit = 1;
}

int
main (int argc, char *argv[])
{
    // install the signal handler
    program_exit = 0;
    broken_pipe = 0;
    signal(SIGINT, signal_handler);
    signal(SIGPIPE, signal_handler);

    //Initalise LCM object - specReading
    lcm_t *lcm = lcm_create(NULL);

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    acfr_sensor_t *sensor = acfr_sensor_create(lcm, rootkey);
    if(sensor == NULL)
        return 0;

    acfr_sensor_canonical(sensor, '\r', '\n');

    // read some additional config
    char key[64];
    sprintf(key, "%s.rate", rootkey);
    int rate = bot_param_get_int_or_fail(sensor->param, key);

    sprintf(key, "%s.com_port", rootkey);
    char * com_port = bot_param_get_str_or_fail(sensor->param, key);

    program_novatel(sensor, rate, com_port);

    acfr_sensor_noncanonical(sensor, 1, 0);

    fd_set rfds;
    char buf[512];
    int64_t timestamp;

    float lat_std = 0, lon_std = 0;

    while(!program_exit)
    {
        // check for broken pipes, if it is broken make sure it is closed and then reopen it
        if(broken_pipe)
            sensor->port_open = 0;

        memset(buf, 0, sizeof(buf));

        FD_ZERO(&rfds);
        FD_SET(sensor->fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
        {
            timestamp = timestamp_now();

            // read a byte and check to see if it is a sync bytes
            do
            {
                acfr_sensor_read(sensor, &buf[0], 1);
            }
            while((buf[0] & 0xFF) != 0xAA);

            // read another two bytes and check
            acfr_sensor_read(sensor, &buf[1], 2);
            if(buf[1] == 0x44 && buf[2] == 0x12)
            {
                // we have a valid header, get one more byte to get the header length
                acfr_sensor_read(sensor, &buf[3], 1);
                unsigned char header_length = (unsigned char)buf[3];

                // read the rest of the header
                acfr_sensor_read(sensor, &buf[4], header_length - 4);

                // get the message type
                unsigned short message_id = *(unsigned short *)&buf[4];
                unsigned short message_length = *(unsigned short *)&buf[8];
                unsigned short gps_week = *(unsigned short *)&buf[14];
                unsigned int gps_msec = *(unsigned int *)&buf[16];

                // work out the GPS time
                int64_t gps_time = ((((int64_t)gps_week + 1024) * 604800) * 1000000) + ((int64_t)gps_msec * 1000) + (int64_t)(315964800000000) - LEAP_SECONDS * 1000000;

                // read the rest of the message including the checksum
                acfr_sensor_read(sensor, &buf[header_length], message_length + 4);

                // Check the CRC
                unsigned long crc = CalculateBlockCRC32(header_length + message_length, (unsigned char *)buf);
                if(crc != (*(unsigned long *)&buf[header_length + message_length]))
                    printf("CRC error: 0x%08lX, actual: 0x%08lX\n", crc, *(unsigned long *)&buf[header_length + message_length]);
                else
                {
                    if(message_id == 507)
                        parse_inspvab(lcm, &buf[header_length], timestamp, gps_time, lat_std, lon_std);

                    if(message_id == 42)
                        parse_bestposb(&buf[header_length], &lat_std, &lon_std);
                }
            }
        }
        else
            fprintf(stderr, "Select timeout\n");
    }

    char msg[256];
    sprintf(msg, "unlogall %s\r\n", com_port);
    acfr_sensor_write(sensor, msg, strlen(msg));
    acfr_sensor_destroy(sensor);

    return 0;

}

