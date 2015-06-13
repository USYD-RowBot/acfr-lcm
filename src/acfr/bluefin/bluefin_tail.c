/* Bluefin tail driver
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
#include <bot_param/param_client.h>
#include "acfr-common/sensor.h"


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

int bluefin_write(acfr_sensor_t *sensor, char *d)
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


    //Initalise LCM object - specReading
    lcm_t *lcm = lcm_create(NULL);

    char rootkey[64];
    sprintf(rootkey, "acfr.%s", basename(argv[0]));

    acfr_sensor_t *sensor = acfr_sensor_create(lcm, rootkey);
    if(sensor == NULL)
        return 0;

    // Set canonical mode
    acfr_sensor_canonical(sensor, '\n', '\r');

    // get the bluefin version numbers and print them out, we need to get three lines back
    char msg[64];
    memset(msg, 0, sizeof(msg));
    sprintf(msg, "#00!?\n");
    bluefin_write(sensor, msg);
    int line_count = 0;
    for(int i = 0; i<3; i++)
    {
        int ret = acfr_sensor_read_timeout(sensor, msg, sizeof(msg), 1);
        if(ret > 0)
        {
            line_count++;
            printf("%s", msg);
        }
    }
    if(line_count != 3)
    {
        fprintf(stderr, "The Bluefin tail did not return the correct number of items\n");
        return 0;
    }


    return 1;
}

