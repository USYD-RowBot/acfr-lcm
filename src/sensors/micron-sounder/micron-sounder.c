#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <libgen.h>

#include "perls-lcmtypes/senlcm_micron_sounder_t.h"
#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"

void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, char *vehicle_name, char *rootkey)
{
    int opt;

    const char *default_name = "DEFAULT";
    //*channel_name = malloc(strlen(default_name)+1);
    memset(vehicle_name, 0, 32);
    strncpy(vehicle_name, default_name, strlen(default_name));
    
    while ((opt = getopt (argc, argv, "hn:k:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            //free(*channel_name);
            //*channel_name = malloc(200);
            //snprintf(*channel_name, 200, "%s.MICRON_SOUNDER", (char *)optarg);
            memset(vehicle_name, 0, 32);
            strncpy(vehicle_name, optarg, strlen(optarg));
            break;
         case 'k':
            memset(rootkey, 0, 64);
            strncpy(rootkey, optarg, strlen(optarg));
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
    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);


    char vehicle_name[32];
    char channel_name[64];
    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    parse_args(argc, argv, vehicle_name, rootkey);

    //Initalise LCM object - specReading
    lcm_t *lcm = lcm_create(NULL);

    acfr_sensor_t *sensor = acfr_sensor_create(lcm, rootkey);
    if(sensor == NULL)
        return 0;

    // Get the channel name
    char key[64];
    sprintf(key, "%s.channel", rootkey);

    if(bot_param_has_key(sensor->param, key))
    {
        char *channel = bot_param_get_str_or_fail(sensor->param, key);
        sprintf(channel_name, "%s.%s", vehicle_name, channel);
    }
    else
        sprintf(channel_name, "%s.MICRON_SOUNDER", vehicle_name);

    acfr_sensor_canonical(sensor, '\r', '\n');

    fd_set rfds;
    char buf[16];
    char value[16];
    senlcm_micron_sounder_t micron;

    while(!program_exit)
    {
        memset(buf, 0, sizeof(buf));
        memset(&micron, 0, sizeof(senlcm_micron_sounder_t));

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
            micron.utime = timestamp_now();
            if(acfr_sensor_read(sensor, buf, sizeof(buf)) > 0)
            {
                // parse out the range value leaving the m off then end
                memset(value, 0, sizeof(value));
                strncpy(value, buf, strlen(buf) - 1);
                micron.altitude = atof(value);

                senlcm_micron_sounder_t_publish(lcm, channel_name, &micron);
            }
        }
    }

    return 1;
}





