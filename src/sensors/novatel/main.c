/*
 *  Novatel GPS LCM driver
 *  Decodes the ASCII version of the INSPVA message
 *  This is a stop gap driver whilst we still need the old seabed_gui to work.
 *
 *  Christian Lees
 *  ACFR
 *  29/4/12
 */

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

#include "perls-common/serial.h"
#include "perls-lcmtypes/senlcm_novatel_t.h"
#include "perls-common/timestamp.h"

#include "time_conversion.h"



#define DTOR M_PI/180

enum {io_socket, io_serial};

int readline(int fd, char *buf, int max_len)
{
    int i=0;
    do
    {
        if(recv(fd, &buf[i++], 1, 0) == -1)
        {
            printf("BREAK\n");
            break;
        }
    }
    while((buf[i-1] != '\n'));
    return i;
}

int chop_string(char *data, char **tokens)
{
    char *token;
    int i = 0;

    token = strtok(data, " ,:");
    while(token != NULL)
    {
        tokens[i++] = token;
        token = strtok(NULL, " ,:");
    }
    return i;
}

int program_gps(int fd, char *cmd)
{
    write(fd, cmd, strlen(cmd));
    return 1;
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
            snprintf(*channel_name, 200, "%s.NOVATEL", (char *)optarg);
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

    // read the config file
    BotParam *param;
    char rootkey[64];
    char key[64];

    lcm_t *lcm = lcm_create(NULL);
    param = bot_param_new_from_server (lcm, 1);

    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    char *channel_name;
    parse_args(argc, argv, &channel_name);


    // read the config file
    char *io_str;
    int io;
    sprintf(key, "%s.io", rootkey);
    io_str = bot_param_get_str_or_fail(param, key);
    if(!strcmp(io_str, "serial"))
        io = io_serial;
    else if(!strcmp(io_str, "socket"))
        io = io_socket;

    char *serial_dev;
    char *inet_port;
    char *ip;
    int baud;
    char *parity;


    if(io == io_serial)
    {
        sprintf(key, "%s.serial_dev", rootkey);
        serial_dev = bot_param_get_str_or_fail(param, key);

        sprintf(key, "%s.baud", rootkey);
        baud = bot_param_get_int_or_fail(param, key);

        sprintf(key, "%s.parity", rootkey);
        parity = bot_param_get_str_or_fail(param, key);
    }

    if(io == io_socket)
    {
        sprintf(key, "%s.ip", rootkey);
        ip = bot_param_get_str_or_fail(param, key);

        sprintf(key, "%s.port", rootkey);
        inet_port = bot_param_get_str_or_fail(param, key);
    }



    // Connect to the GPS
    int gps_fd;
    struct addrinfo hints, *gps_addr;
    if(io == io_socket)
    {

        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        getaddrinfo(ip, inet_port, &hints, &gps_addr);
        gps_fd = socket(gps_addr->ai_family, gps_addr->ai_socktype, gps_addr->ai_protocol);
    }
    else
    {
        gps_fd = serial_open(serial_dev, serial_translate_speed(baud), serial_translate_parity(parity), 1);
        if(gps_fd < 0)
        {
            printf("Error opening port %s\n", serial_dev);
            return 0;
        }
        serial_set_canonical(gps_fd, '\r', '\n');
    }

    printf("Programing GPS\n");
    program_gps(gps_fd, "UNLOGALL\r\n");
    program_gps(gps_fd, "LOG INSPVA ONTIME 0.2\r\n");
    program_gps(gps_fd, "LOG BESTPOS ONTIME 0.2\r\n");

    printf("Ready\n");
    struct timeval tv;

    int connected = 0;
    char data[256];
    //struct timeval tv;
    fd_set rfds;
    int ret;

    // main loop
    while(!program_exit)
    {
        if(io == io_socket)
        {
            if(!connected)
            {
                tv.tv_sec = 2;
                tv.tv_usec = 0;
                FD_ZERO(&rfds);
                FD_SET(gps_fd, &rfds);
                int ret = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
                if(ret != -1)
                {
                    if(connect(gps_fd, gps_addr->ai_addr, gps_addr->ai_addrlen) < 0)
                    {
                        perror("GPS connect");
                        continue;
                    }
                    else
                    {
                        printf("GPS: got a connection\n");
                        tv.tv_sec = 1;  // 1 Secs Timeout
                        tv.tv_usec = 1000;  // Not init'ing this can cause strange errors
                        setsockopt(gps_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

                        connected = 1;
                    }
                }
                else
                {
                    printf("Select timeout\n");
                    continue;
                }
            }
        }

        senlcm_novatel_t nov;

        // now receive and process the GPS messages

        memset(data, 0, sizeof(data));
        if(io == io_socket)
            readline(gps_fd, data, 256);
        else
            read(gps_fd, data, 256);

        nov.utime = timestamp_now();
        //printf("%s\n", data);
        char *tok[64];
        ret = chop_string(data, tok);

        // we need to decode the inspva message as well as the bestpos message for the standard deviations
        // for now we will determine the message type by the response length, I'll fix this later
        if(ret > 1)
        {
            //printf("%s, %d\n", tok[0], ret);
            //if((strstr(tok[0], "INSPVA") != NULL) && ret == 13)
            if((tok[0][0] == '<') && (ret == 13))
            {
                nov.latitude = atof(tok[3]) * DTOR;
                nov.longitude = atof(tok[4]) * DTOR;
                nov.roll = atof(tok[9]) * DTOR;
                nov.pitch = atof(tok[10]) * DTOR;
                nov.heading = atof(tok[11]) * DTOR;
                nov.height = atof(tok[5]);
                nov.north_velocity = atof(tok[6]);
                nov.east_velocity = atof(tok[7]);
                nov.up_velocity = atof(tok[8]);
                nov.status = tok[12];

                // Convert the GPS time into something useful
                unsigned short g_year;
                unsigned char g_month, g_day, g_hour, g_minute;
                float g_seconds;
                TIMECONV_GetUTCTimeFromGPSTime(atoi(tok[1]), atof(tok[2]), &g_year, &g_month, &g_day, &g_hour, &g_minute, &g_seconds);

                struct tm gps_time;
                gps_time.tm_year = g_year - 1900;
                gps_time.tm_mon = g_month - 1;
                gps_time.tm_mday = g_day;
                gps_time.tm_hour = g_hour;
                gps_time.tm_min = g_minute;
                gps_time.tm_sec = (int)floor(g_seconds);
                //printf("%u, %u, %u, %u, %u, %u\n", gps_time.tm_year, gps_time.tm_mon, gps_time.tm_mday, gps_time.tm_hour, gps_time.tm_min, gps_time.tm_sec);

                time_t gps_utc_time = mktime(&gps_time);
                nov.gps_time = (int64_t)(gps_utc_time * 1e6) + (int64_t)((fmod(g_seconds,1.0)) * 1e6);
                //printf("%s\n", asctime(&gps_time));
                senlcm_novatel_t_publish(lcm, channel_name, &nov);
            }
            if((tok[0][0] == '<') && ret == 22)
            {
                nov.latitude_sd = atof(tok[8]) * DTOR;
                nov.longitude_sd = atof(tok[9]) * DTOR;
            }

        }

    }
}



