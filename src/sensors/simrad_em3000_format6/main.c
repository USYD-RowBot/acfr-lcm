#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <libgen.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netdb.h>
#include <bot_param/param_client.h>
#include "perls-common/serial.h"
#include "perls-common/units.h"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes/senlcm_simrad_em3000_format6_t.h"

typedef struct
{
    lcm_t *lcm;
    senlcm_simrad_em3000_format6_t em;
    int count;
} state_t;

enum {io_socket, io_serial};

int get_24b_int(unsigned char *d)
{
    int x = (d[0] & 0xFF) + ((d[1] << 8) & 0xFF00) + ((d[2] << 16) & 0xFF0000);
    if(x & 0x800000)
        x |= ~0xFFFFFF;

    return x;
}

int get_8b_int(unsigned char *d)
{
    int x = (d[0] & 0xFF);
    if(x & 0x80)
        x |= ~0xFF;

    return x;
}

int get_16b_int(unsigned char *d)
{
    int x = (d[0] & 0xFF) + ((1[d] << 8) & 0xFF00);
    if(x & 0x8000);
    x |= ~0xFFFF;
    return x;
}

int get_16b_int_unsin(unsigned char *d)
{
    int x = (d[0] & 0xFF) + ((d[1] << 8) & 0xFF00);
    return x;
}

double get_double(unsigned char *d)
{
    return *((double *)d);
}

float get_float(unsigned char *d)
{
    return *((float *)d);
}

float get_short(unsigned char *d)
{
    return *((short *)d);
}

int get_int(unsigned char *d)
{
    return *((int *)d);
}

const char * const gps_mode[] =
{
    "None The GPS is not able to make this measurement",
    "Search The GPS system is solving ambiguities and searching for a valid solution",
    "Doppler The GPS measurement is based on a Doppler measurement",
    "SPS Standard positioning service, the GPS measurement has no additional external corrections",
    "Differential The GPS measurement used code-phase differential corrections",
    "RTK float The GPS measurement used L1 carrier-phase differential corrections to give a floating ambiguity solution.",
    "RTK integer The GPS measurement used L1/L2 carrier-phase differential corrections to give an integer ambiguity solution",
    "WAAS The GPS measurement used SBAS corrections",
    "OmniSTAR The GPS measurement used OmniSTAR VBS corrections",
    "OmniSTAR HP The GPS measurement used OmniSTAR HP corrections",
    "No data",
    "Blanked",
    "Doppler (PP) Doppler GPS measurement post processed with GrafNav",
    "SPS (PP) SPS GPS measurement post processed with GrafNav",
    "Differential (PP) Differential GPS measurement post processed with GrafNav",
    "RTK float (PP) RTK Float GPS measurement post processed with GrafNav",
    "RTK integer(PP) RTK Integer GPS measurement post processed with GrafNav",
    "OmniSTAR XP The GPS measurement used OmniSTAR XP corrections",
    "CDGPS",
    "Not recognised",
    "Unknown"
};

/*
// GPS status message
int parse_ncom_channel_0(unsigned char *d, state_t *state)
{
    state->em.gps_time = get_int(&d[0]);
    state->em.gps_sats = d[4];

    if(d[5] > 20)
        d[5] = 20;
    state->em.gps_pos_mode = gps_mode[d[5]];

    if(d[6] > 20)
        d[6] = 20;
    state->em.gps_vel_mode = NULL; //gps_mode[d[6]];

    if(d[7] > 20)
        d[7] = 20;
    state->em.gps_ori_mode = NULL; //   gps_mode[d[7]];

    return 1;
}
*/
int parse_em3000(unsigned char *d, state_t *state, int64_t timestamp)
{
    // decode an em3000 packet from the Falkor

    if(d[0] != 0x90)
        return 0;

    // check the checksum
    //char checksum = 0;
    //for(int i=1; i<22; i++)
    //checksum += d[i];

    //printf("Checksum = 0x%X, 0x%X\n", checksum & 0xFF, d[22]);
    //if((checksum & 0xFF) != d[22])
    //{
    //    fprintf(stderr, "Checksum error\n");
    //    return 0;
    //}

    //state->.utime = timestamp;

    //state->em.data_time = (int)get_short(&d[1]);
    //state->em.header = &d[1];
    state->em.r = (float)(get_16b_int(&d[2])) * 1e-2;
    state->em.p = (float)(get_16b_int(&d[4])) * 1e-2;
    state->em.heave = (float)(get_16b_int(&d[6])) * 1e-2;

    state->em.h = (float)(get_16b_int_unsin(&d[8])) * 1e-2;


    //if(d[62] == 0)
    //    parse_ncom_channel_0(&d[63], state);
    //if(state->count == 9)
    //{
    senlcm_simrad_em3000_format6_t_publish(state->lcm, "SIMRAD_EM3000_FORMAT6", &state->em);
    state->count = 0;
    //}
    //else
    //    state->count++;

    //return 1;
}


int program_exit;
void
signal_handler(int sigNum)
{
    // do a safe exit
    program_exit = 1;
}

int main (int argc, char *argv[])
{

    state_t state;
    state.count = 0;

    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);

    // read the config file
    BotParam *param;
    char rootkey[64];
    char key[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    state.lcm = lcm_create(NULL);
    param = bot_param_new_from_server (state.lcm, 1);

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

    // open the port
    struct addrinfo hints, *batt_addr;
    int fd;
    if(io == io_serial)
    {
        fd = serial_open(serial_dev, serial_translate_speed(baud), serial_translate_parity(parity), 1);
        if(fd < 0)
        {
            printf("Error opening port %s\n", serial_dev);
            return 0;
        }
        serial_set_noncanonical(fd, 1, 0);
    }
    else if(io == io_socket)
    {
        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        getaddrinfo(ip, inet_port, &hints, &batt_addr);
        fd = socket(batt_addr->ai_family, batt_addr->ai_socktype, batt_addr->ai_protocol);
        if(connect(fd, batt_addr->ai_addr, batt_addr->ai_addrlen) < 0)
        {
            printf("Could not connect to %s on port %s\n", ip, inet_port);
            return 1;
        }
        struct timeval tv;
        tv.tv_sec = 1;  // 1 Secs Timeout
        tv.tv_usec = 0000;  // Not init'ing this can cause strange errors
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval));

    }
    int bytes_read;
    unsigned char buf[128];
    fd_set rfds;
    while(!program_exit)
    {
        FD_ZERO (&rfds);
        FD_SET (fd, &rfds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        memset(buf, 0, 128);
        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &timeout);
        if(ret > 0)
        {
            int64_t timestamp = timestamp_now();
            read(fd, buf, 1);
            if(buf[0] == 0x90)
            {
                bytes_read = 1;
                while(bytes_read < 10)
                    bytes_read += read(fd, &buf[bytes_read], 10 - bytes_read);
                printf("Bytes read = %d\n", bytes_read);
                parse_em3000(buf, &state, timestamp);
            }
        }
    }

    return 0;
}

