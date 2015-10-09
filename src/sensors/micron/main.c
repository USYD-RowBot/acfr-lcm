#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <libgen.h>
#include <bot_param/param_client.h>

#include "perls-common/serial.h"
#include "perls-common/units.h"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes/senlcm_micron_ping_t.h"

typedef enum
{
    angle_low,
    angle_medium,
    angle_high,
    angle_ultimate
} angle_size_t;

typedef struct
{
    lcm_t *lcm;
    int run;
    double left_limit;
    double right_limit;
    double initial_gain;
    angle_size_t angle_resolution;
    double range;
    double range_resolution;
    double ad_span;
    double ad_low;
    int64_t timestamp;
} state_t;

#define MAX_BUF_LEN 512

// Fill in the header structure of the commands for the sonar head
int compose_command_header(unsigned char *d, unsigned char tx, unsigned char rx, short length, short message_length, unsigned char message_type)
{
    char cb[11];

    memset(cb, 0, sizeof(cb));
    cb[0] = '@';                            // Header
    snprintf(&cb[1], 5, "%04X", length);    // ASCII hex length
    memcpy(&cb[5], &length, 2);             // Binary length
    cb[7] = tx;                             // TX node
    cb[8] = rx;                             // RX node
    cb[9] = message_length;                 // Message byte count
    cb[10] = message_type;                  // Message type
    cb[11] = 0x80;                           // Sequence end
    memcpy(d, cb, 12);

    // return the number of bytes
    return 12;
}

// Sonar head command
int compose_head_command(unsigned char *d, state_t *state)
{
    unsigned char c[66];
    memset(c, 0, 66);

    // Fill in the header
    compose_command_header(c, 0xff, 0x02, 60, 56, 19);
    c[12] = 2;
    c[13] = 1;


    // Head control bits
    c[15] = 0x23;
    c[14] = 0x01;

    // Head type = DST
    c[16] = 11;

    // range, not actually used by the sonar head, just there for a reference in the
    // returned data
    unsigned short r = (unsigned short)(state->range * 10);
    memcpy(&c[35], &r, 2);

    // convert the limit angles from degrees to 1/16 gradians
    // 0 is astern, 3200 is ahead, 1600 is left 90 degrees, 4800 is right 90 degrees
    short left_limit_grad16 = (short)(state->left_limit / 360.0 * 400.0) * 16;
    short right_limit_grad16 = (short)(state->right_limit / 360.0 * 400.0) * 16;
    memcpy(&c[37], &left_limit_grad16, 2);
    memcpy(&c[39], &right_limit_grad16, 2);

    printf("angles %u, %u\n", left_limit_grad16, right_limit_grad16);

    // AD span and lower limit, input values are in dB
    c[41] = (unsigned char)(state->ad_span / 80.0 * 255.0);
    c[42] = (unsigned char)(state->ad_low / 80.0 * 255.0);


    // the initial gain is 0 - 80dB, translates to 0 - 210
    c[43] = (unsigned char)(state->initial_gain / 80.0 * 210.0);
    c[44] = c[43];
    printf("Initial gain value = %u\n", c[43]);

    // high speed limit of the scanning motor, 10 micosecond units
    c[49] = 25;

    // step angle size, 0.225, 0.45, 0.9, 1.8 degrees
    switch(state->angle_resolution)
    {
    case angle_low:
        c[50] = 32;
        break;
    case angle_medium:
        c[50] = 16;
        break;
    case angle_high:
        c[50] = 8;
        break;
    case angle_ultimate:
        c[50] = 4;
        break;
    }

    printf("Step size = %u\n", c[50]);

    // Calculate the AD interval and the number of bins given the desired range and resolution
    // Using a VOS of 1500 m/s

    double sample_time = 2 * state->range / 1500;      // two times the time of flight
    short num_bins = (int)(state->range / state->range_resolution);
    if(num_bins > 1500)                         // upper limit for a DST sonar
        num_bins = 1500;

    // the AD interval is in units of 640ns
    unsigned short AD_interval = (unsigned short)((sample_time / (double)num_bins) / 640e-9);

    memcpy(&c[51], &AD_interval, 2);
    memcpy(&c[53], &num_bins, 2);

    printf("Setting AD interval to %d with %d bins\n", AD_interval, num_bins);

    // set some default parameters
    unsigned short t;
    t = 1000;
    memcpy(&c[55], &t, 2);                   // Max AD buffer
    t = 100;
    memcpy(&c[57], &t, 2);                   // Lockout
    t = 1600;
    memcpy(&c[59], &t, 2);                   // Minor axis direction
    c[61] = 1;                              // Major axis direction

    c[65] = 0x0A;

    memcpy(d, c, 66);

    return 66;

}

int compose_send_data_command(unsigned char *d)
{
    unsigned char c[18];
    memset(c, 0, 18);
    int len;
    len = compose_command_header(c, 0xff, 0x02, 12, 7, 25);
    c[len - 1] = 0x02;

    // for now we will leave the time flag set to zero as it is only 32 bits
    // we can do something tricky later

    c[17] = 0x0A;

    memcpy(d, c, 18);

    return 18;
}


int compose_version_command(unsigned char *d)
{
    unsigned char c[14];
    int len;
    len = compose_command_header(c, 0xff, 0x02, 8, 3, 23);
    c[len - 1] = 0x02;
    c[len] = 0x0A;

    memcpy(d, c, 14);

    return 14;
}

// decode a seanet packet
int decode_seanet(unsigned char *d, int len, state_t *state)
{
    int message_type = d[10];
//    int tx_node = d[7];
//    int rx_node = d[8];
    int ret = 0;

    switch(message_type)
    {
    case 1:
    {
        // Version
        unsigned int cpu_id = *(unsigned int *)&d[13];
        unsigned int program_length = *(unsigned int *)&d[17];
        unsigned short checksum = *(unsigned short *)&d[21];

        printf("CPU ID = 0x%08X, program length = %u, checksum = 0x%04X\n", cpu_id, program_length, checksum);

        break;
    }
    case 19:        // Head command response
        break;

    case 2:         // Scanline data reply
    {
        senlcm_micron_ping_t p;
        p.utime = state->timestamp;
        p.range = (double)(*(unsigned short *)&d[20]) / 10.0;
        p.range_resolution = state->range_resolution;
        p.gain = (double)d[26] / 210.0;
        p.slope = (double)(*(unsigned short *)&d[27]) / 255.0;
        p.AD_interval = (double)(*(unsigned short *)&d[33]);
        p.left_limit = ((double)(*(unsigned short *)&d[35]) / 16.0) / 400.0 * 2 * M_PI;
        p.right_limit = ((double)(*(unsigned short *)&d[37]) / 16.0) / 400.0 * 2 * M_PI;
        p.angle = ((double)(*(unsigned short *)&d[40]) / 16.0) / 400.0 * 2 * M_PI;
        p.num_bins = (int)(*(unsigned short *)&d[42]);
        p.bins = (unsigned char *)malloc(p.num_bins);
        memcpy(p.bins, &d[45], p.num_bins);

        // publish the data
        senlcm_micron_ping_t_publish(state->lcm, "MICRON", &p);
        free(p.bins);

        ret = 1;
        break;
    }
    case 4:         // Alive message
    {
        printf("Alive Msg, 0x%02X\n", d[20] & 0xFF);
        break;
    }
    default:        // Not decoded
        printf("Got a unknown message type %d\n", message_type);
        break;
    }

    return ret;
}

int program_exit;
void
signal_handler(int sigNum)
{
    // do a safe exit
    program_exit = 1;
}


int main(int argc, char **argv)
{
    state_t state;

    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);

    BotParam *params;
    char rootkey[64];
    char key[64];

    state.lcm = lcm_create(NULL);

    params = bot_param_new_from_server (state.lcm, 1);
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    // read the initial config
    sprintf(key, "%s.device", rootkey);
    char *device = bot_param_get_str_or_fail(params, key);

    sprintf(key, "%s.baud", rootkey);
    int baud = bot_param_get_int_or_fail(params, key);

    sprintf(key, "%s.parity", rootkey);
    char *parity = bot_param_get_str_or_fail(params, key);

    sprintf(key, "%s.left_limit", rootkey);
    state.left_limit = bot_param_get_double_or_fail(params, key);

    sprintf(key, "%s.right_limit", rootkey);
    state.right_limit = bot_param_get_double_or_fail(params, key);

    sprintf(key, "%s.initial_gain", rootkey);
    state.initial_gain = bot_param_get_double_or_fail(params, key);

    sprintf(key, "%s.range", rootkey);
    state.range = bot_param_get_double_or_fail(params, key);

    sprintf(key, "%s.range_resolution", rootkey);
    state.range_resolution = bot_param_get_double_or_fail(params, key);

    sprintf(key, "%s.angle_resolution", rootkey);
    char *angle_resolution = bot_param_get_str_or_fail(params, key);
    if(strstr(angle_resolution, "low") != NULL)
        state.angle_resolution = angle_low;
    else if(strstr(angle_resolution, "medium") != NULL)
        state.angle_resolution = angle_medium;
    else if(strstr(angle_resolution, "high") != NULL)
        state.angle_resolution = angle_high;
    else if(strstr(angle_resolution, "ultimate") != NULL)
        state.angle_resolution = angle_ultimate;
    else
    {
        printf("angle resolution must be one of the following: low, medium, high, ultimate\n");
        return 1;
    }

    sprintf(key, "%s.ad_span", rootkey);
    state.ad_span = bot_param_get_double_or_fail(params, key);

    sprintf(key, "%s.ad_low", rootkey);
    state.ad_low = bot_param_get_double_or_fail(params, key);

    // Open the serial port
    int serial_fd = serial_open(device, serial_translate_speed(baud), serial_translate_parity(parity), 1);
    serial_set_noncanonical(serial_fd, 1, 0);

    // lets send the sonar a head command to put it in the right mode
    // we will resend this command everytime a config variable changes
    unsigned char d[128];
    int bytes;
    bytes = compose_version_command(d);
    write(serial_fd, d, bytes);

    bytes = compose_head_command(d, &state);
    int written = write(serial_fd, d, bytes);
    printf("Wrote %d bytes\n", written);

    for(int i=0; i<bytes; i++)
        printf("0x%02X ", d[i] & 0xFF);
    printf("\n");
    usleep(1000);
    bytes = compose_send_data_command(d);
    write(serial_fd, d, bytes);

    int return_count = 0;

    fd_set rfds;
    unsigned char buf[MAX_BUF_LEN];
    int64_t timestamp;
    int lcm_fd = lcm_get_fileno(state.lcm);

    // now enter the main loop
    while(!program_exit)
    {
        FD_ZERO(&rfds);
        FD_SET(lcm_fd, &rfds);
        FD_SET(serial_fd, &rfds);

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
            else
            {
                state.timestamp = timestamp_now();
                bytes = 0;
                do
                {
                    bytes += read(serial_fd, &buf[bytes], 7 - bytes);
                }
                while(bytes < 7);

                if(bytes == 7)
                {
                    unsigned short data_len = *(unsigned short *)&buf[5];
                    // read the rest
                    do
                    {
                        bytes += read(serial_fd, &buf[bytes], (data_len + 7 - 1) - bytes);
                    }
                    while(bytes < (data_len + 7 - 1));
                }

                ret = decode_seanet(buf, bytes, &state);
                // we need to send the data command for every second return data packet
                if(ret == 1)
                {
                    if(return_count++ == 1)
                    {
                        bytes = compose_send_data_command(d);
                        write(serial_fd, d, bytes);
                        return_count = 0;
                    }


                }
            }
        }
    }
    close(serial_fd);

    return 0;
}

