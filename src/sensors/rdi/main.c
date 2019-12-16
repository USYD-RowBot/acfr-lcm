#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>
#include <lcm/lcm.h>

#include "perls-lcmtypes/senlcm_rdi_pd4_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd5_t.h"
#include "perls-lcmtypes/senlcm_rdi_pd0_t.h"
#include "perls-lcmtypes/senlcm_rdi_control_t.h"
#include "perls-lcmtypes/senlcm_raw_ascii_t.h"
#include "perls-lcmtypes/senlcm_raw_t.h"
#include "perls-lcmtypes/acfrlcm_auv_relay_t.h"

#include "acfr-common/units.h"
#include "acfr-common/error.h"
#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"


#include "rdi.h"

#define RDI_DEV_TICKS_PER_SECOND (100) //  1/tofp_hundredth
#define RDI_DEV_TICKS_WRAPAROUND (24*3600*100)

#define EXPECT_RESPONSE 1
#define NO_RESPONSE 0

enum {MODE_PD0, MODE_PD5, MODE_PD4};

typedef struct
{
    int pd5_count_max;
    int pd0_count_max;
    pthread_mutex_t count_lock;
    int mode;
    lcm_t *lcm;
    acfr_sensor_t *sensor;
    int programming;
    double range;
    char PD5_channel[128];
    char PD0_channel[128];
    char RAW_channel[128];
} state_t;

rdi_pd_mode_t rdi_pd_mode = RDI_PD5_MODE;
int rdi_pd_len = RDI_PD5_LEN;

int program_exit;

static int64_t
rdi_timestamp_sync (timestamp_sync_state_t *tss, int64_t tofp_hours, int64_t tofp_minute,
                    int64_t tofp_second, int64_t tofp_hundredth, int64_t host_utime)
{
    int64_t dev_ticks = (tofp_hours*3600 + tofp_minute*60 + tofp_second)*100 + tofp_hundredth;

    return timestamp_sync (tss, dev_ticks, host_utime);
}

int rdi_send_command(acfr_sensor_t *sensor, char *cmd, int er)
{
    acfr_sensor_write(sensor, cmd, strlen(cmd));
    char buf;
    int count = 0;

    if(er)
    {
        // wait to get a prompt
        do
        {
            acfr_sensor_read(sensor, &buf, 1);
            if(++count == 50)
            {
                return 0;
            }
        }
        while(buf != '>');

    }
    return 1;
}

static void
program_dvl(state_t * state) //const char *config)
{

    char buf[256];
    bool alive=0;

    tcsendbreak(state->sensor->fd, 0);
    usleep(2000000);

    acfr_sensor_canonical(state->sensor, '\r', '\n');

    int alive_tries = 0;
    while (!alive)
    {
        tcsendbreak(state->sensor->fd, 0);
        //gsd_write (gsd, "===\n", strlen ("===\n")); // software break
        alive = acfr_sensor_read_timeoutms (state->sensor, buf, 256, 10000);
        printf("*\n");
        if(++alive_tries > 20)
        {
            printf("Couldn't wake the DVL up\n");
            return;
        }
    }


    // wait to get a prompt
    do
    {
        acfr_sensor_read(state->sensor, buf, 1);

    }
    while(buf[0] != '>');

    // Convert the max depth in meters to the command in decimeters
    char max_depth_cmd[15];
    char low_gain_switch_altitude[8];
    sprintf( max_depth_cmd, "BX%05d\r", (int)state->range*10 );
    sprintf( low_gain_switch_altitude, "BI%03d\r", (int)1 ); // metres, 0 to 999, default 3
    printf( "Sending max depth command: %s\n", max_depth_cmd);

    if(state->mode == MODE_PD5)
    {
        printf("Programming PD5 mode\n");
        rdi_send_command(state->sensor, "BP001\r", EXPECT_RESPONSE); // Bottom tracking ping
        rdi_send_command(state->sensor, max_depth_cmd, EXPECT_RESPONSE); // max depth in decimetre
        rdi_send_command(state->sensor, low_gain_switch_altitude, EXPECT_RESPONSE); // range to change from low to high gain mode
        rdi_send_command(state->sensor, "WP00000\r", EXPECT_RESPONSE); // No water profiling
        rdi_send_command(state->sensor, "PD5\r", EXPECT_RESPONSE);
        rdi_send_command(state->sensor, "CF11110\r", EXPECT_RESPONSE);
        rdi_send_command(state->sensor, "CK\r", NO_RESPONSE); // Keep parameters on power cycle
        rdi_send_command(state->sensor, "CS\r", NO_RESPONSE);
    }
    else if(state->mode == MODE_PD4)
    {
        printf("Programming PD4 mode\n");
        rdi_send_command(state->sensor, "BP001\r", EXPECT_RESPONSE);
        rdi_send_command(state->sensor, max_depth_cmd, EXPECT_RESPONSE);
        rdi_send_command(state->sensor, low_gain_switch_altitude, EXPECT_RESPONSE); // range to change from low to high gain mode
        rdi_send_command(state->sensor, "WP00000\r", EXPECT_RESPONSE);
        rdi_send_command(state->sensor, "PD4\r", EXPECT_RESPONSE);
        rdi_send_command(state->sensor, "CF11110\r", EXPECT_RESPONSE);
        rdi_send_command(state->sensor, "CK\r", NO_RESPONSE);
        rdi_send_command(state->sensor, "CS\r", NO_RESPONSE);
    }
    else
    {
        printf("Programming PD0 mode\n");
        rdi_send_command(state->sensor, "WD 100 000 000\r", EXPECT_RESPONSE);
        rdi_send_command(state->sensor, "CF01110\r", EXPECT_RESPONSE);
        rdi_send_command(state->sensor, "WS0020\r", EXPECT_RESPONSE);
        rdi_send_command(state->sensor, "WN025\r", EXPECT_RESPONSE);
    }
    fflush(NULL);
}

/*
static int
myopts (generic_sensor_driver_t *gsd)
{
    getopt_add_description (gsd->gopt, "RDI Workhorse & Explorer DVL sensor driver.");
    return 0;
}
*/




int get_rdi_and_send(state_t *state, timestamp_sync_state_t *tss)
{
    // get an RDI response
    char buf[1024];
    int len;
    int64_t timestamp;

    // get the start
    do
    {
        acfr_sensor_read(state->sensor, buf, 1);
        timestamp = timestamp_now();
    }
    while (buf[0] != 0x7F && buf[0] != 0x7D);

    len = 0;
    while(len < 3)
        len += acfr_sensor_read(state->sensor, &buf[1+len], 3 - len);

    // find the length of the data to recv
    unsigned short data_len = *(unsigned short *)&buf[2];
    // get the rest
    len = 4;
    while(len < (data_len + 2))
        len += acfr_sensor_read(state->sensor, &buf[len], data_len + 2 - len);

    senlcm_raw_t raw_msg;

    raw_msg.utime = timestamp;
    raw_msg.length = len;
    raw_msg.data = (uint8_t *)buf;

    senlcm_raw_t_publish(state->lcm, state->RAW_channel, &raw_msg);

    if(buf[0] == RDI_PD0_HEADER)
    {
        rdi_pd0_t pd0;
        // we have a PD0 message
        if(rdi_parse_pd0(buf, len, &pd0))
        {
            senlcm_rdi_pd0_t lcm_pd0 = rdi_pd0_to_lcm_pd0(&pd0);
            lcm_pd0.utime = rdi_timestamp_sync (tss, pd0.variable.rtc_hour, pd0.variable.rtc_min, pd0.variable.rtc_sec,
                                                pd0.variable.rtc_hund, timestamp);
            senlcm_rdi_pd0_t_publish (state->lcm, state->PD0_channel, &lcm_pd0);
            //gsd_update_stats (gsd, true);
            free_rdi_pd0(&pd0);
        }
        else
        {
            fprintf(stderr, "Failed to parse PD0 message.\n");
        }
    }
    else if(buf[0] == RDI_PD45_HEADER)
    {
        // try to parse it
        switch (rdi_pd_mode)
        {
        case RDI_PD4_MODE:
        {
            rdi_pd4_t pd4;
            if (0 == rdi_parse_pd4 (buf, len, &pd4))
            {
                senlcm_rdi_pd4_t lcm_pd4 = rdi_pd4_to_lcm_pd4 (&pd4);
                lcm_pd4.utime = rdi_timestamp_sync (tss, pd4.tofp_hour, pd4.tofp_minute, pd4.tofp_second,
                                                    pd4.tofp_hundredth, timestamp);
                senlcm_rdi_pd4_t_publish (state->lcm, state->PD5_channel, &lcm_pd4);
            }
            else
            {
                printf("parse error, PD4\n");
            }
            break;
        }
        case RDI_PD5_MODE:
        {
            rdi_pd5_t pd5;
            if (0 == rdi_parse_pd5 (buf, len, &pd5))
            {
                senlcm_rdi_pd5_t lcm_pd5 = rdi_pd5_to_lcm_pd5 (&pd5);
                lcm_pd5.utime = lcm_pd5.pd4.utime = timestamp; //rdi_timestamp_sync (tss, pd5.tofp_hour, pd5.tofp_minute,
                //       pd5.tofp_second, pd5.tofp_hundredth, timestamp);
                senlcm_rdi_pd5_t_publish (state->lcm, state->PD5_channel, &lcm_pd5);
            }
            else
            {
                printf("parse error, PD5\n");
            }

            break;
        }
        default:
            printf("unsupported PD mode by driver");
            //gsd_update_stats (gsd, -1);
        } // switch
    } else {
        printf("Unknown header.\n");
    }


    return 1;
}

void mp_callback(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_raw_ascii_t *msg, void *u)
{
    state_t *state = (state_t *)u;
    int i_value;
    char *ptr;
    int ret;

    if(strstr(msg->msg, "RDI") != NULL)
    {
        // the message is for us
        pthread_mutex_lock(&state->count_lock);
        if((ptr = strstr(msg->msg, "pd0count=")) != NULL)
        {
            ptr += strlen("pd0count=");
            ret = sscanf(ptr, "%d", &i_value);
            if(ret == 1)
                state->pd0_count_max = i_value;
        }

        if((ptr = strstr(msg->msg, "pd5count=")) != NULL)
        {
            ptr += strlen("pd5count=");
            ret = sscanf(ptr, "%d", &i_value);
            if(ret == 1)
                state->pd5_count_max = i_value;
        }
        pthread_mutex_unlock(&state->count_lock);
    }
}


void relay_callback(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_relay_t *msg, void *u)
{
    state_t *state = (state_t *)u;

    if(!strcmp(msg->channel, "doppler"))
        if(msg->state == 1)
        {
            if(!state->programming)
            {
                state->programming = 1;
                printf("Reprogramming the DVL\n");
                sleep(2);
                program_dvl(state);
                state->programming = 0;
            }
        }

}

void rdi_control_callback(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_rdi_control_t *rc, void *u)
{
    printf("In RDI control callback\n");
    state_t *state = (state_t *)u;
    pthread_mutex_lock(&state->count_lock);
    switch(rc->command)
    {
    case SENLCM_RDI_CONTROL_T_PD0_COUNT:
        state->pd0_count_max = rc->i;
        break;
    case SENLCM_RDI_CONTROL_T_PD5_COUNT:
        state->pd5_count_max = rc->i;
        break;
    case SENLCM_RDI_CONTROL_T_RANGE:
        state->range = rc->d;
        break;
    }
    printf("new range recv, %f\n", state->range);
    pthread_mutex_unlock(&state->count_lock);
}



// Process LCM messages with callbacks
static void *
lcm_thread (void *context)
{
    //generic_sensor_driver_t *gsd = (generic_sensor_driver_t *) context;
    printf("LCM thread starting\n");
    state_t *state = (state_t *)context;
    while (!program_exit)
    {
        lcm_handle_timeout(state->lcm, 1000);
    }

    return 0;
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
parse_args (int argc, char **argv, char **vehicle_name)
{
    int opt;

    const char *default_name = "DEFAULT";
    *vehicle_name = malloc(strlen(default_name)+1);
    strcpy(*vehicle_name, default_name);

    int n;
    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            n = strlen((char *)optarg);
            free(*vehicle_name);
            *vehicle_name = malloc(n);
            strcpy(*vehicle_name, (char *)optarg);
            break;
         }
    }
}



void signal_handler(int sig_num)
{
    // do a safe exit
    program_exit = 1;
}





int
main (int argc, char *argv[])
{
    timestamp_sync_state_t *tss = timestamp_sync_init (RDI_DEV_TICKS_PER_SECOND, RDI_DEV_TICKS_WRAPAROUND, 1.01);

    state_t state;
    //state.gsd = gsd_create (argc, argv, NULL, &myopts);
    //gsd_launch (state.gsd);

    char *vehicle_name;
    parse_args(argc, argv, &vehicle_name);
    snprintf(state.PD5_channel, 128, "%s.RDI", vehicle_name);
    snprintf(state.PD0_channel, 128, "%s.RDI_PD0", vehicle_name);
    snprintf(state.RAW_channel, 128, "%s.RDI.RAW", vehicle_name);


    state.lcm = lcm_create(NULL);
    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));
    state.sensor = acfr_sensor_create(state.lcm, rootkey);
    if(state.sensor == NULL)
        return 0;

    pthread_mutex_init(&state.count_lock, NULL);
    int pd5_count = 0, pd0_count = 0;


    // listen for changes
    state.pd0_count_max = 0;
    state.pd5_count_max = 0;
    state.range = 25;


    char key[256];
    sprintf(key, "%s.mode", rootkey);
    char *key_str = bot_param_get_str_or_fail(state.sensor->param, key);
    if(!strcmp(key_str, "PD0"))
        state.mode = MODE_PD0;
    else if(!strcmp(key_str, "PD5"))
    {
        rdi_pd_mode = RDI_PD5_MODE;
        state.mode = MODE_PD5;
        rdi_pd_len = RDI_PD5_LEN;
    }
    else if(!strcmp(key_str, "PD4"))
    {
        rdi_pd_mode = RDI_PD4_MODE;
        state.mode = MODE_PD4;
        rdi_pd_len = RDI_PD4_LEN;
    }
    else
    {
        printf("Unknown mode %s in config file\n", key_str);
        return 0;
    }
    sprintf(key, "%s.pd5_count_max", rootkey);
    state.pd5_count_max = bot_param_get_int_or_fail(state.sensor->param, key);
    sprintf(key, "%s.pd0_count_max", rootkey);
    state.pd0_count_max = bot_param_get_int_or_fail(state.sensor->param, key);
    sprintf(key, "%s.range", rootkey);
    state.range = bot_param_get_double_or_fail(state.sensor->param, key);
    printf( "read max range from config file: %f [m]\n", state.range);

    // initialize dvl
    //gsd_flush (state.gsd);
    //gsd_reset_stats (state.gsd);
    program_dvl(&state);

    //gsd_noncanonical(state.gsd, 1024, 1);
    acfr_sensor_noncanonical(state.sensor, 1024, 1);

    //gsd_flush (state.gsd);
    //gsd_reset_stats (state.gsd);

    // create an LCM thread to listen so the command pass through will work
    pthread_t tid;
    pthread_create(&tid, NULL, lcm_thread, &state);
    pthread_detach(tid);

    senlcm_raw_ascii_t_subscribe(state.lcm, "MP_PASSOUT", &mp_callback, &state);
    acfrlcm_auv_relay_t_subscribe(state.lcm, "RELAY", &relay_callback, &state);
    senlcm_rdi_control_t_subscribe(state.lcm, "RDI_CONTROL", &rdi_control_callback, &state);

    double current_range = state.range;

    while (!program_exit)
    {
        // are we changing the range
        pthread_mutex_lock(&state.count_lock);
        if(abs(state.range - current_range) > 0.001)
        {
            char range_cmd[64];
            sprintf(range_cmd, "BX%05d\r", (int)(state.range*10));
            printf("Changing Range to %fm\n", state.range);
            rdi_send_command(state.sensor, range_cmd, EXPECT_RESPONSE); // max depth in decimetre
            rdi_send_command(state.sensor,"CS\r", NO_RESPONSE);         //trigger the ping
            current_range = state.range;
        }
        pthread_mutex_unlock(&state.count_lock);

        if(state.mode == MODE_PD0)
        {

            pthread_mutex_lock(&state.count_lock);
            // Are we doing PD5 now?
            if(pd5_count++ < state.pd5_count_max)
            {
                //Is this the first bottom ping in this sequence?
                if(pd5_count == 1)
                {
                    //Turn off water pings and on bottom pings
                    rdi_send_command(state.sensor, "BP1\r", EXPECT_RESPONSE);
                    rdi_send_command(state.sensor, "WP0\r", EXPECT_RESPONSE);
                    rdi_send_command(state.sensor, "PD5\r", EXPECT_RESPONSE);
                }

                rdi_send_command(state.sensor,"CS\r", NO_RESPONSE); //trigger the ping
                get_rdi_and_send(&state, tss);
            }
            else if(pd0_count++ < state.pd0_count_max)
            {

                if(pd0_count == 1)
                {
                    //Turn on water pings and off bottom pings
                    rdi_send_command(state.sensor,"BP0\r", EXPECT_RESPONSE);
                    rdi_send_command(state.sensor,"WP1\r", EXPECT_RESPONSE);
                    rdi_send_command(state.sensor,"PD0\r", EXPECT_RESPONSE);
                }

                rdi_send_command(state.sensor,"CS\r", NO_RESPONSE); //trigger the ping
                get_rdi_and_send(&state, tss);
            }
            else
            {
                pd0_count=0;
                pd5_count=0;
            }

            pthread_mutex_unlock(&state.count_lock);
        }
        else
            // we are in free running mode, just PD5 or PD4
            get_rdi_and_send(&state, tss);

    } // while

    timestamp_sync_free (tss);
    pthread_join(tid, NULL);

    return 0;
}
