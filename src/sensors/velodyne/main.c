#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <math.h>
#include <assert.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "perls-common/bot_util.h"
#include "perls-common/getopt.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"
#include "perls-common/timeutil.h"

#include "perls-sensors/velodyne.h"

#include "perls-lcmtypes/senlcm_velodyne_t.h"

#define UDP_MAX_LEN 1600
#define REPORT_INTERVAL_USECS (1.0 * 1e6)
#define READ_TIMEOUT_USEC (1.0 * 1e6)

// define state structure
typedef struct _state_t state_t;
struct _state_t
{
    int done;
    int is_daemon;
    lcm_t *lcm;

    char *lcm_chan;
    char *velodyne_model;

    timestamp_sync_state_t *tss_data;
    timestamp_sync_state_t *tss_position;
};

//Init the state structure
state_t state = {0};

//----------------------------------------------------------------------------------
// Called when program shuts down
//----------------------------------------------------------------------------------
static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("\nmy_signal_handler()\n");
    if (state.done)
    {
        printf ("Goodbye\n");
        exit (EXIT_FAILURE);
    }
    else
        state.done = 1;
}


uint32_t fread_le32(FILE *f)
{
    uint8_t b[4];
    int ret = fread (b, 4, 1, f);
    if (ret > 0)
    {
        return b[0] + (b[1]<<8) + (b[2]<<16) + (b[3]<<24);
    }
    else
    {
        return 0;
    }
}

// make and bind a udp socket to an ephemeral port
static int make_udp_socket(int port)
{
    int sock = socket (PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
        return -1;

    struct sockaddr_in listen_addr;
    memset (&listen_addr, 0, sizeof (struct sockaddr_in));
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons (port);
    listen_addr.sin_addr.s_addr = 0; //INADDR_ANY;

    int res = bind(sock, (struct sockaddr*) &listen_addr, sizeof (struct sockaddr_in));
    if (res < 0)
        return -2;

    return sock;
}

/*
// Read an ethereal-formatted log file and play the data
//    as velodyne_t messages
int play_log_file(const char *pathname)
{
    FILE *f = fopen(pathname, "r");

    if (f == NULL) {
        perror(pathname);
        return -1;
    }

    char hdr[24];
    fread(hdr, 1, 24, f);

    int64_t utime_offset = 0;

    lcm_t *lc = lcm_create(NULL);

    uint8_t *packet = malloc(2048);

    while (1) {
        int64_t utime;
        uint32_t secs = fread_le32(f);
        uint32_t usecs = fread_le32(f);

        utime = secs;
        utime *= 1000000;
        utime += usecs;

        if (utime_offset == 0)
            utime_offset = timestamp_now() - utime;

        uint32_t packet_len = fread_le32(f);
        uint32_t capture_len = fread_le32(f);
        (void) packet_len;

        if (fread(packet, 1, capture_len, f) == 0)
            break;

        senlcm_velodyne_t v;
        v.utime = utime + utime_offset;
        v.datalen = capture_len - 42;
        v.data = &packet[42];

        int64_t delay = v.utime - timestamp_now();

        if (delay > 0)
            usleep(delay);

        senlcm_velodyne_t_publish(lc, "VELODYNE", &v);
    }

    return 0;
}
*/

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    BotParam * param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param)
    {
        ERROR ("Could not get configuration parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
        exit (EXIT_FAILURE);
    }

    // read the config file
    state.velodyne_model = VELODYNE_HDL_32E_MODEL_STR;
    bot_param_get_str (param, "sensors.velodyne.model", &state.velodyne_model);
    state.lcm_chan = "VELODYNE";
    bot_param_get_str (param, "sensors.velodyne.channel", &state.lcm_chan);

    // options
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, "Velodyne Driver");
    getopt_add_bool    (gopt, 'h', "help",    0,  "Show this");
    getopt_add_bool    (gopt, 'D', "daemon",  0,  "Run as daemon?");
    getopt_add_string  (gopt, 'c', "channel", state.lcm_chan, "LCM Channel");
    getopt_add_string  (gopt, 'm', "model",   state.velodyne_model, "Velodyne Model (HDL_32E, HDL_64E)");


    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len!=0)
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help"))
    {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    state.lcm_chan = strdup (getopt_get_string (gopt, "channel"));
    state.velodyne_model = strdup (getopt_get_string (gopt, "model"));

    // daemon mode?
    state.is_daemon = getopt_get_bool (gopt, "daemon");
    if (state.is_daemon)
        daemon_fork ();

    lcm_t *lc = lcm_create (NULL);

    int data_fd = make_udp_socket (VELODYNE_DATA_PORT);
    if (data_fd < 0)
    {
        printf ("Couldn't open data socket UDP socket\n");
        return EXIT_FAILURE;
    }

    int position_fd = 0;
    if (0==strcmp (VELODYNE_HDL_32E_MODEL_STR, state.velodyne_model))
    {
        position_fd = make_udp_socket (VELODYNE_POSITION_PORT);
        if (position_fd < 0)
        {
            printf ("Couldn't open position socket UDP socket\n");
            return EXIT_FAILURE;
        }
    }

    int data_packet_count = 0;
    int64_t data_last_report_time = timestamp_now ();
    double data_hz = 0.0;

    int position_packet_count = 0;
    int64_t position_last_report_time = timestamp_now ();
    double position_hz = 0.0;

    fd_set set;
    struct timeval tv;

    // packet timestamps give us the microseconds since top of the hour
    // param dev_ticks_per_second = 1e6   The nominal rate at which the device time increments
    // param dev_ticks_wraparound = 3.6e9 Assume that dev_ticks wraps around every wraparound ticks
    // param rate = based on 5sec/day drift rate  An upper bound on the rate error. Should be (1 + eps)
    state.tss_data = timestamp_sync_init (1e6, 3.6e9, 1.00006);
    state.tss_position = timestamp_sync_init (1e6, 3.6e9, 1.00006);

    printf ("\n");
    while (!state.done)
    {

        FD_ZERO (&set);
        FD_SET (data_fd, &set);
        if (position_fd)
            FD_SET (position_fd, &set);

        tv.tv_sec = 0;
        tv.tv_usec = READ_TIMEOUT_USEC;

        int ret = select (data_fd+position_fd+1, &set, NULL, NULL, &tv);
        if (ret < 0)
            ERROR ("ERROR: select()");
        else if (ret == 0) // Timeout
            printf ("Timeout: no data to read. \n");
        else   // We have data.
        {

            uint8_t buf[UDP_MAX_LEN];
            struct sockaddr_in from_addr;
            socklen_t from_addr_len = sizeof (from_addr);
            ssize_t len = 0;
            uint8_t packet_type = 0;

            if (FD_ISSET (data_fd, &set))
            {
                len = recvfrom (data_fd, (void*)buf, UDP_MAX_LEN, 0,
                                (struct sockaddr*) &from_addr, &from_addr_len);
                if (len != VELODYNE_DATA_PACKET_LEN)
                {
                    ERROR ("\nERROR: Bad data packet len, expected %d, got %d", VELODYNE_DATA_PACKET_LEN, (int) len);
                    continue;
                }
                packet_type = SENLCM_VELODYNE_T_TYPE_DATA_PACKET;
                data_packet_count++;
            }
            else if (FD_ISSET (position_fd, &set))
            {
                len = recvfrom (position_fd, (void*)buf, UDP_MAX_LEN, 0,
                                (struct sockaddr*) &from_addr, &from_addr_len);
                if (len != VELODYNE_POSITION_PACKET_LEN)
                {
                    ERROR ("\nERROR: Bad position packet len, expected %d, got %d", VELODYNE_POSITION_PACKET_LEN, (int) len);
                    continue;
                }
                packet_type = SENLCM_VELODYNE_T_TYPE_POSITION_PACKET;
                position_packet_count++;
            }
            else
                ERROR ("ERROR: What?");

            // pull the usec since top of the hour and use for timestamp sync
            // timestamp data is 4 bytes in reverse order
            uint32_t cycle_usec;
            if (packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET)
                cycle_usec = VELODYNE_GET_TIMESTAMP_USEC (buf);
            else if (packet_type == SENLCM_VELODYNE_T_TYPE_POSITION_PACKET)
                cycle_usec = VELODYNE_GET_TIMESTAMP_USEC (buf);

            senlcm_velodyne_t v;
            if (packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET)
                v.utime = timestamp_sync (state.tss_data, cycle_usec, timestamp_now ());
            else if (packet_type == SENLCM_VELODYNE_T_TYPE_POSITION_PACKET)
                v.utime = timestamp_sync (state.tss_position, cycle_usec, timestamp_now ());
            v.packet_type = packet_type;
            v.datalen = len;
            v.data = buf;

            senlcm_velodyne_t_publish (lc, state.lcm_chan, &v);

            if (!state.is_daemon)
            {

                double elapsed_time = 0.0;
                if (packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET)
                {
                    elapsed_time = (v.utime - data_last_report_time) / 1e6;
                    data_last_report_time = v.utime;
                    data_hz = (1.0) / elapsed_time;
                }
                else if (packet_type == SENLCM_VELODYNE_T_TYPE_POSITION_PACKET)
                {
                    elapsed_time = (v.utime - position_last_report_time) / 1e6;
                    position_last_report_time = v.utime;
                    position_hz = (1.0) / elapsed_time;
                }

                if (!state.is_daemon && !(data_packet_count % 1000))
                {
                    printf ("Data: count=%d rate=%0.2lf Hz. Position: count=%d rate=%0.2lf Hz. \r",
                            data_packet_count, data_hz,
                            position_packet_count, position_hz);
                }
            }
        }
    }

    timestamp_sync_free (state.tss_data);
    timestamp_sync_free (state.tss_position);
}
