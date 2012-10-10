#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <arpa/inet.h>

// external linking req'd
#include <gsl/gsl_math.h>

#include "perls-lcmtypes/senlcm_mocap_t.h"

#include "perls-common/bot_util.h"
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"


// specifically for type-casting the tcp data
typedef struct body_pose body_pose_t;
struct body_pose
{
    float x;
    float y;
    float z;
    float r;
    float p;
    float h;
    float residual;
};

typedef struct _state_t state_t;
struct _state_t
{
    int done;
    int is_daemon;
    int sock;
    lcm_t *lcm;

    int packet_size;
    int n_bodies;
    body_pose_t *bodies;
};

typedef struct _config_t config_t;
struct _config_t
{
    int server_port;
    char server_ip[16];

    char mocap_channel_prefix[256];
    int mocap_num_bodies;
    char **mocap_bodies;
};

// Init state structure
state_t state = {0};


//----------------------------------------------------------------------
// Loads the required info from the .cfg file into the state
//----------------------------------------------------------------------
void
mocap_load_cfg (config_t *config)
{
    BotParam *param = bot_param_new_from_file (BOTU_PARAM_DEFAULT_CFG);
    if (!param) {
        ERROR ("Could not create configuration parameters from file %s", BOTU_PARAM_DEFAULT_CFG);
        exit (EXIT_FAILURE);
    }

    // server information
    strcpy (config->server_ip, bot_param_get_str_or_fail (param, "mocap.server_ip"));
    config->server_port = bot_param_get_int_or_fail (param, "mocap.server_port");

    // lcm channel prefix
    strcpy (config->mocap_channel_prefix, bot_param_get_str_or_fail (param, "mocap.mocap_channel_prefix"));

    // mocap body names
    config->mocap_num_bodies = bot_param_get_int_or_fail (param, "mocap.mocap_num_bodies");
    config->mocap_bodies = bot_param_get_str_array_alloc (param, "mocap.mocap_bodies");
}

//----------------------------------------------------------------------------------
// Called when program shuts down 
//----------------------------------------------------------------------------------
static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    printf ("\nmy_signal_handler()\n");
    if (state.done) {
        printf ("Goodbye\n");
        exit (EXIT_FAILURE);
    }
    else
        state.done = 1;
}

int
connect_to_server (const char* server_ipaddr, int server_port)
{
    int sock;
    if ((sock = socket (PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        PERROR ("socket() failed");
        return EXIT_FAILURE;
    }

    struct sockaddr_in servAddr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = server_ipaddr ? inet_addr (server_ipaddr) : inet_addr ("127.0.0.1"),
        .sin_port = server_port > 0 ? htons (server_port) : htons (1221),
    };

    if (connect (sock, (struct sockaddr *) &servAddr, sizeof (servAddr)) < 0) {
        PERROR ("connect() failed: is mocap running?");
        close (sock);
        return EXIT_FAILURE;
    }

    return sock;
}

//----------------------------------------------------------------------------------
// Reads on tcp connection until byte b is reached 
//----------------------------------------------------------------------------------
int
read_until_byte (char b)
{
    char buf[1024];
    while (1) {
        if (recv (state.sock, buf, 1, MSG_WAITALL) != 1)
            return -1;

        if (buf[0] == b)
            break;
    }

    return 1;
}

//----------------------------------------------------------------------------------
// Reads 2 byte packet size and stores in state
//----------------------------------------------------------------------------------
int
read_size (void)
{
    char buf[1024];
    if (recv (state.sock, buf, 2, MSG_WAITALL) != 2)
        return -1;

    state.packet_size = (int) buf[0];

    return 1;
}

//----------------------------------------------------------------------------------
// Reads packet minus initial 3 bytes
//----------------------------------------------------------------------------------
int
read_packet (void)
{
    int frame_size = state.packet_size - 3;
    char *total_data = malloc (frame_size);
    char buf[1024];
    int data_left = frame_size;

    // loop to catch data in 1024 byte blocks
    while (data_left > 0) {
        if (recv (state.sock, buf, GSL_MIN (data_left, 1024), MSG_WAITALL) != GSL_MIN (data_left, 1024))
            return -1;

        memcpy (total_data + (frame_size - data_left), buf, GSL_MIN (data_left, 1024));
        data_left = data_left - GSL_MIN (data_left, 1024);
    }

    if (state.n_bodies != ((int) total_data[5])) {
        if (state.bodies != 0)
            free (state.bodies);

        state.n_bodies = (int) total_data[5];
        state.bodies = malloc (sizeof (body_pose_t) * state.n_bodies);
    }

    int body = 0;
    for (body = 0; body < ((int) total_data[5]); body++)
        memcpy (state.bodies+body, &total_data[6+32*body], 28);


    free (total_data);

    return 1;
}

//----------------------------------------------------------------------------------
// publish the mocap state
//----------------------------------------------------------------------------------
void
mocap_publish (config_t *config)
{
    int64_t utime = timestamp_now ();
    
    for (int i=0; i<state.n_bodies; i++) {
        senlcm_mocap_t mocap = {
            .utime = utime,

            .xyzrph = {state.bodies[i].x/1000.0, // millimeters to meters
                       state.bodies[i].y/1000.0,
                       state.bodies[i].z/1000.0,
                       state.bodies[i].r,
                       state.bodies[i].p,
                       state.bodies[i].h},

            .residual = state.bodies[i].residual,

            .valid = (state.bodies[i].residual > 0) ? 1 : 0,
        };

        char channel[256];        
        if (i < config->mocap_num_bodies)
            snprintf (channel, sizeof channel, "%s%s", config->mocap_channel_prefix, config->mocap_bodies[i]);
        else
            snprintf (channel, sizeof channel, "%s%d", config->mocap_channel_prefix, i);

        senlcm_mocap_t_publish (state.lcm, channel, &mocap);
    }
}


//----------------------------------------------------------------------------------
// Main function
//----------------------------------------------------------------------------------
int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // install custom signal handler
    struct sigaction act = {
        .sa_sigaction = my_signal_handler,
    };
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT, &act, NULL);

    // init lcm
    state.lcm = lcm_create (NULL);
    if (!state.lcm) {
        ERROR ("lcm_create() failed");
        exit (EXIT_FAILURE);
    }

    // load in config file option
    config_t config = {0};
    mocap_load_cfg (&config);

    char request_cmd[] = "#M2001\r";
    char message_start_byte = 1;

    // Read in the command line options
    getopt_t *gopt = getopt_create ();
    char default_port[5];
    snprintf (default_port, 5, "%d", config.server_port);

    getopt_add_description (gopt, "MoCap Driver: Publishes LCM senlcm_mocap_t messages");
    getopt_add_string (gopt,    'i',    "ipaddr",   config.server_ip,   "MoCap server ip address");
    getopt_add_int    (gopt,    'p',    "port",     default_port,       "MoCap server port");
    getopt_add_bool   (gopt,    'D',    "daemon",   0,   	            "Run as system daemon");
    getopt_add_bool   (gopt,    'h',    "help",     0,             	    "Display Help");

    if (!getopt_parse (gopt, argc, argv, 1)) {
        getopt_do_usage (gopt,"");
        bot_param_str_array_free (config.mocap_bodies);
        return EXIT_FAILURE;
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt,"");
        bot_param_str_array_free (config.mocap_bodies);
        return EXIT_SUCCESS;
    }
 
    //start as daemon if asked
    if (getopt_get_bool (gopt, "daemon")) {
        daemon_fork ();
        state.is_daemon = 1;
    }
    else
        state.is_daemon = 0;

    if (getopt_has_flag (gopt, "ipaddr"))
        strcpy (config.server_ip, getopt_get_string (gopt, "ipaddr"));

    if (getopt_has_flag (gopt, "port"))
        config.server_port = getopt_get_int (gopt, "port");
  
    // connect to server
    if ((state.sock = connect_to_server (config.server_ip, config.server_port)) < 0) {
        bot_param_str_array_free (config.mocap_bodies);
        exit (EXIT_FAILURE);
    }

    // send server request
    if (send (state.sock, request_cmd, strlen (request_cmd), MSG_NOSIGNAL) != strlen (request_cmd)) {
        PERROR ("send request failure");
        bot_param_str_array_free (config.mocap_bodies);
        exit (EXIT_FAILURE);
    }

    int status = EXIT_SUCCESS;
    // loop and capture data
    while (!state.done) {
        // read until start of packet
        if (read_until_byte (message_start_byte) < 0 ) {
            PERROR ("recv error");
            status = EXIT_FAILURE;
            break;
        }

        // read frame size
        if (read_size () < 0) {
            PERROR ("recv error");
            status = EXIT_FAILURE;
            break;
        }

        // read rest of packet
        if (read_packet() < 0) {
            PERROR ("recv error");
            status = EXIT_FAILURE;
            break;
        }
        
        mocap_publish (&config);
    }

    // done, close socket
    close (state.sock);
    if (state.bodies != 0)
        free (state.bodies);
    bot_param_str_array_free (config.mocap_bodies);

    return status;
}
