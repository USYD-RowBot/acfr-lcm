#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <libgen.h>
#include <math.h>
#include <errno.h>
#include <string.h>



#include <bot_param/param_client.h>
#include "acfr-common/timestamp.h"
#include "perls-lcmtypes/acfrlcm_auv_acfr_nav_t.h"
#include "perls-lcmtypes/senlcm_novatel_t.h"


#define SELECT_TIMEOUT 10000        // usec block timeout on select for LCM

// acfr_nav_translate:
// Subscribes to the senlcm_novatel_t LCM message, 
// translates it into an acfrlcm_auv_acfr_nav_t message, 
// and transmits that via LCM
// Used to allow the WAMV ASV to directly use GPS for localization 
// where the AUVs require an additional nav filtering layer to combine various nav sources

typedef struct
{
    lcm_t *lcm;
    char root_key[64];
	char *vehicle_name;
	char ch_in_novatel[128];
	char ch_out_nav[128];

} state_t;


// globals for signalling exit
int program_exit;


void signal_handler(int sig_num)
{
    // do a safe exit
    if(sig_num == SIGINT)
    program_exit = 1;
}


void incoming_msg_handler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_novatel_t *msgin, void *u)
{
    state_t *state = (state_t *)u;

    // TODO Translate


    acfrlcm_asv_torqeedo_motor_status_t_publish(state->lcm, ch_out_nav, &state.//TODO);    
}


int main(int argc, char **argv)
{
    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);
    signal(SIGPIPE, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialise state which holds all current data
    state_t state;
	memset(&state.root_key, 0, 64);
    
    // Initialise LCM
    state.lcm = lcm_create(NULL);


    // Get vehicle name from command line arg
    char opt;
    int got_key = 0;
    while((opt = getopt(argc, argv, "hk:n:")) != -1)
    {
        if(opt == 'h')
        {
            fprintf(stderr, "Usage: acfr-nav-translate -n <vehicle name>\n");
            fprintf(stderr, " e.g.: acfr-nav-translate -n WAMV\n");
            return 0;
        }
        if(opt == 'n')
        {
            int n = strlen((char *)optarg);
            state.vehicle_name = malloc(n);
            strcpy(state.vehicle_name, (char *)optarg);
            printf("Vehicle: %s\n", state.vehicle_name);
        }
    }
	
	// Set channel names with vehicle name
	snprintf(ch_in_novatel, 128, "%s.NOVATEL", vehicle_name);
	snprintf(ch_out_nav, 128, "%s.ACFR_NAV", vehicle_name);

	// Get the root key from the command line, program name
	sprintf(state.root_key, "%s", basename(argv[0]));

	// Read the config file
    char key[64]; // temp to hold keys for lookup
    BotParam *param = bot_param_new_from_server (state.lcm, 1);
    if(param == NULL)
    {
        fprintf(stderr, "NavTranslate: can't access bot-param config\n");
        return 0;
    }

	// Read the config file values 
    sprintf(key, "%s.verbose", root_key);
    printf("Requesting Parameter: %s\n", key);
    state.verbose = bot_param_get_boolean_or_fail(param, key);

    sprintf(key, "%s.origin_lat", root_key);
    printf("Requesting Parameter: %s\n", key);
    state.origin_lat = bot_param_get_TODO_or_fail(param, key);

	printf(key, "%s.origin_long", root_key);
    printf("Requesting Parameter: %s\n", key);
    state.origin_long = bot_param_get_TODO_or_fail(param, key);
	

	// Subscribe to the LCM channels for incoming messages
    senlcm_novatel_t_subscribe(state.lcm, ch_in_novatel, &incoming_msg_handler, &state);

	// Setup select for just the LCM msgs
    fd_set rfds, dup_rfds; // list of file descriptors where we will listen for incoming messages and duplicate set for use
    int lcm_fd = lcm_get_fileno(state.lcm); // get the file descriptor id for the lcm messager
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(lcm_fd, &rfds); // add the lcm file descriptor to the set to watch

    // Main program loop
    while(!program_exit)
    {
        dup_rfds = rfds; // reset file descriptors
        tv.tv_sec = 0;
        tv.tv_usec = SELECT_TIMEOUT;
        
		// check incoming message sources
        ret = select (FD_SETSIZE, &dup_rfds, NULL, NULL, &tv);
        if(ret == -1)
        {
            fprintf(stderr, "NavTranslate: Select failure: %i", errno);
        }
        else if(ret != 0) // check incoming message
        {
			if(FD_ISSET(lcm_fd, &dup_rfds)) // LCM message, call the handler
            {
                lcm_handle(state.lcm);
            }
		}

    } // end main loop

    // close LCM commections
    lcm_destroy(state.lcm);
    return 0;

} // end main









