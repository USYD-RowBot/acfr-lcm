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

	// TODO Lat, Long ...?

	// Subscribe to the LCM channels for incoming messages
    senlcm_novatel_t_subscribe(state.lcm, ch_in_novatel, &incoming_msg_handler, &state);

	
	







}









