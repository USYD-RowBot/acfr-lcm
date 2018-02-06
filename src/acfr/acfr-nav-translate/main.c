#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <libgen.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <proj_api.h>
#include <errno.h>
#include <unistd.h>
#include <stdbool.h>


#include <bot_param/param_client.h>
#include "acfr-common/timestamp.h"
#include "acfr-common/units.h"
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
    bool verbose;
	char *vehicle_name;
	char ch_in_novatel[128]; // channel names (inc. vehicle prefix)
	char ch_out_nav[128];
	double origin[2]; // lat lon, read in as degrees, stored as radians
	projPJ gps;
    projPJ tmerc;
} state_t;


// globals for signalling exit
int program_exit;


void signal_handler(int sig_num)
{
    // do a safe exit
    if(sig_num == SIGINT)
    program_exit = 1;
}

void incoming_msg_handler(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_novatel_t *msg_in, void *u)
{
    state_t *state = (state_t *)u;
    acfrlcm_auv_acfr_nav_t nav;
	memset(&nav, 0, sizeof(acfrlcm_auv_acfr_nav_t));

    // Translate
	nav.utime = msg_in->utime;      	// keep orig. value 

	nav.latitude = msg_in->latitude;	// In (radians)
	nav.longitude = msg_in->longitude;	// In (radians)
	
	// set to source F.O.R values, then translate to
	nav.x = msg_in->latitude;			// Northings relative to origin (m)
	nav.y = msg_in->longitude;			// Eastings relative to origin (m)
    if (state->verbose)
    {
        printf("Lat nav.x: %f Lon nav.y: %f \n", nav.x, nav.y);
    }
	int error_code = pj_transform(state->gps, state->tmerc, 1, 1, &nav.y, &nav.x, 0);  // src_def, dest_def, point_count, point_offset, x, y, z 
	if (error_code != 0)													 // Note: supposed to be NED, but actually END (y x z) x y z = N E D (z/D = null)
	{
		fprintf(stderr, "NavTranslate FATAL ERROR: can't perform Transverse Mercator Projection %d\n", error_code);
        exit(-1);
	}
    
	nav.depth =	0;	 					            // could be det. from height - tides? but not used 
    nav.roll = msg_in->roll;		            	// Right-Handed Body F.O.R. (radians)
    nav.pitch = msg_in->pitch;	 		            // Right-Handed Body F.O.R. (radians)
    nav.heading = fmod(msg_in->heading, (2 * M_PI));		// Right-Handed (+ down) Body F.O.R. (radians)
    
    double velocity_compass_angle = atan2(msg_in->east_velocity, msg_in->north_velocity); 
    if (errno == EDOM) // No velocity case
    {
        velocity_compass_angle = 0;
    }
    if (velocity_compass_angle < 0)
    {
        velocity_compass_angle += (2 * M_PI);
    }
	double velocity_magnitude = sqrt(msg_in->east_velocity * msg_in->east_velocity + msg_in->north_velocity * msg_in->north_velocity);
    double velocity_body_angle = velocity_compass_angle - nav.heading; 
  	
    nav.vx = velocity_magnitude * sin(velocity_body_angle);	// Body F.O.R. Forward velocity (m/s)
   	nav.vy = velocity_magnitude * cos(velocity_body_angle);	// Body F.O.R. Starboard velocity (m/s)
    
	nav.vz = -msg_in->up_velocity; 	    // not used for wam-v 2D planner  
	nav.rollRate = 0;					// not used for wam-v 2D planner 
    nav.pitchRate = 0;					// not used for wam-v 2D planner 
    nav.headingRate = 0;				// not used for wam-v 2D planner 
    nav.altitude = 0;					// not used for wam-v 2D planner 
    nav.fwd_obstacle_dist = 0;			// not yet implemented


    acfrlcm_auv_acfr_nav_t_publish(state->lcm, state->ch_out_nav, &nav);    
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
    state.verbose = false;
    
    // Initialise LCM
    state.lcm = lcm_create(NULL);


    // Get vehicle name from command line arg
    char opt;
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
	//snprintf(state.ch_in_novatel, 128, "%s.NOVATEL", state.vehicle_name);
	//temp hack as NOVATEL doesn't have prefix yet
	snprintf(state.ch_in_novatel, 128, "NOVATEL");
	snprintf(state.ch_out_nav, 128, "%s.ACFR_NAV", state.vehicle_name);
    if (state.verbose)
    {
        printf("Channel in: %s\n", state.ch_in_novatel);
        printf("Channel out: %s\n", state.ch_out_nav);
    }

	// Get the root key from the command line, program name
	//sprintf(state.root_key, "%s", basename(argv[0]));
    sprintf(state.root_key, "nav.acfr-nav-new"); // use std 

	// Read the config file
    char key[64]; // temp to hold keys for lookup
    BotParam *param = bot_param_new_from_server (state.lcm, 1);
    if(param == NULL)
    {
        fprintf(stderr, "NavTranslate: can't access bot-param config\n");
        return 0;
    }

	// Read the config file values 
    sprintf(key, "%s.verbose", state.root_key);
    printf("Requesting Parameter: %s\n", key);
    state.verbose = bot_param_get_boolean_or_fail(param, key);
	
	// And convert Lat and Long from degrees to radians
    sprintf(key, "%s.latitude", state.root_key);
    printf("Requesting Parameter: %s\n", key);
    state.origin[0] = DTOR * bot_param_get_double_or_fail(param, key);

	sprintf(key, "%s.longitude", state.root_key);
    printf("Requesting Parameter: %s\n", key);
    state.origin[1] = DTOR * bot_param_get_double_or_fail(param, key);
	
    if (state.verbose)
    {
        printf("Origin - Lat: %f Lon: %f \n", state.origin[0], state.origin[1]);
        printf("Origin - Lat: %f Lon: %f \n", (RTOD * state.origin[0]), (RTOD * state.origin[1]));
    }

	// Setup for Transverse Mercator Projection using proj.4 library 
    char proj_str[128];
    sprintf(proj_str, "+proj=tmerc +lat_0=%f +lon_0=%f +axis=ned +units=m", RTOD * state.origin[0], RTOD * state.origin[1]);
    state.gps = pj_init_plus("+proj=latlong +ellps=WGS84");
    state.tmerc = pj_init_plus(proj_str);
	if (state.tmerc == 0 || state.gps == 0)
	{
		fprintf(stderr, "NavTranslate FATAL ERROR: can't setup Transverse Mercator Projection\n");
		return -1;
	}

	// Subscribe to the LCM channels for incoming messages
    senlcm_novatel_t_subscribe(state.lcm, state.ch_in_novatel, &incoming_msg_handler, &state);

	// Setup select for just the LCM msgs
    fd_set rfds, dup_rfds; // list of file descriptors where we will listen for incoming messages and duplicate set for use
    int lcm_fd = lcm_get_fileno(state.lcm); // get the file descriptor id for the lcm messager
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(lcm_fd, &rfds); // add the lcm file descriptor to the set to watch
    int ret = 0;

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
            fprintf(stderr, "NavTranslate: Select failure: %i %s", errno, strerror(errno));
        }
        else if(ret != 0) // check incoming message
        {
			if(FD_ISSET(lcm_fd, &dup_rfds)) // LCM message, call the handler
            {
                lcm_handle(state.lcm);
            }
		}

    } // end main loop
    
    // free proj4 memory
    pj_free(state.gps);
    pj_free(state.tmerc);

    // close LCM commections
    lcm_destroy(state.lcm);
    return 0;

} // end main


