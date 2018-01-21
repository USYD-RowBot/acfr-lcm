/*
    Listens for an LCM relay command message and sends the corresponding request to the relay board via Ethernet. 
    On 1Hz HB (and after each successful change) supplies an status message with a bit to indicate the status of each relay/io port.
    Jorja Martin 01/2008
    ACFR
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <string.h>

#include <bot_param/param_client.h>
#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"
#include "acfr-common/lcm_util.h"
#include "perls-lcmtypes/acfrlcm_relay_command_t.h"
#include "perls-lcmtypes/acfrlcm_relay_status_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

// Contants
#define CMD_TIMEOUT 5000000
#define NUM_RELAYS 24                   // Number of relays on board
#define NUM_IOS 8                       // Number of IO ports on board
#define MAX_DEV_NAME_LEN 20             // Maximum length for an attached device name 
#define MAX_DELAY 2147483647            // Maximum auto relay off dealy in ms
#define MIN_DELAY 100                   // Minimum auto relay off delay in ms
#define ON_LEN 9                        // Message length for ASCII 'on' message
#define OFF_LEN 10                      // Message length for ASCII 'off' message
#define GET_LEN 7						// Message length for ASCII 'get status' message
#define ON_DELAY_LEN 20                 // Message length for ASCII 'on' message with delay for auto 'off' 
#define BUFLENGTH 128                   // Buffer length for incoming tcp messages
#define SELECT_TIMEOUT 10000            // usec block timeout on select for tcp and LCM
#define RETRIES 3						// Number of attempts to receive status over tcp before skipping relay (status update)

typedef struct
{
    lcm_t *lcm;
    char root_key[64];
    int64_t prev_time; // arrival time of previous LCM request message
    acfr_sensor_t *sensor; // tcp connection to board
    bool ok_response; // whether an ok response has been received back from the board, to acknowledge the request
    uint8_t request_no; // relay or io port where the change was requested, 0: none, 1..24 relays, 25..32 = 1..8 io's
    bool request_state; // state that was last requested on:true/off:false
    uint32_t state_list; // register for state bit per relay (0: inactive, 1: active) from 1..24 relays, 25..32 = 1..8 io's
    bool verbose;       // for extra debug prints
} state_t;

// Globals for signalling exit
int program_exit;
int broken_pipe;
void signal_handler(int sig_num)
{
    // on sig safe exit
    if(sig_num == SIGPIPE)
        broken_pipe = 1;
    else if (sig_num == SIGTERM)
        broken_pipe = 1;
    else if(sig_num == SIGINT)
        program_exit = 1;
}

// handles incoming LCM HB messages, at 1Hz, triggers polling all relays/ios via tcp, then send status message via LCM
void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    if (state->verbose)
    {
        fprintf(stderr,".");
    }
	int cmd_len = 0;
    int len = 0;
	int res = 0;

	// setup tcp 
	fd_set rfds, dup_rfds; // list of file descriptors where we will listen for incoming messages and duplicate set for use
    char buf[BUFLENGTH];
    struct timeval tv;
    int ret;
    FD_ZERO(&rfds);
    FD_SET(state->sensor->fd, &rfds); // add the relay file descriptor to the set to watch
    memset(buf, 0, sizeof(buf));

	char get_relay_cmd[GET_LEN] = {'G','R',' ','n','n','\n'};
	cmd_len = GET_LEN;

    for (int i = 1; i <= NUM_RELAYS; i++)
    {
		// copy in the relay number digits over the place holder 'n's
        div_t divresult = div(i,10);
        get_relay_cmd[3] = atoi((const char*)&divresult.quot);
        get_relay_cmd[4] = atoi((const char*)&divresult.rem);
		// send status request
		res = acfr_sensor_write(state->sensor, get_relay_cmd, cmd_len);
        if (state->verbose)
        {
            fprintf(stderr, "Relay cmd: %s", get_relay_cmd);
        }
        if (res < 0)
        {
            fprintf(stderr, "dS2824 Relay Driver: Failed to write to relay board. %i - %s", errno, strerror(errno));
		}
		// Get response from relay board
		dup_rfds = rfds; // reset file descriptors
        tv.tv_sec = 0;
        tv.tv_usec = SELECT_TIMEOUT;
        bool success = false;
        int count = 0;
		// check incoming message source
        while (!success && count < RETRIES)
        {
            ret = select (FD_SETSIZE, &dup_rfds, NULL, NULL, &tv);
            if(ret == -1) 
            {
                fprintf(stderr, "dS2824 Relay Driver: Select failure: %i", errno);
            }
            else if(ret != 0) // check incoming message
            {
                if(FD_ISSET(state->sensor->fd, &dup_rfds)) // tcp, read the bytes
                {
                   len += acfr_sensor_read(state->sensor, &buf[len], BUFLENGTH - len);
                    if (len > 0)
                    {
                        if ((buf[0] == 'A') && (len >= 2) && (buf[1] == 'c')) // Active message
                        {
		    				state->state_list = state->state_list | (uint32_t)(pow(2, i-1)); // Switch bit on
		    				len = 0; // full message received reset read char length
		    			}
		    			else if ((buf[0] == 'I') && (len >= 2) && (buf[1] == 'n')) // InActive message
                        {
                            state->state_list = state->state_list & ~(uint32_t)(pow(2, i-1)); // Switch bit off
		    				len = 0;
		    			}
		    			else
		    			{
		    				len = 0;
		    				if (state->verbose)
		    				{
		    					fprintf(stderr, "dS2824 Relay Driver: Unknown incoming tcp message: %s\n", buf);
		    				}
		    			} // end if unknown message
		    		} // end if chars read
                } // end if tcp
            } // end if select
            count ++;
        } // end success
        if (!success)
        {
            fprintf(stderr, "dS2824 Relay Driver: Cannot detect state of Relay %d status update skipped\n", i);
        }
    } // end relays

	char get_io_cmd[GET_LEN-1] = {'G','I',' ','n','\n'};
	cmd_len = GET_LEN-1;

    for (int i = 1; i <= NUM_IOS; i++)
    {
        // copy in the relay number digits over the place holder 'n's
        get_io_cmd[3] = atoi((const char*)&i);
		// send status request
        res = acfr_sensor_write(state->sensor, get_io_cmd, cmd_len);
        if (state->verbose)
        {
            fprintf(stderr, "IO cmd: %s", get_io_cmd); 
        }
        if (res < 0)
        {
            fprintf(stderr, "dS2824 Relay Driver (IO): Failed to write to relay board. %i - %s", errno, strerror(errno));
        }
		// Get response from relay board
		dup_rfds = rfds; // reset file descriptors
        tv.tv_sec = 0;
        tv.tv_usec = SELECT_TIMEOUT;
        bool success = false;
        int count = 0;
        // check incoming message source
        while (!success && count < RETRIES)
        {
            ret = select (FD_SETSIZE, &dup_rfds, NULL, NULL, &tv);
            if(ret == -1)
            {
                fprintf(stderr, "dS2824 Relay Driver (IO): Select failure: %i", errno);
            }
            else if(ret != 0) // check incoming message
            {
                if(FD_ISSET(state->sensor->fd, &dup_rfds)) // tcp, read the bytes
                {
                   len += acfr_sensor_read(state->sensor, &buf[len], BUFLENGTH - len);
                    if (len > 0)
                    {
                        if ((buf[0] == 'A') && (len >= 2) && (buf[1] == 'c')) // Active message
                        {
                            state->state_list = state->state_list | (uint32_t)(pow(2, i-1)); // Switch bit on
                            len = 0; // full message received reset read char length
                        }
                        else if ((buf[0] == 'I') && (len >= 2) && (buf[1] == 'n')) // InActive message
                        {
                            state->state_list = state->state_list & ~(uint32_t)(pow(2, i-1)); // Switch bit off
                            len = 0;
                        }
                        else
                        {
                            len = 0;
                            if (state->verbose)
                            {
                                fprintf(stderr, "dS2824 Relay Driver (IO): Unknown incoming tcp message: %s\n", buf);
                            }
                        } // end if unknown message
                    } // end if chars read
                } // end if tcp
            } // end if select
            count ++;
        } // end success
        if (!success)
        {
            fprintf(stderr, "dS2824 Relay Driver (IO): Cannot detect state of IO %d status update skipped\n", i);
        }
    } //end ios
} // end hb handler

// handles incoming LCM relay requests, sends requests to the board via tcp
void relay_cmd_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_relay_command_t *mc, void *u)
{
    state_t *state = (state_t *)u;
    if (state->verbose)
    {
        fprintf(stderr, "LCM");
    }
    // if message timestamp is latest received, and still valid (not older than 5s)
    if ((mc->utime >= state->prev_time) && ((timestamp_now() - CMD_TIMEOUT) <= mc->utime))
    {
        // update state
        state->prev_time = mc->utime;

        int len = 0;
        // Set relay state command (not I/O output state)
        if ((mc->relay_number > 0) && (mc->relay_number <= NUM_IOS)) // TODO && (mc->relay_device[mc->relay_number] != "no_device"))
        {
            char set_relay_on_cmd[ON_LEN] = {'S','R',' ','n','n',' ','o','n','\n'};
            char set_relay_off_cmd[OFF_LEN] = {'S','R',' ','n','n',' ','o','f','f','\n'};
			div_t divresult = div(mc->relay_number,10);

            // send SR (set relay) command
            if (mc->relay_request == true) // set ON
            {
                len = ON_LEN;
				// copy in the relay number digits over the place holder 'n's
            	set_relay_on_cmd[3] = atoi((const char*)&divresult.quot);
            	set_relay_on_cmd[4] = atoi((const char*)&divresult.rem);
				// If relay off delay value is set, need to turn the relay off again after that many ms
				if ((mc->relay_request == true) && (mc->relay_off_delay >= MIN_DELAY) && (mc->relay_off_delay <= MAX_DELAY))
            	{
                	// TODO - implement delayed off command functionality

            	}
				// Then send the command
            	if (state->verbose)
            	{
            	    fprintf(stderr, "Relay on command: %s/n",set_relay_on_cmd);
            	}
            	state->ok_response = false; // set response received equal to false
            	state->request_no = mc->relay_number; // record where the request from 1..24 relays 25..32 = 1..8 io's
            	int res = acfr_sensor_write(state->sensor, set_relay_on_cmd, len);
            	if (res < 0)
            	{
            	    fprintf(stderr, "dS2824 Relay Driver: Failed to write to relay board. %i - %s", errno, strerror(errno));
            	}
            }
            else // false: set OFF
            {
                len = OFF_LEN;
				// copy in the relay number digits over the place holder 'n's
            	set_relay_off_cmd[3] = atoi((const char*)&divresult.quot);
            	set_relay_off_cmd[4] = atoi((const char*)&divresult.rem);

            	// Then send the command
            	if (state->verbose)
            	{
            	    fprintf(stderr, "Relay off command: %s/n",set_relay_off_cmd);
            	}
            	state->ok_response = false; // set response received equal to false
            	state->request_no = mc->relay_number; // record where the request from 1..24 relays 25..32 = 1..8 io's
            	int res = acfr_sensor_write(state->sensor, set_relay_off_cmd, len); 
				if (res < 0)
		    	{
        			fprintf(stderr, "dS2824 Relay Driver: Failed to write to relay board. %i - %s", errno, strerror(errno));
     			}
			}
        }
        // Set I/O output state command (not relay state)
        else if ((mc->io_number > 0) && (mc->io_number <= NUM_IOS)) // TODO && (mc->io_device[mc->io_number] != "no_device"))
        {
            char set_io_on_cmd[ON_LEN-1] = {'S','O',' ','n',' ','o','n','\n'};
			char set_io_off_cmd[OFF_LEN-1] = {'S','O',' ','n',' ','o','f','f','\n'};
			// send SO (set I/O output) command
			if (mc->io_request == true) // set ON
            {
                len = ON_LEN-1;
				// copy in the relay number digits over the place holder 'n's
            	set_io_on_cmd[3] = atoi((const char*)&mc->io_number);
				// Then send the command
            	if (state->verbose)
            	{
            	    fprintf(stderr, "I/O on command: %s/n",set_io_on_cmd);
            	}
            	state->ok_response = false; // set response received equal to false
            	state->request_no = NUM_RELAYS + mc->io_number; // record where the request from 1..24 relays 25..32 = 1..8 io's

            	int res = acfr_sensor_write(state->sensor, set_io_on_cmd, len);
            	if (res < 0)
            	{       
            	    fprintf(stderr, "dS2824 Relay Driver: Failed to write to relay board. %i - %s", errno, strerror(errno));
            	}


            }
            else // false: set OFF
            {
                len = OFF_LEN-1;
				// copy in the relay number digits over the place holder 'n's
            	set_io_off_cmd[3] = atoi((const char*)&mc->io_number);
            	// Then send the command
            	if (state->verbose)
            	{
            	    fprintf(stderr, "I/O off command: %s/n",set_io_off_cmd);
            	}
            	state->ok_response = false; // set response received equal to false
            	state->request_no = NUM_RELAYS + mc->io_number; // record where the request from 1..24 relays 25..32 = 1..8 io's

            	int res = acfr_sensor_write(state->sensor, set_io_off_cmd, len);
    			if (res < 0)
    			{		
        			fprintf(stderr, "dS2824 Relay Driver: Failed to write to relay board. %i - %s", errno, strerror(errno));
		    	}
			}
        } // end io
        else
        {
            // No valid state request, so ignore
            return;
        }
        // Return to main loop to wait for response
        return;
    }
} // end relay_cmd_handler

int main(int argc, char **argv)
{
    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);
    signal(SIGPIPE, signal_handler); 
    signal(SIGTERM, signal_handler); 

    // Initialise state which holds all current data
    state_t state;
    state.prev_time = timestamp_now();
    memset(&state.root_key, 0, 64);
    state.ok_response = false; // received no response
    state.request_no = 0; // set to out of range 0: none, 1..24 relays, 25..32 = 1..8 io's
    state.request_state = false; // request off state
    state.state_list = 0; // bits 1..24 relays, 25..32 = 1..8 io's
    
    
    // Initialise LCM
    state.lcm = lcm_create(NULL);

    // Check for command line args
    char opt;
    while((opt = getopt(argc, argv, "h:")) != -1)
    {
        if(opt == 'h') // help option
        {
            fprintf(stderr, "Usage: dS2824_relays \n");
            fprintf(stderr, "e.g. dS2824_relays -h (to show this help msg)\n");
            return 0;
        }
    }

    // Get the config root key from the command line, program name
	char root_key[64];
	sprintf(root_key, "%s", basename(argv[0]));

    // Read the config file now 
    char key[64]; // temp to hold keys for lookup
    BotParam *param = bot_param_new_from_server (state.lcm, 1);
    if(param == NULL)
    {
        fprintf(stderr, "dS2824 Relay Driver: cannot access bot-param config\n");
        return 0;
    }

    // Read the lcm channel names 
    sprintf(key, "%s.channel_control", root_key);
    printf("Requesting Parameter: %s\n", key);
    char *channel_control = bot_param_get_str_or_fail(param, key);
    printf("Channel in: %s\n", (char *)channel_control);
    sprintf(key, "%s.channel_status", root_key);
    printf("Requesting Parameter: %s\n", key);
    char *channel_status = bot_param_get_str_or_fail(param, key);
    printf("Channel out: %s\n", (char *)channel_status);
    sprintf(key, "%s.verbose", root_key);
    bool verbose = bot_param_get_boolean_or_fail(param, key);
    state.verbose = verbose;
    // get the list of devices attached to relays
    char relay_devices[NUM_RELAYS][MAX_DEV_NAME_LEN];
    for (int i = 1; i <= NUM_RELAYS; i++)
    {
        sprintf(key, "%s.relay_%d", root_key, i);
        printf("Requesting Parameter: %s\n", key);
        strcpy(relay_devices[i-1], bot_param_get_str_or_fail(param, key));
    }
    // get the list of devices attached to I/Os
    char io_devices[NUM_IOS][MAX_DEV_NAME_LEN];
    for (int j = 1; j <= NUM_IOS; j++)
    {
        sprintf(key, "%s.io_%d", root_key, j);
        printf("Requesting Parameter: %s\n", key);
        strcpy(io_devices[j-1], bot_param_get_str_or_fail(param, key));
    }

    // Subscribe to the LCM channels for incoming messages
    acfrlcm_relay_command_t_subscribe(state.lcm, channel_control, &relay_cmd_handler, &state);
	perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);

    // Read the io tcp config to setup Ethernet connection to relay board
    state.sensor = acfr_sensor_create(state.lcm, root_key);
    if(state.sensor == NULL)
    {
        fprintf(stderr, "Could not open tcp connection to relay board: %s\n", state.root_key);
        return 0;
    }
    acfr_sensor_canonical(state.sensor, 0x0D, 0); // see if carriage return works TODO confirm
    //state.root_key = root_key;
    
    // Incoming message buffer
    fd_set rfds, dup_rfds; // list of file descriptors where we will listen for incoming messages and duplicate set for use
    int lcm_fd = lcm_get_fileno(state.lcm); // get the file descriptor id for the lcm messager
    char buf[BUFLENGTH];
    int len = 0;
    struct timeval tv;
    int ret;
    FD_ZERO(&rfds);
    FD_SET(state.sensor->fd, &rfds); // add the motor serial file descriptor to the set to watch
    FD_SET(lcm_fd, &rfds); // add the lcm file descriptor to the set to watch 
    memset(buf, 0, sizeof(buf));
    if (verbose)
    {
        printf("SETUP COMPLETE\n");
    }

    // Main program loop - used to call LCM handlers for incoming LCM messages and send a status message when tcp messages confirming a change are received
    while(!program_exit)
    {
        dup_rfds = rfds; // reset file descriptors
        tv.tv_sec = 0;
        tv.tv_usec = SELECT_TIMEOUT;
        
        // check incoming message sources
        ret = select (FD_SETSIZE, &dup_rfds, NULL, NULL, &tv);
        if(ret == -1) 
        {
            fprintf(stderr, "dS2824 Relay Driver: Select failure: %i", errno);
        }
        else if(ret != 0) // check incoming message
        {
            if(FD_ISSET(lcm_fd, &dup_rfds)) // LCM message, call the handler
            {
                lcm_handle(state.lcm);
            }
            if(FD_ISSET(state.sensor->fd, &dup_rfds)) // tcp, read the bytes
            {
                len += acfr_sensor_read(state.sensor, &buf[len], BUFLENGTH - len);
                if (len > 0)
                {
                    if ((buf[0] == 'O') && (len >= 2) && (buf[1] == 'k')) // Ok message
                    {
						len = 0; // full message received reset read char length
                        if ((state.request_no > 0) && (state.ok_response == false))
                        {
                            // update the register
                            if (state.request_state == true) // request on
                            {
                                state.state_list = state.state_list | (uint32_t)(pow(2, state.request_no)); // bit on
                            } 
                            else // request off
                            {
                                state.state_list = state.state_list & ~(uint32_t)(pow(2, state.request_no)); // bit off
                            }
                            // break status message into bytes (for LCM)
                            acfrlcm_relay_status_t status_msg;
                            status_msg.utime = timestamp_now();
							status_msg.state_list[0] = (state.state_list >> 24) & 0xFF;
							status_msg.state_list[1] = (state.state_list >> 16) & 0xFF;
							status_msg.state_list[2] = (state.state_list >> 8) & 0xFF;
							status_msg.state_list[3] = state.state_list & 0xFF;
							// publish status message
                            acfrlcm_relay_status_t_publish(state.lcm, channel_status, &status_msg);
                            
                            if (verbose) // print status
                            {
                               printf("Update status: %X\n", state.state_list);
                            }
                        }
                    } // end ok msg
                }
            } // end tcp         
        }

    } // end main loop

    // close tcp and LCM commections
    acfr_sensor_destroy(state.sensor);
    lcm_destroy(state.lcm);

    return 0;
} // end main



