/*
    Listens for an LCM relay command message and sends the corresponding request to the relay board via Ethernet. 
    
    Jorja Martin 10/2007
    ACFR
*/

//#include <stdio.h>
//#include <stdlib.h>
//#include <signal.h>
//#include <libgen.h>
//#include <math.h>
//#include <errno.h>
//#include <string.h>

#include <bot_param/param_client.h>
#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"
//#include "acfr-common/units.h"
#include "acfr-common/lcm_util.h"
#include "perls-lcmtypes/acfrlcm_relay_command_t.h"
#include "perls-lcmtypes/acfrlcm_relay response_t.h"

// Conversion values
//#define BYTE_MAX 256                // hex byte conversion value
//#define BITS_PER_BYTE 8             // for bit shifting
//#define S_TO_MICROS 1000000         // 10^6 conversion to microsecond

// Message structure matching values

// Other contants
#define CMD_TIMEOUT 5000000
#define NUM_RELAYS 24                   // Number of relays on board
#define NUM_IOS 8                       // Number of IO ports on board
#define MAX_DEV_NAME_LEN 20                  // Maximum length for an attached device name 
#define MAX_DELAY 2147483647            // Maximum auto relay off dealy in ms
#define MIN_DELAY 100                   // Minimum auto relay off delay in ms
#define ON_LEN 8                        // Message length for ASCII 'on' message
#define OFF_LEN 9                       // Message length for ASCII 'off' message
#define ON_DELAY_LEN 19                 // Message length for ASCII 'on' message with delay for auto 'off' 

typedef struct
{
    lcm_t *lcm;
    char root_key[64];
    acfr_sensor_t *sensor;
//    int fwd_speed;
//    int rev_speed;
    char counter;
    int64_t prev_time;
//    acfrlcm_asv_torqeedo_motor_command_t mot_command;
//    acfrlcm_asv_torqeedo_motor_status_t mot_status;

} state_t;

// globals for signalling exit
int program_exit;

void signal_handler(int sig_num)
{
    // do a safe exit
    if(sig_num == SIGPIPE)
        broken_pipe = 1;
    else if (sig_num == SIGTERM)
        broken_pipe = 1;
    else if(sig_num == SIGINT)
        program_exit = 1;
}

    if (res < 0)
    {
        fprintf(stderr, "dS2824 Relay Driver: Failed to write to relay drive. %i - %s", errno, strerror(errno));
        program_exit = 1;
     }

} // end send_msg


void relay_cmd_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_relay_command_t *mc, void *u)
{
    state_t *state = (state_t *)u;
    // if message timestamp is latest received, and still valid (not older than 5s)
    if ((mc->utime >= state->prev_time) && ((timestamp_now() - CMD_TIMEOUT) <= mc->utime))
    {
        // update state
        state->prev_time = mc->utime;
        //state->relay_number = mc->relay_number;
        //state->relay_request = mc->relay_request;
        //state->relay_off_delay = mc->relay_off_delay;
        //state->io_number = mc->io_number;
        //state->io_request = mc->io_request;

        int len = 0;
        // Set relay state command (not I/O output state)
        if ((mc->relay_number > 0) && (mc->relay_number <= NUM_IOS) && (mc->relay_device[mc->relay_number] != "no_device"))
        {
            // send SR (set relay) command
            if (mc->relay_request = true) // set ON
            {
                char set_relay_cmd[ON_LEN] = {'S','R',' ','n','n',' ','o','n'};
                len = ON_LEN;
            }
            else // false: set OFF
            {
                char set_relay_cmd[OFF_LEN] = {'S','R',' ','n','n',' ','o','f','f'};
                len = OFF_LEN;
            }
            // copy in the relay number digits over the place holder 'n's
            div_t divresult = div(relay_number,10);
            set_relay_cmd[3] = atoi(mc->divresult.quot);
            set relay_cmd[4] = atoi(mc->divresult.rem);
            // If relay off delay value is set, need to turn the relay off again after that many ms
            if ((mc->relay_request == true) && (mc->relay_off_delay >= MIN_DELAY) && mc->relay_off_delay <= MAX_DELAY))
            {
                // TODO - implement delayed off command functionality

            }
            // Then send the command
            if (verbose)
            {
                printf(set_io_cmd);
            }
            int res = acfr_sensor_write(s, set_relay_cmd, len);
			if (res < 0)
		    {
        		fprintf(stderr, "dS2824 Relay Driver: Failed to write to relay drive. %i - %s", errno, strerror(errno));
        		program_exit = 1;
     		}
        }
        // Set I/O output state command (not relay state)
        else if ((mc->io_number > 0) && (mc->io_number <= NUM_IOS) && (mc->io_device[mc->io_number] != "no_device"))
        {
            // send SO (set I/O output) command
			if (mc->io_request = true) // set ON
            {
                char set_io_cmd[ON_LEN] = {'S','O',' ','n','n',' ','o','n'};
                len = ON_LEN;
            }
            else // false: set OFF
            {
                char set_io_cmd[OFF_LEN] = {'S','O',' ','n','n',' ','o','f','f'};
                len = OFF_LEN;
            }
            // copy in the relay number digits over the place holder 'n's
            div_t divresult = div(relay_number,10);
            set_io_cmd[3] = atoi(mc->divresult.quot);
            set io_cmd[4] = atoi(mc->divresult.rem);
            // Then send the command
            if (verbose)
            {
                printf(set_io_cmd);
            }
            int res = acfr_sensor_write(s, set_io_cmd, len);
    		if (res < 0)
    		{		
        		fprintf(stderr, "dS2824 Relay Driver: Failed to write to relay drive. %i - %s", errno, strerror(errno));
        		program_exit = 1;
		    }
        } // end io
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
    memset(&state.root_key, 0, 64);
    //state.fwd_speed = -1; 
    //state.rev_speed = -1; // both negative for idle (stopped) at start 
    //state.counter = 0;
    //state.prev_time = timestamp_now();
    //memset(&state.mot_command, 0, sizeof(acfrlcm_asv_torqeedo_motor_command_t));
    //memset(&state.mot_status, 0, sizeof(acfrlcm_asv_torqeedo_motor_status_t));
    
    // Initialise LCM
    state.lcm = lcm_create(NULL);

    // Check for command line args
    char opt;
    int got_key = 0;
    while((opt = getopt(argc, argv, "h:")) != -1)
    {
        if(opt == 'h')
        {
            fprintf(stderr, "Usage: dS2824_relays \n");
            fprintf(stderr, "e.g. dS2824_relays -h (to show this help info)\n");
            return 0;
        }
    }

    // Get the config root key from the command line, program name
    sprintf(rootkey, "%s", basename(argv[0]));

    // Read the config file now 
    char key[64]; // temp to hold keys for lookup
    BotParam *param = bot_param_new_from_server (state.lcm, 1);
    if(param == NULL)
    {
        fprintf(stderr, "dS2824 Relay Driver: cannot access bot-param config\n");
        return 0;
    }

    // Read the lcm channel names 
    sprintf(key, "%s.channel_control", state.root_key);
    printf("Requesting Parameter: %s\n", key);
    char *channel_control = bot_param_get_str_or_fail(param, key);
    printf("Channel in: %s\n", (char *)channel_control);
    sprintf(key, "%s.channel_response", state.root_key);
    printf("Requesting Parameter: %s\n", key);
    char *channel_response = bot_param_get_str_or_fail(param, key);
    printf("Channel out: %s\n", (char *)channel_response);
    sprintf(key, "%s.verbose", state.root_key);
    bool verbose = bot_param_get_boolean_or_fail(param, key);
    // get the list of devices attached to relays
    char relay_devices[NUM_RELAYS][MAX_DEV_NAME_LEN];
    for (int i = 1; i <= NUM_RELAYS; i++)
    {
        sprintf(key, "%s.relay_%d", state.root_key, i);
        printf("Requesting Parameter: %s\n", key);
        char *relay_devices[i][0] = bot_param_get_str_or_fail(param, key);
    }
    // get the list of devices attached to I/Os
    char io_devices[NUM_IOS][MAX_DEV_NAME_LEN];
    for (int i = 1; i <= NUM_IOS; i++)
    {
        sprintf(key, "%s.io_%d", state.root_key, i);
        printf("Requesting Parameter: %s\n", key);
        char *io_devices[i][0] = bot_param_get_str_or_fail(param, key);
    }

    // Subscribe to the LCM channels for incoming messages
    acfrlcm_relay_command_t_subscribe(state.lcm, channel_control, &relay_cmd_handler, &state);

    // Read the io tcp config to setup Ethernet connection to relay board
    state.sensor = acfr_sensor_create(state.lcm, state.root_key);
    if(state.sensor == NULL)
    {
        fprintf(stderr, "Could not open tcp connection to relay board: %s\n", state.root_key);
        return 0;
    }
    acfr_sensor_noncanonical(state.sensor, 1, 0);

    // Incoming message buffer
    fd_set rfds, dup_rfds; // list of file descriptors where we will listen for incoming messages and duplicate set for use
    int lcm_fd = lcm_get_fileno(state.lcm); // get the file descriptor id for the lcm messager
    char buf[BUFLENGTH];
    struct timeval tv;
    int ret;
    bool start_found = false; // start of message AC found
    bool end_found = false; // end of message AD found
    bool mask_next_value = false; // no escape characters currently detected
    int i = 0;
    FD_ZERO(&rfds);
    FD_SET(state.sensor->fd, &rfds); // add the motor serial file descriptor to the set to watch
    FD_SET(lcm_fd, &rfds); // add the lcm file descriptor to the set to watch 
    memset(buf, 0, sizeof(buf));
    memset(&state.mot_command, 0, sizeof(acfrlcm_asv_torqeedo_motor_status_t));
    printf("SETUP COMPLETE\n");

    // Main program loop - used to read status messages from the 
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
                mask_next_value = false; // no escape characters currently detected
                // while there are more characters to read
//                while(!end_found && (acfr_sensor_read(state.sensor, &buf[i], 1))) // && (i < MATCH_BYTE_LEN))
//                {   
//                    //printf("%02X %i\n", (uint8_t)buf[i], i);
//                    // match message start
//                    if ((uint8_t)buf[i] == (uint8_t)MATCH_BYTE_0)
//                    {
//                        start_found = true;
//                        state.mot_status.utime = timestamp_now();
//                        end_found = false; // only count an end after a start
//                    }
//                    if (start_found && (uint8_t)buf[i] == MATCH_BYTE_END)
//                    {
//                        end_found = true;
//                    }
//					// if an AE (escape character) has been detected and removed, we need to mask the next incoming character
//                    if (start_found && !end_found && mask_next_value)
//                    {
//                        buf[i] = buf[i] + BYTE_MASK;
//					    mask_next_value = false;
//                    }
//					if (start_found && (uint8_t)buf[i] != MATCH_BYTE_ESCAPE) // dont inrement to overwrite 'AE' control characters in msgs
//                    {
//                        i++; // increment to next space in storage array
//                    }
//                    else if (start_found && (uint8_t)buf[i] == MATCH_BYTE_ESCAPE)
//                    {
//                        // don't increment, set flag to apply mask to next character
//                        mask_next_value = true; 
//                    }
//                    if (i >= BUFLENGTH) // reset buffer in case of (invalid) long msg
//                        i = 0;
//                }

//                if (start_found && end_found)
//                {   
//                    //printf("Start & End\n"); 
//                    if (i == MATCH_BYTE_LEN && buf[1] == MATCH_BYTE_1 && buf[2] == MATCH_BYTE_2)
//                    {
//                        // fill LCM motor status message from serial buffer
//               
//                        //state.mot_status.prop_speed = (int16_t) buf[P_SPEED_POS]; // copy MSB
//                        //state.mot_status.prop_speed =<< BITS_PER_BYTE; // shift one byte 
//                        //state.mot_status.prop_speed += buf[P_SPEED_POS + 1] // and add the LSB
//                        state.mot_status.prop_speed = BYTE_MAX * (uint8_t)buf[P_SPEED_POS] + (uint8_t)buf[P_SPEED_POS + 1];
//                        state.mot_status.voltage = BYTE_MAX * (uint8_t)buf[VOLT_POS] + (uint8_t)buf[VOLT_POS + 1];
//                        state.mot_status.field_2 = BYTE_MAX * (uint8_t)buf[CURRENT_POS] + (uint8_t)buf[CURRENT_POS + 1];  // TODO change this to current
//                        state.mot_status.motor_temp = (uint8_t)buf[PCB_TEMP_POS]; // TODO change this to PCB temp 16bit signed
//                        signed short pcb_temp =  BYTE_MAX * (uint8_t)buf[PCB_TEMP_POS] + (uint8_t)buf[PCB_TEMP_POS + 1];
//                        state.mot_status.field_4 = BYTE_MAX * (uint8_t)buf[STATOR_TEMP_POS] + (uint8_t)buf[STATOR_TEMP_POS + 1]; // TODO change this to stator temp
//
//                        // and publish status message
//                        acfrlcm_asv_torqeedo_motor_status_t_publish(state.lcm, channel_response, &state.mot_status);
//
//                        if (verbose) // print motor status
//                        {
//                            printf("Prop Speed: %05i   Voltage: %3.2f   Current: %3.2f   PCB Temperature: %3.1f   Stator Temperature: %3.1f\n", state.mot_status.prop_speed, ((double)state.mot_status.voltage)/100, ((double)state.mot_status.field_2)/10, ((double)pcb_temp)/10, (double)state.mot_status.field_4/10);
//                        }
//                        // reset outgoing message 
//                        memset(&state.mot_command, 0, sizeof(acfrlcm_asv_torqeedo_motor_status_t));
//                    }
//                    // reset flags, buffer, index
//                    start_found = false;
//                    end_found = false;
//                    memset(buf, 0, sizeof(buf));
//                    i = 0;
//                }

            } // end tcp         
        }

    } // end main loop

    // close tcp and LCM commections
    acfr_sensor_destroy(state.sensor);
    lcm_destroy(state.lcm);

    return 0;
} // end main



