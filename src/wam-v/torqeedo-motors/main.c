/*
    Listens for an LCM motor command message and sends adjusts speed of the corresponding thruster via serial. 
    Two instances should be run, one per thruster, port and starboard.
    Jorja Martin 10/2007
    ACFR
*/

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <libgen.h>
#include <math.h>
#include <errno.h>
#include <string.h>

#include <bot_param/param_client.h>
#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"
#include "acfr-common/units.h"
#include "acfr-common/lcm_util.h"
#include "perls-lcmtypes/acfrlcm_asv_torqeedo_motor_command_t.h"
#include "perls-lcmtypes/acfrlcm_asv_torqeedo_motor_status_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"

// Conversion values
#define BYTE_MAX 256                // hex byte conversion value
#define BITS_PER_BYTE 8             // for bit shifting
#define S_TO_MICROS 1000000         // 10^6 conversion to microsecond
// Position of info in Status Msg
#define P_SPEED_POS 3
#define POWER_POS 5
#define VOLT_POS 7
#define CURRENT_POS 9              
#define PCB_TEMP_POS 11             
#define STATOR_TEMP_POS 13          
// Message structure matching values
#define MATCH_BYTE_0 172            // match header values AC 00 00 ... AD
#define MATCH_BYTE_1 0
#define MATCH_BYTE_2 0
#define MATCH_BYTE_END 173
#define MATCH_BYTE_ESCAPE 174       // AE escape character used where AC or AD occur in messages
#define BYTE_MASK 128         		// mask used to hide AC or AD when occurs in messges
#define MATCH_BYTE_LEN 17
#define MSG_3003 3003
#define MSG3003_LEN 6               // status request message length
#define MSG_3082 3082
#define MSG3082_LEN 10              // speed command message length - note: most are 10, could be up to 13 with added escape chars so all packed to 13
#define MAX_SUBS 3                  // maximum number of control character substitutions that could occur per message
#define MAX_SPEED 1000              // maximum prop control speed value +&-
#define MSG3082_SPEED_POS 5         // postion of speed value in outgoing serial msg
// Other contants
#define CMD_TIMEOUT 2000000
#define NUM_MSGS 2
#define BUFLENGTH 256
#define SELECT_TIMEOUT 10000        // usec block timeout on select for serial and LCM

typedef struct
{
    lcm_t *lcm;
    char root_key[64];
    acfr_sensor_t *sensor;
    int cmd_speed;
    char counter;
    int64_t prev_time;
    bool enabled;
    acfrlcm_asv_torqeedo_motor_command_t mot_command;
    acfrlcm_asv_torqeedo_motor_status_t mot_status;

} state_t;

// globals for signalling exit
int program_exit;

void signal_handler(int sig_num)
{
    // do a safe exit
    if(sig_num == SIGINT)
        program_exit = 1;
}

uint8_t calc_maxim1wire_crc(const uint8_t input_msg[], int leng)
{
    // Source: http://www.sal.wisc.edu/PFIS/docs/rss-nir/archive/public/Product%20Manuals/maxim-ic/AN27.pdf
    // This procedure calculates the cumulative Maxim 1-Wire CRC of all bytes passed to it.
    // The result accumulates in the global variable CRC.
    uint8_t crc = 0;
    const uint8_t lookup_table[] = {
     0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
     157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
     35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
     190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
     70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
     219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
     101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
     248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
     140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
     17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
     175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
     50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
     202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
     87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
     233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
     116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};
    for (int i = 0; i < leng; i++) // for each byte in msg
    {
        crc = lookup_table[(crc ^ input_msg[i])]; // bitwise XOR with CRC then lookup in table
        //printf("%X \n", crc);
    }

    return crc;
}

void send_msg(int message_type, state_t *u)
{    
    state_t *state = (state_t *)u;
    int res = 0;
    // string to send to request a status message
    static char msg_3003[MSG3003_LEN] = {0xAC,0x30,0x03,0xCF,0xAD,0xFF};
    // string to send zero speed, stopped (must send a 3082 msg or motor -> non-responsive)
    static char msg_3082_idle[MSG3082_LEN] = {0xAC, 0x30, 0x82, 0x00, 0x00, 0x00, 0x00, 0xA5, 0xAD, 0xFF}; // has to be correct length or motor error
    // string to send for forwards + rev speeds @[1]0x30=from motor, @[2]0x82=drive_msg, @[3]0x01=drive on+no error, @[4]0x64=100%power(0/100)
    char msg_3082_move[MSG3082_LEN + MAX_SUBS + 1] = {0xAC, 0x30, 0x82, 0x01, 0x64, 0x00, 0x00, 0x00, 0xAD, 0xFF, 0x00, 0x00, 0x00, 0x00}; // edit to add speed @[5..6](-1000..1000) and CRC @[7] (placeholders 0x00)

    switch(message_type)
    {
        // send motor status request message
        case MSG_3003:
            //printf("MSG_3003\n");
            res = acfr_sensor_write(state->sensor, (char *)msg_3003, MSG3003_LEN);
            break;

        // send motor speed control message
        case MSG_3082:
            //printf("MSG_3082\n");
            // if forwards, (and no reverse) OR reverse (and no forwards)
            if ((state->enabled) && (state->cmd_speed != 0))
            {
                printf("F: %d\n", state->cmd_speed);
                if (state->cmd_speed > MAX_SPEED)
                {
                    state->cmd_speed = MAX_SPEED;
                    fprintf(stderr, "Torqeedo: Speed control value (forward) out of range.");
                }
                else if (state->cmd_speed < -MAX_SPEED) // ensure value is in range of mesage array length
            	
                {
                    state->cmd_speed = -MAX_SPEED;
                    fprintf(stderr, "Torqeedo: Speed control value (reverse) out of range.");
                }
                // set fwd speed, int16_t into int8_t array
                msg_3082_move[MSG3082_SPEED_POS] = (int8_t)(state->cmd_speed >> BITS_PER_BYTE); // MSB
                div_t divresult = div (state->cmd_speed, BYTE_MAX);
                msg_3082_move[MSG3082_SPEED_POS + 1] = (int8_t)divresult.rem; // LSB

                //printf("speed: %d %d %d\n",state->fwd_speed, (int16_t) msg_3082_move[5], (int16_t) msg_3082_move[6]);

                // calculate + add crc to msg based on msg section: from address through to byte before crc
                msg_3082_move[7] = (uint8_t) calc_maxim1wire_crc((uint8_t*)&msg_3082_move[1], 6);

                // check for and substitute packet escape characters if req'd due to values equal to control chars (in power or in crc)
                int subs_count = 0; // count the num of substituted characters so that dont accidentally substitute the end char 
                for (int i=5; i < (MSG3082_LEN - 2 + subs_count); i++)
                {
                    if ((uint8_t) msg_3082_move[i] == 0xAC || // matches control char value
                        (uint8_t) msg_3082_move[i] == 0xAD ||
                        (uint8_t) msg_3082_move[i] == 0xAE )
                    {
                        subs_count++;
                        for (int j = (MSG3082_LEN + MAX_SUBS - 1); j > (i + 1); j--) // for the rest of the array
                        {
                            msg_3082_move[j] = msg_3082_move[j - 1]; // shift along one position - sufficient padding at end of array for us to do this
                        }
                        msg_3082_move[i+1] = msg_3082_move[i] - BYTE_MASK; // then shift original value with mask
                        msg_3082_move[i] = 0xAE; // and add in the escape character
                        
                    }
                }
            // write to serial output, adding 1 extra byte of 0's for timing issue (in documentation) wont work owise
            res = acfr_sensor_write(state->sensor, (char *)msg_3082_move, MSG3082_LEN + subs_count + 1);
            }

            // otherwise if not enabled &/OR speed equals 0, either idle or invalid - so treat as stopped
            else
            {
                res = acfr_sensor_write(state->sensor, (char *)msg_3082_idle, MSG3082_LEN);
            }
        break;
    }
    //printf("res: %d\n", res);
    if (res < 0)
    {
        fprintf(stderr, "Torqeedo: Failed to write to motor control socket. %i - %s", errno, strerror(errno));
        program_exit = 1;
    }

} // end send_msg


void torqeedo_cmd_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_asv_torqeedo_motor_command_t *mc, void *u)
{
    state_t *state = (state_t *)u;
    // if message timestamp is latest received, and still valid (not older than 2s)
    if ((mc->utime >= state->prev_time) && ((timestamp_now() - CMD_TIMEOUT) <= mc->utime))
    {
        // update state
        state->prev_time = mc->utime;
        state->cmd_speed = mc->command_speed;
        state->enabled = mc->enabled;
    }
//    printf("TC in:  F %i R %i\n", state->fwd_speed, state->rev_speed);
}

void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    //printf("HB\n");
    // On the heart beat we will trigger writing messages to serial
    if (state->counter % NUM_MSGS == 0) // Alternate TODO and check timeouts
    {
        send_msg(MSG_3082, state); // Motor speed message   
    }
    else
    {
        send_msg(MSG_3003, state); // Status request
    }
    state->counter++;
    
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
    state.cmd_speed = 0; 
    state.counter = 0;
    state.prev_time = timestamp_now();
    state.enabled = false;
    memset(&state.mot_command, 0, sizeof(acfrlcm_asv_torqeedo_motor_command_t));
    memset(&state.mot_status, 0, sizeof(acfrlcm_asv_torqeedo_motor_status_t));
    
    // Initialise LCM
    state.lcm = lcm_create(NULL);

	char *vehicle_name;

    // Determine which motor we control based on command line switch
    char opt;
    int got_key = 0;
    while((opt = getopt(argc, argv, "hk:n:")) != -1)
    {
        if(opt == 'k')
        {
            strcpy(state.root_key, optarg);
            got_key = 1;
        }
        if(opt == 'h')
        {
            fprintf(stderr, "Usage: torqeedo -k <config key> -n <vehicle name>\n");
            fprintf(stderr, " e.g.: torqeedo -k torqeedo.port-motor -n WAMV\n");
            return 0;
        }
        if(opt == 'n')
        {
			int n = strlen((char *)optarg);
            vehicle_name = malloc(n);
            strcpy(vehicle_name, (char *)optarg);
            printf("Vehicle: %s\n", vehicle_name);
        }
    }
    if(!got_key)
    {
        fprintf(stderr, "Torqeedo: a config file key is required, use -h for help\n");
        return 0;
    }

    // Read the config file now we know whether we are Port or Starboard
    char key[64]; // temp to hold keys for lookup
    BotParam *param = bot_param_new_from_server (state.lcm, 1);
    if(param == NULL)
    {
        fprintf(stderr, "Torqeedo: cannot access bot-param config\n");
        return 0;
    }

    // Read the lcm channel names 
    sprintf(key, "%s.channel_control", state.root_key);
    printf("Requesting Parameter: %s\n", key);
    //char *channel_control = bot_param_get_str_or_fail(param, key);
    char *channel_control_end = bot_param_get_str_or_fail(param, key);
	char *channel_control = malloc(strlen(vehicle_name) + strlen(channel_control_end) + 2);
	sprintf(channel_control, "%s.%s", vehicle_name, channel_control_end); 
    printf("Channel in: %s\n", (char *)channel_control);

    sprintf(key, "%s.channel_status", state.root_key);
    printf("Requesting Parameter: %s\n", key);
    //char *channel_status = bot_param_get_str_or_fail(param, key);
    char *channel_status_end = bot_param_get_str_or_fail(param, key);
	char *channel_status = malloc(strlen(vehicle_name) + strlen(channel_status_end) + 2);
	sprintf(channel_status, "%s.%s", vehicle_name, channel_status_end);
    printf("Channel out: %s\n", (char *)channel_status);

    sprintf(key, "%s.verbose", state.root_key);
    bool verbose = bot_param_get_boolean_or_fail(param, key);
    
    // Subscribe to the LCM channels for incoming messages
    acfrlcm_asv_torqeedo_motor_command_t_subscribe(state.lcm, channel_control, &torqeedo_cmd_handler, &state);
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_10HZ", &heartbeat_handler, &state);

    // Read the serial config to setup serial connection to motor
    state.sensor = acfr_sensor_create(state.lcm, state.root_key);
    if(state.sensor == NULL)
    {
        fprintf(stderr, "Could not open motor serial port %s\n", state.root_key);
        return 0;
    }
    // TODO - set RS-485 mode flags etc if req'd
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
            fprintf(stderr, "Torqeedo: Select failure: %i", errno);
        }
        else if(ret != 0) // check incoming message
        {
            if(FD_ISSET(lcm_fd, &dup_rfds)) // LCM message, call the handler
            {
                lcm_handle(state.lcm);
            }
            if(FD_ISSET(state.sensor->fd, &dup_rfds)) // serial, read the bytes
            {
                mask_next_value = false; // no escape characters currently detected
                // while there are more characters to read
                while(!end_found && (acfr_sensor_read(state.sensor, &buf[i], 1))) // && (i < MATCH_BYTE_LEN))
                {   
                    //printf("%02X %i\n", (uint8_t)buf[i], i);
                    // match message start
                    if ((uint8_t)buf[i] == (uint8_t)MATCH_BYTE_0)
                    {
                        start_found = true;
                        state.mot_status.utime = timestamp_now();
                        end_found = false; // only count an end after a start
                    }
                    if (start_found && (uint8_t)buf[i] == MATCH_BYTE_END)
                    {
                        end_found = true;
                    }
					// if an AE (escape character) has been detected and removed, we need to mask the next incoming character
                    if (start_found && !end_found && mask_next_value)
                    {
                        buf[i] = buf[i] + BYTE_MASK;
					    mask_next_value = false;
                    }
					if (start_found && (uint8_t)buf[i] != MATCH_BYTE_ESCAPE) // dont inrement to overwrite 'AE' control characters in msgs
                    {
                        i++; // increment to next space in storage array
                    }
                    else if (start_found && (uint8_t)buf[i] == MATCH_BYTE_ESCAPE)
                    {
                        // don't increment, set flag to apply mask to next character
                        mask_next_value = true; 
                    }
                    if (i >= BUFLENGTH) // reset buffer in case of (invalid) long msg
                        i = 0;
                }

                if (start_found && end_found)
                {   
                    //printf("Start & End\n"); 
                    if (i == MATCH_BYTE_LEN && buf[1] == MATCH_BYTE_1 && buf[2] == MATCH_BYTE_2)
                    {
                        // fill LCM motor status message from serial buffer
               
                        //state.mot_status.prop_speed = (int16_t) buf[P_SPEED_POS]; // copy MSB
                        //state.mot_status.prop_speed =<< BITS_PER_BYTE; // shift one byte 
                        //state.mot_status.prop_speed += buf[P_SPEED_POS + 1] // and add the LSB
                        state.mot_status.prop_speed = BYTE_MAX * (uint8_t)buf[P_SPEED_POS] + (uint8_t)buf[P_SPEED_POS + 1];
                        state.mot_status.voltage = BYTE_MAX * (uint8_t)buf[VOLT_POS] + (uint8_t)buf[VOLT_POS + 1];
                        state.mot_status.current = BYTE_MAX * (uint8_t)buf[CURRENT_POS] + (uint8_t)buf[CURRENT_POS + 1];  // current
                        state.mot_status.pcb_temp = BYTE_MAX * (uint8_t)buf[PCB_TEMP_POS] + (uint8_t)buf[PCB_TEMP_POS + 1];
                        //state.mot_status.pcb_temp = (int16_t)buf[PCB_TEMP_POS]; // PCB temp 16bit signed
                        //signed short pcb_temp =  BYTE_MAX * (uint8_t)buf[PCB_TEMP_POS] + (uint8_t)buf[PCB_TEMP_POS + 1];
                        state.mot_status.stator_temp = BYTE_MAX * (uint8_t)buf[STATOR_TEMP_POS] + (uint8_t)buf[STATOR_TEMP_POS + 1]; // stator temp

                        // and publish status message
                        acfrlcm_asv_torqeedo_motor_status_t_publish(state.lcm, channel_status, &state.mot_status);

                        if (verbose) // print motor status
                        {
                            printf("Prop Speed: %05i   Voltage: %3.2f   Current: %3.2f   PCB Temperature: %3.1f   Stator Temperature: %3.1f\n", state.mot_status.prop_speed, ((double)state.mot_status.voltage)/100, ((double)state.mot_status.current)/10, ((double)state.mot_status.pcb_temp)/10, (double)state.mot_status.stator_temp/10);
                        }
                        // reset outgoing message 
                        memset(&state.mot_command, 0, sizeof(acfrlcm_asv_torqeedo_motor_status_t));
                    }
                    // reset flags, buffer, index
                    start_found = false;
                    end_found = false;
                    memset(buf, 0, sizeof(buf));
                    i = 0;
                }

            } // end serial            
        }

    } // end main loop

    // close serial and LCM commections
    acfr_sensor_destroy(state.sensor);
    lcm_destroy(state.lcm);

    return 0;
} // end main



