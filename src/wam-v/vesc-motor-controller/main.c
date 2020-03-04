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
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <pthread.h>


#include <bot_param/param_client.h>
#include "acfr-common/timestamp.h"
#include "acfr-common/sensor.h"
#include "acfr-common/units.h"
#include "acfr-common/lcm_util.h"
#include "perls-lcmtypes/acfrlcm_asv_torqeedo_motor_command_t.h"
#include "perls-lcmtypes/acfrlcm_asv_torqeedo_motor_status_t.h"
#include "perls-lcmtypes/perllcm_heartbeat_t.h"


#include <fcntl.h>
#include <termios.h>

#include "buffer.h"
#include "datatypes.h"
#include "bldc_interface.h"
#include "crc.h"
#include "packet.h"
#include "bldc_interface_uart.h"
#include "comm_uart.h"





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
#define MAX_SPEED 5000              // maximum prop control speed value +&-
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

int fd;
char *channel_status;
// globals for signalling exit
int program_exit;

state_t state;



static void send_func(unsigned char *d, unsigned int len)
{
    printf("sending: ");
    for (int i = 0; i < len; i++)
        printf("0x%02X ", d[i]);
    printf("\n");

    write(fd, (void *)d, len);
}


// read thread which collect all incoming serial and processes int 
void *read_thread()
{
    fd_set rfds;
    struct timeval tv;
    int retval;
    unsigned char d;
    printf("read thread starting\n");

    // will run unless the exit command (control-c) is triggered
    while (!program_exit)
    {
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        // timeouts set to occur every second
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        // allows program to monitor different files, such as the file descript fd used for the serial communication
        retval = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);

        if (retval > 0)
        {

            read(fd, &d, 1);
            //printf("0x%02X\n", d & 0xFF);
            bldc_interface_uart_process_byte(d);
        }
        else
        {
            printf("select timeout, %d\n", retval);
        }
    }
}

// timer thread which must run at least once every millisecond
void *timer_thread()
{
    printf("timer thread starting\n");
    while (!program_exit)
    {
        packet_timerfunc();
        usleep(1000);
    }
}


int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0)
    {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      /* 8-bit characters */
    tty.c_cflag &= ~PARENB;  /* no parity bit */
    tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}





void signal_handler(int sig_num)
{
    // do a safe exit
    if(sig_num == SIGINT)
        bldc_interface_set_rpm(0);
        program_exit = 1;
}

int send_motor_command()
{
    //if ((state.enabled) && (state.cmd_speed != 0))
    if ((state.enabled))
    {
        printf("F: %d\n", state.cmd_speed);
        if (state.cmd_speed > MAX_SPEED)
        {
            state.cmd_speed = MAX_SPEED;
            fprintf(stderr, "Torqeedo: Speed control value (forward) out of range.");
        }
        else if (state.cmd_speed < -MAX_SPEED)
        {
            state.cmd_speed = -MAX_SPEED;
            fprintf(stderr, "Torqeedo: Speed control value (reverse) out of range.");
        }
        bldc_interface_set_rpm(state.cmd_speed);
    }
    else
    {
    }
}

void torqeedo_cmd_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_asv_torqeedo_motor_command_t *mc, void *u)
{
    
    // if message timestamp is latest received, and still valid (not older than 2s)
    if ((mc->utime >= state.prev_time) && ((timestamp_now() - CMD_TIMEOUT) <= mc->utime))
    {
        printf("Recieved motor command\n");
        // update state
        state.prev_time = mc->utime;
        state.cmd_speed = mc->command_speed;
        state.enabled = mc->enabled;
    }
//    printf("TC in:  F %i R %i\n", state->fwd_speed, state->rev_speed);
}

void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    //printf("HB\n");
    // On the heart beat we will trigger writing messages to serial
    if (state.counter % NUM_MSGS == 0) // Alternate TODO and check timeouts
    {
        send_motor_command(); 
    }
    else
    {
        //printf("Acquiring values: \n");
        bldc_interface_get_values();
    }
    state.counter++;
    
}

void bldc_val_received(mc_values *val)
{
    // printf("\r\n");
    // printf("Input voltage: %.2f V\r\n", val->v_in);
    // printf("Temp:          %.2f degC\r\n", val->temp_mos);
    // printf("Current motor: %.2f A\r\n", val->current_motor);
    // printf("Current in:    %.2f A\r\n", val->current_in);
    // printf("RPM:           %.1f RPM\r\n", val->rpm);
    // printf("Duty cycle:    %.1f %%\r\n", val->duty_now * 100.0);
    // printf("Ah Drawn:      %.4f Ah\r\n", val->amp_hours);
    // printf("Ah Regen:      %.4f Ah\r\n", val->amp_hours_charged);
    // printf("Wh Drawn:      %.4f Wh\r\n", val->watt_hours);
    // printf("Wh Regen:      %.4f Wh\r\n", val->watt_hours_charged);
    // printf("Tacho:         %i counts\r\n", val->tachometer);
    // printf("Tacho ABS:     %i counts\r\n", val->tachometer_abs);
    // printf("Fault Code:    %s\r\n", bldc_interface_fault_to_string(val->fault_code));

    // fill LCM motor status message from serial buffer
    state.mot_status.prop_speed = val->rpm;
    state.mot_status.voltage = val->v_in;
    state.mot_status.current = val->current_in;  // current
    state.mot_status.pcb_temp = val->temp_mos;
    state.mot_status.stator_temp = 0; // stator temp

    // and publish status message
    acfrlcm_asv_torqeedo_motor_status_t_publish(state.lcm, channel_status, &state.mot_status);
}


 

int main(int argc, char **argv)
{
    
    // install the signal handler
    program_exit = 0;
    signal(SIGINT, signal_handler);
    signal(SIGPIPE, signal_handler); 
    signal(SIGTERM, signal_handler); 

    // Initialise state which holds all current data
    
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
    char *portname;

    // Determine which motor we control based on command line switch
    char opt;
    int got_key = 0;
    while((opt = getopt(argc, argv, "hk:n:d:")) != -1)
    {
        if(opt == 'k')
        {
            strcpy(state.root_key, optarg);
            got_key = 1;
        }
        if(opt == 'h')
        {
            fprintf(stderr, "Usage: vesc-controller -k <config key> -n <vehicle name>\n");
            fprintf(stderr, " e.g.: vesc-controller -k torqeedo.port-motor -n WAMV\n");
            return 0;
        }
        if(opt == 'n')
        {
			int n = strlen((char *)optarg);
            vehicle_name = malloc(n);
            strcpy(vehicle_name, (char *)optarg);
            printf("Vehicle: %s\n", vehicle_name);
        }
        if(opt == 'd')
        {
			int n = strlen((char *)optarg);
            portname = malloc(n);
            strcpy(portname, (char *)optarg);
            printf("Port: %s\n", portname);
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
        fprintf(stderr, "Vesc-controller: cannot access bot-param config\n");
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
	channel_status = malloc(strlen(vehicle_name) + strlen(channel_status_end) + 2);
	sprintf(channel_status, "%s.%s", vehicle_name, channel_status_end);
    printf("Channel out: %s\n", (char *)channel_status);

    sprintf(key, "%s.verbose", state.root_key);
    bool verbose = bot_param_get_boolean_or_fail(param, key);
    
    // Subscribe to the LCM channels for incoming messages
    acfrlcm_asv_torqeedo_motor_command_t_subscribe(state.lcm, channel_control, &torqeedo_cmd_handler, NULL);
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_10HZ", &heartbeat_handler, NULL);

    // Read the serial config to setup serial connection to motor
    // state.sensor = acfr_sensor_create(state.lcm, state.root_key);
    // if(state.sensor == NULL)
    // {
    //     fprintf(stderr, "Could not open motor serial port %s\n", state.root_key);
    //     return 0;
    // }
    // acfr_sensor_noncanonical(state.sensor, 1, 0);



    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
        while(!program_exit)
        {
            lcm_handle(state.lcm);
        }
        close(fd);
        lcm_destroy(state.lcm);
    }

    // baudrate 115200, 8 bits, no parity, 1 stop bit 
    set_interface_attribs(fd, B115200);

    pthread_t timerT;
    pthread_t readT;
    pthread_create(&timerT, NULL, timer_thread, NULL);
    pthread_create(&readT, NULL, read_thread, NULL);

    // initiate the bldc specific interfaces
    bldc_interface_uart_init(send_func);

    // bldc_interface_set_rx_mcconf_func(mcconf_callback);
    bldc_interface_set_rx_value_func(bldc_val_received);

    printf("SETUP Complete");
    while(!program_exit)
    {
        lcm_handle(state.lcm);
    }

    bldc_interface_set_rpm(0);
    pthread_join(timerT, NULL);
    pthread_join(readT, NULL);

    
    // close serial and LCM commections
    close(fd);
    lcm_destroy(state.lcm);

    return 0;
} // end main



