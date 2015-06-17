#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>
#include <libgen.h>

#include "acfr-common/timestamp.h"
#include "acfr-common/units.h"
#include "perls-lcmtypes/senlcm_seabotix_sensors_t.h"
#include "perls-lcmtypes/acfrlcm_seabotix_joystick_t.h"

#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "acfr-common/sensor.h"


// Data conversion
#define d2f(x) *(float *)x
#define d2current(x) ((float)x / 10.0)
#define d2speed(x) (short)((short)(x - 0x80) * -1 * 4500 / 102)

#define OPERATOR_AUTO_DEPTH 0
#define OPERATOR_AUTO_HEADING 0
#define OPERATOR_TRIM 0

typedef struct
{
    lcm_t *lcm;
    acfr_sensor_t *sensor;

	acfrlcm_seabotix_joystick_t joystick;
	int sending_joystick;

	int vertical_gain;
	int horizontal_gain;

} state_t;


unsigned char seabotix_checksum(unsigned char *message, long size)
{

    int sum = 0;

    for (int i=0; i<size; i++)   // disregard last element (should be 0 anyway)
    {
        sum = sum + message[i];
    }

    unsigned char chksm = (unsigned char)(sum & 0xFF);
    //message[size-1] = chksm;

    return chksm;
}

int set_heading(state_t *state) //unsigned char HEADING_SENSOR_, int16_t VAL) {
{
/*    
    if ((HEADING_SENSOR_ == 1) && (abs(VAL)>1800)) {
        printf("Error: Declination angle greater than allowed range.\n");
        exit(3);
    }
    if ((HEADING_SENSOR_ == 12) && ( (VAL<-50) || (VAL>40) )) {
        printf("Error: Update rate outside allowed range.\n");
        exit(3);
    }
    if ((HEADING_SENSOR_ == 13) && ( (VAL<0) || (VAL>16) )) {
        printf("Error: Average filter outside allowed range.\n");
        exit(3);
    }
    if ((HEADING_SENSOR_ == 14) && ( (VAL<-128) || (VAL>127) )) {
        printf("Error: Temperature offset outside allowed range.\n");
        exit(3);
    }
    if ((HEADING_SENSOR_ == 15) && ( (VAL<0) || (VAL>15) )) {
        printf("Error: Decimation filter value outside allowed range.\n");
        exit(3); 
    }
    
    unsigned char message[] = {0x0D, 0x06, HEADING_SENSOR_, (unsigned char)(VAL), (unsigned char)(VAL>>8), 0x00};
    ssize_t n = write(usbSerial, message, sizeof(message)/sizeof(message[0])); // Get no confirmation write was successful
    if (n < 0) {
        printf("Error: headingSensorSetting write failure.\n");
        exit(3);
    }
*/    
    unsigned char message[] = {0x0D, 0x06, 12, 40, 0, 0x00};
    message[sizeof(message) - 1] = seabotix_checksum(message, sizeof(message) - 1);
	acfr_sensor_write(state->sensor, (char *)message, sizeof(message));

    return 1;
}

float IEEE754toMCHP(float f) {
    
    float newfloat = f;
    unsigned char *p = (unsigned char *)&newfloat;
    
    // get value of sign
    unsigned char sign = p[3]&(1<<7);
    
    // move MSB across by one to the left
    p[3] <<= 1;
    
    // move one bit from byte 2 to byte 3 - clear bit in byte 3, then OR it with MSb in byte 2
    p[3] &= ~0x01;
    p[3] = p[3] | ((p[2] & (1<<7)>>7));
    
    // Set sign as MSb of byte 2 - clear MSb in byte 2, then OR it with sign
    p[2] &= ~(1<<7);
    p[2] |= sign;
    
    return newfloat;
}

int set_depth(state_t *state)
{

    // can't bitshift floats, so convert to an unsigned int first by copying it whole (assuming sizeof(int) == sizeof(float))
    float mchp_depth = IEEE754toMCHP(5000);
    unsigned int ui;
    memcpy(&ui, &mchp_depth, sizeof (ui));

    unsigned char message[] = {0x0E, 0x08, 1, (unsigned char)(ui), (unsigned char)(ui>>8), (unsigned char)(ui>>16), (unsigned char)(ui>>24), 0x00};
    message[sizeof(message) - 1] = seabotix_checksum(message, sizeof(message) - 1);
	acfr_sensor_write(state->sensor, (char *)message, sizeof(message));

    return 1;
}

// Send a request for the sensors every second
void heartbeat_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
    
    
    // send an MT-09 to get it all started
    unsigned char message_09[] = {0x09, 14, OPERATOR_AUTO_DEPTH, OPERATOR_AUTO_HEADING, OPERATOR_TRIM, 
        state->vertical_gain, state->horizontal_gain, 0x32, 0x38, 0x39, 0x35, 0x00, 0x00, 0x00};
    message_09[sizeof(message_09) - 1] = seabotix_checksum(message_09, sizeof(message_09) - 1);
	acfr_sensor_write(state->sensor, (char *)message_09, sizeof(message_09));
    
    // send an MT-26 message
    unsigned char message[] = {0x14, 0x04, 0x25, 0x00};
    message[3] = seabotix_checksum(message, 3);
    acfr_sensor_write(state->sensor, (char *)message, 4);
}

// Commands need to be sent to the ROV at 10Hz
void heartbeat_10Hz_handler(const lcm_recv_buf_t *rbuf, const char *ch, const perllcm_heartbeat_t *hb, void *u)
{
    state_t *state = (state_t *)u;
	// Send the joystick command if the command is still valid, 2 second timeout
	
		if(hb->utime - state->joystick.utime < 2e6)
		{
			unsigned char message[] = {0x0B, 0x0B, 
				(unsigned char)(state->	joystick.x), (unsigned char)(state->joystick.x>>8), 
				(unsigned char)(state->joystick.y), (unsigned char)(state->joystick.y>>8), 
				(unsigned char)(state->joystick.z), (unsigned char)(state->joystick.z>>8), 
				(unsigned char)(state->joystick.v), (unsigned char)(state->joystick.v>>8), 
				0x00};
			message[sizeof(message) - 1] = seabotix_checksum(message, sizeof(message) - 1);
			acfr_sensor_write(state->sensor, (char *)message, sizeof(message));
		}

}

// Joystick LCM command, just make a copy of it
void seabotix_joystick_handler(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_seabotix_joystick_t *sj, void *u)
{
    state_t *state = (state_t *)u;
	memcpy(&state->joystick, sj, sizeof(acfrlcm_seabotix_joystick_t));
}

int parse_mt20(const unsigned char *d, lcm_t *lcm, int64_t timestamp)
{
    printf("ACK, message MT-%02X\n", d[2] & 0xFF);
    
    return 1;
}
    

int parse_mt25(const unsigned char *d, lcm_t *lcm, int64_t timestamp)
{
    senlcm_seabotix_sensors_t rov;
    memset(&rov, 0, sizeof(senlcm_seabotix_sensors_t));

    for(int i=0; i<32; i++)
        printf("0x%02X ", d[i] & 0xFF);
    printf("\n");


    rov.utime = timestamp;
    rov.heading = d2f(&d[8]) * DTOR;
    rov.depth = d2f(&d[12]);
    rov.pitch = d2f(&d[16]) * DTOR;
    rov.roll = d2f(&d[20]) * DTOR;
    rov.turns = d2f(&d[24]);
    rov.temperature_internal = d2f(&d[28]);
    rov.temperature_external = d2f(&d[32]);

    rov.PF_faults = d[36];
    rov.PF_current = d2current(d[37]);
    rov.PF_speed = d2speed(d[38]);
    rov.PF_temperature = d[39];

    rov.PA_faults = d[40];
    rov.PA_current = d2current(d[41]);
    rov.PA_speed = d2speed(d[42]);
    rov.PA_temperature = d[43];

    rov.PV_faults = d[44];
    rov.PV_current = d2current(d[45]);
    rov.PV_speed = d2speed(d[46]);
    rov.PV_temperature = d[47];

    rov.SF_faults = d[48];
    rov.SF_current = d2current(d[49]);
    rov.SF_speed = d2speed(d[50]);
    rov.SF_temperature = d[51];

    rov.SA_faults = d[52];
    rov.SA_current = d2current(d[53]);
    rov.SA_speed = d2speed(d[54]);
    rov.SA_temperature = d[55];

    rov.SV_faults = d[56];
    rov.SV_current = d2current(d[57]);
    rov.SV_speed = d2speed(d[58]);
    rov.SV_temperature = d[59];

    senlcm_seabotix_sensors_t_publish(lcm, "SEABOTIX_SENSORS", &rov);

    return 1;
}


int program_exit;
int broken_pipe;
void
signal_handler(int sig_num)
{
    printf("Got a signal\n");
    // do a safe exit
    if(sig_num == SIGPIPE)
        broken_pipe = 1;
    else if (sig_num == SIGTERM)
        broken_pipe = 1;
    else if(sig_num == SIGINT)
        program_exit = 1;
}

int
main (int argc, char *argv[])
{
    // install the signal handler
    program_exit = 0;
    broken_pipe = 0;

    struct sigaction sa;
    sa.sa_flags = 0;
    sigemptyset (&sa.sa_mask);
    sigaddset(&sa.sa_mask, SIGPIPE);
    sigaddset(&sa.sa_mask, SIGHUP);
    sigaddset(&sa.sa_mask, SIGTERM);
    sigaddset(&sa.sa_mask, SIGINT);
    sa.sa_handler = SIG_IGN;
    if (sigaction(SIGPIPE, &sa, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }
    sa.sa_handler = signal_handler;
    sigaction(SIGHUP, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);


    //Initalise LCM object
    state_t state;
    state.lcm = lcm_create(NULL);

    char rootkey[64];
    sprintf(rootkey, "sensors.%s", basename(argv[0]));

    state.sensor = acfr_sensor_create(state.lcm, rootkey);
    if(state.sensor == NULL)
        return 0;

    acfr_sensor_noncanonical(state.sensor, 1, 0);

	// Read the addtional config variables
    char key[64];
    sprintf(key, "%s.vertical_gain", rootkey);
    printf("%s\n", key);
    state.vertical_gain = bot_param_get_int_or_fail(state.sensor->param, key);
    sprintf(key, "%s.horizontal_gain", rootkey);
    state.horizontal_gain = bot_param_get_int_or_fail(state.sensor->param, key);
    
    // LCM subscriptions
    perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_1HZ", &heartbeat_handler, &state);    
	perllcm_heartbeat_t_subscribe(state.lcm, "HEARTBEAT_10HZ", &heartbeat_10Hz_handler, &state);
	acfrlcm_seabotix_joystick_t_subscribe(state.lcm, "SEABOTIX_JOYSTICK", &seabotix_joystick_handler, &state);


    // send an MT-09 to get it all started
    unsigned char message[] = {0x09, 14, OPERATOR_AUTO_DEPTH, OPERATOR_AUTO_HEADING, OPERATOR_TRIM, 
        state.vertical_gain, state.horizontal_gain, 0x32, 0x38, 0x39, 0x35, 0x00, 0x00, 0x00};
    message[sizeof(message) - 1] = seabotix_checksum(message, sizeof(message) - 1);
	acfr_sensor_write(state.sensor, (char *)message, sizeof(message));

    printf("Sent MT-09 command\n");

    set_heading(&state);
    set_depth(&state);
    
    int len;
    char buf[256];

    int64_t timestamp;

    fd_set rfds;
    int lcm_fd = lcm_get_fileno(state.lcm);

    while(!program_exit)
    {
        // check for broken pipes, if it is broken make sure it is closed and then reopen it
        if(broken_pipe)
        {
            //sensor->port_open = 0;
            fprintf(stderr, "Pipe broken\n");
            continue;
        }

        memset(buf, 0, sizeof(buf));

        FD_ZERO(&rfds);
        FD_SET(state.sensor->fd, &rfds);
        FD_SET(lcm_fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
        {
            if(FD_ISSET(lcm_fd, &rfds))
                lcm_handle(state.lcm);
            else if(FD_ISSET(state.sensor->fd, &rfds))
            {
                
                timestamp = timestamp_now();
                
                len = acfr_sensor_read(state.sensor, buf, 2);
                
                // second byte conatins the number of bytes in the message
                len += acfr_sensor_read(state.sensor, &buf[2], buf[1] - 2);
               
                // check the length
                if(len != buf[1])
                {
                    fprintf(stderr, "Incorrect packet length, got %d, expected %d, message type MT-%02X\n", len, buf[1], buf[0] & 0xFF);
                    continue;
                }

                // check the checksum
                unsigned char checksum = seabotix_checksum((unsigned char *)buf, len - 1);
                if((checksum & 0xFF) != (buf[len - 1] & 0xFF))
                {
                    fprintf(stderr, "Check sum error, expected 0x%02X, got 0x%02X\n", checksum & 0xFF, buf[len - 1] & 0xFF);
                    continue;
                }

                // parse the message
                switch(buf[0])
                {
                case 0x25:
                    parse_mt25((unsigned char *)buf, state.lcm, timestamp);
                    break;
                case 0x20:
                    parse_mt20((unsigned char *)buf, state.lcm, timestamp);
                    break;
                default:
                    printf("Unsupported message 0x%02X\n", buf[0] & 0xFF);
                    break;
                }
            }
        }
    }

    acfr_sensor_destroy(state.sensor);
    lcm_destroy(state.lcm);

    return 0;

}



