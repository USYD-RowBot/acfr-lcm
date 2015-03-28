/*
    Payload comms to wave-glider C&C box

    Lachlan Toohey
    ACFR
    25-3-15

    Based on siris_motors
    by Christian Lees
*/

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <libgen.h>
#include <pthread.h>
#include <sys/select.h>

#include "perls-common/serial.h"
#include <bot_param/param_client.h>
#include "perls-lcmtypes/perllcm_heartbeat_t.h"
#include "perls-lcmtypes/senlcm_usbl_fix_short_t.h"
#include "perls-lcmtypes/acfrlcm_ship_status_t.h"
#include "perls-lcmtypes/acfrlcm_auv_status_short_t.h"
#include "acfr-common/sensor.h"

// this should be the largest chunk of data *BEFORE* including
// escaped characters
#define MAX_MESSAGE_SIZE 557

typedef struct {
    pthread_mutex_t lock;
    char *message;
    size_t length;
    int64_t last_time;
} message_channel_t;

void channel_init(message_channel_t *channel) {
    channel->message = 0;
    channel->length = 0;
    channel->last_time = 0;
}

typedef struct
{
    acfr_sensor_t *sensor;
    // serial port stuff
    char read_buffer[MAX_MESSAGE_SIZE*2];
    char write_buffer[MAX_MESSAGE_SIZE];
    size_t data_end;
    size_t read_cursor;
    size_t write_cursor;

    pthread_mutex_t queue_lock;
    message_channel_t queue[3];

    // lcm stuff
    lcm_t *lcm;


} state_t;

void
send_motor_command(state_t *state, double vertical, double port, double starboard)
{
   // send a command to the motors
    char animatics_str[MAX_MESSAGE_SIZE];
    int thr_ref, len;

    // from animatics user manual p46
    // the 32212 is for the motor, 32.5 is for the gear box, 60 is from RPM to Hz
    // compose animatics command string for velocity mode
    /*
    thr_ref = (int)(vertical * 32212.0 * 32.5 / 60.0);
    len = sprintf(animatics_str, "MV A=800 V=%i G t=0\n",thr_ref);
    write(state->vert_fd, animatics_str, len);


    usleep(5000);
    write(state->vert_fd, ANIMATICS_PRINT_STRING, strlen(ANIMATICS_PRINT_STRING));
    write(state->port_fd, ANIMATICS_PRINT_STRING, strlen(ANIMATICS_PRINT_STRING));
    write(state->starb_fd, ANIMATICS_PRINT_STRING, strlen(ANIMATICS_PRINT_STRING));
    */

}

void
usbl_callback(const lcm_recv_buf_t *rbuf, const char *ch, const senlcm_usbl_fix_short_t *usbl, void *u)
{
    message_channel_t *channel = (message_channel_t *)u;

    // calculate the string to send here

    pthread_mutex_lock(&channel->lock);
    pthread_mutex_unlock(&channel->lock);
}

void
auv_status_callback(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_auv_status_short_t *status, void *u)
{
    message_channel_t *channel = (message_channel_t *)u;

    // calculate the string to send here

    pthread_mutex_lock(&channel->lock);
    pthread_mutex_unlock(&channel->lock);
}

void
ship_status_callback(const lcm_recv_buf_t *rbuf, const char *ch, const acfrlcm_ship_status_t *status, void *u)
{
    message_channel_t *channel = (message_channel_t *)u;

    // calculate the string to send here

    pthread_mutex_lock(&channel->lock);
    pthread_mutex_unlock(&channel->lock);
}

int program_exit;
void
signal_handler(int sigNum)
{
    // do a safe exit
    program_exit = 1;
}

uint16_t checksum_next(uint16_t crc, char c) {

    uint8_t ii;

    crc ^= c;

    for (ii=0;ii<8;++ii) {
        if ((crc & 1) == 1) {
            crc = (crc >> 1) ^ 0xA001;
        } else {
            crc = (crc >> 1);
        }
    }

    return crc;
}

uint16_t checksum(char *buffer, size_t length, uint16_t seed) {
    uint16_t crc = seed;

    size_t ii;
    for (ii = 0; ii < length; ++ii) {
        crc = checksum_next(crc, buffer[ii]);
    }

    return crc;
}


size_t cleanse_next(state_t *state) {
    switch (state->read_buffer[state->read_cursor]) {
        case 0x7E:
            // new frame !?
            return 0;
        case 0x7D:
            if (state->data_end == state->read_cursor + 1 ) {
                // this is the hardest case... we need 1 more byte...
                // so we should probably just get it
                // if it screws up then just abandon it all however...
                ssize_t ret;
                ret = acfr_sensor_read(state->sensor, state->read_buffer + state->data_end, 1);
                state->data_end += ret;

                if (ret < 1) {
                    return 0;
                }
            }
            // read the next character (it may
            if (state->read_buffer[1] == 0x5D) {
                state->write_buffer[0] = 0x7D;
            } else if (state->read_buffer[1] == 0x5E) {
                state->write_buffer[0] = 0x7E;
            } else {
                return 0;
            }

            state->read_cursor += 2;
            state->write_cursor += 1;

            return 2;
        default:
            state->write_buffer[0] = state->read_buffer[0];
            state->read_cursor += 1;
            state->write_cursor += 1;
            return 1;
    }
}

bool read_packet(state_t *state) {
    // now get the size, need to read enough bytes to make sure

    ssize_t ret;

    while (state->data_end < 6) {
        ret = acfr_sensor_read(state->sensor, state->read_buffer + state->data_end, 6 - state->data_end);
        // any error, bail out
        if (ret <= 0) return false;
        state->data_end += ret; // add bytes read
    }

    printf("Checking for SOF character\n");

    // check that packet starts with what we want it to
    if (state->read_buffer[0] != 0x7E) {
        return false;
    }

    if (cleanse_next(state) == 0) return false;

    printf("Cleansing header characters\n");

    // lets get to it,
    uint16_t length = *(uint16_t *)(state->read_buffer + 1);

    printf("Packet length: %d\n", length);

    while (state->write_cursor < length + 1) {
        size_t next_read = length + 1 - state->data_end;
        ret = acfr_sensor_read(state->sensor, state->read_buffer + state->data_end, next_read);

        while (state->read_cursor < state->data_end) {
            if (cleanse_next(state) == 0) return false;
        }
    }

    // at this point we should have a valid packet
    // check the checksum...
    uint16_t crc = checksum(state->write_buffer + 1, state->write_cursor - 3, 0x0000);
    uint16_t packet_crc = *((uint16_t *)(state->write_buffer + length - 2));

    if (crc != packet_crc) return false;

    // ran the gauntlet and made it - valid packet has been found
    return true;
}


// listen to the responses from the motors and post the LCM messages
void *serial_listen_thread(void *u)
{
    state_t *state = (state_t *)u;

    // serial -> read, copy to write and de-escape characters
    state->data_end = 0; // end of currently read data
    // denote how far has been parsed in the message and where they are getting read from
    while(!program_exit)
    {
        printf("Looking for packet...\n");
        state->read_cursor = 0;
        state->write_cursor = 0;

        if (read_packet(state)) {
            // deal with the packet now
            printf("Got a packet!\n");
        } else {
            printf("Failed to read packet!\n");
            // failed to get a packet...
            // go through leftover data and see if we have a new frame
            // either way discard any data before at SOF character.
            while (state->read_cursor < state->data_end) {

                if (state->read_buffer[state->read_cursor] == 0x7E) {
                    printf("Found unexpected SOF character\n");
                    memmove(state->read_buffer, state->read_buffer + state->read_cursor, state->data_end - state->read_cursor);
                    break;
                }

                state->read_cursor++;
            }
            state->data_end -= state->read_cursor;
            state->read_cursor = 0;
        }
    }

    return NULL;
}

int
main(int argc, char **argv)
{
    state_t state;

    // install the signal handler
	program_exit = 0;
    signal(SIGINT, signal_handler);
    state.lcm = lcm_create(NULL);

    // read the config file
    char rootkey[64];
    sprintf(rootkey, "acfr.%s", basename(argv[0]));

    acfr_sensor_t *sensor = acfr_sensor_create(state.lcm, rootkey);
    if (sensor == NULL)
        return 0;

    char key[64];
    // vertical thruster info
    /*char *device;
    sprintf(key, "%s.device", rootkey);
    device = bot_param_get_str_or_fail(sensor->param, key);

    int baud;
    sprintf(key, "%s.baud", rootkey);
    baud = bot_param_get_int_or_fail(sensor->param, key);*/

    // lets start LCM
    senlcm_usbl_fix_short_t_subscribe(state.lcm, "USBL_FIX.*", &usbl_callback, &state.queue[0]);
    acfrlcm_auv_status_short_t_subscribe(state.lcm, "AUVSTAT.*", &auv_status_callback, &state.queue[1]);
    acfrlcm_ship_status_t_subscribe(state.lcm, "SHIP_STATUS.*", &ship_status_callback, &state.queue[2]);

    // start the listen threads
    pthread_t tid_cc;
    pthread_create(&tid_cc, NULL, serial_listen_thread, &state);
    pthread_detach(tid_cc);

	int lcm_fd = lcm_get_fileno(state.lcm);
	fd_set rfds;

	// listen to LCM, serial has been delegated
	while(!program_exit)
	{
	    FD_ZERO(&rfds);
        FD_SET(lcm_fd, &rfds);

		struct timeval tv;
		tv.tv_sec = 1;
		tv.tv_usec = 0;

	    int ret = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);
        if(ret == -1)
            perror("Select failure: ");
        else if(ret != 0)
            lcm_handle(state.lcm);


	}

    pthread_join(tid_cc, NULL);

    acfr_sensor_destroy(state.sensor);


	return 0;
}