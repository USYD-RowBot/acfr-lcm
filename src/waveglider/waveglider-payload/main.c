/*
    Payload comms to wave-glider C&C box

    Lachlan Toohey
    ACFR
    25-3-15
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

#include "checksum.h"
#include "messages.h"

// this should be the largest chunk of data *BEFORE* including
// escaped characters

typedef struct {
    uint8_t type; // ==0x91
    uint32_t transaction_id;
    uint16_t size;
    uint32_t payload_type;
    uint8_t board_id;
    uint8_t type_id;
    uint8_t flags;
    uint8_t repeats;
    int32_t latitude;
    int32_t longitude;
    uint32_t timestamp;
} telemetry_header_t;

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
    pthread_mutex_init(&channel->lock, 0);
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

    char transmit_buffer[MAX_MESSAGE_SIZE*2];
    size_t transmit_length;

    glider_state_t glider;

    pthread_mutex_t queue_lock;
    message_channel_t queue[3];

    // lcm stuff
    lcm_t *lcm;


} state_t;


void
send_message(state_t *state)
{
    acfr_sensor_write(state->sensor, state->transmit_buffer, state->transmit_length);
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
    size_t data_length = sizeof(acfrlcm_ship_status_t) + sizeof(telemetry_header_t) + 2; // add the CRC
    size_t data_chunk = sizeof(acfrlcm_ship_status_t) + 12 + 4 + 4;

    pthread_mutex_lock(&channel->lock);
    channel->message = malloc(data_length);

    // get the header data
    telemetry_header_t *header = (telemetry_header_t *)channel->message;
    // get the actual data we care about
    memcpy(channel->message + sizeof(telemetry_header_t), status, sizeof(acfrlcm_ship_status_t));
    // and finally the checksum location
    uint16_t *checksum = ((uint16_t *)(channel->message + data_length));

    header->type = 0x91;
    header->transaction_id = 0;
    header->size = data_length - 7;
    header->payload_type = 0; // NFI what is going on here
    header->board_id = 0x37;
    header->type_id = 0x13;
    header->flags = 0x00;
    header->repeats = 1;
    header->latitude = (uint32_t)(status->latitude * 600000);
    header->longitude = (uint32_t)(status->longitude * 600000);
    header->timestamp = time(NULL);

    *checksum = gen_crc16(channel->message, data_length - 2);

    pthread_mutex_unlock(&channel->lock);
}

int program_exit;
void
signal_handler(int sigNum)
{
    // do a safe exit
    program_exit = 1;
}

void escape_packet(state_t *state, char *buffer, size_t length) {
    size_t rc = 1;
    size_t wc = 1;

    state->transmit_buffer[0] = buffer[0];
    // == 0x7E;

    for (; rc < length;++rc)
    {
        switch (buffer[rc])
        {
        case 0x7E:
            state->transmit_buffer[wc++] = 0x7D;
            state->transmit_buffer[wc++] = 0x5E;
            break;
        case 0x7D:
            state->transmit_buffer[wc++] = 0x7D;
            state->transmit_buffer[wc++] = 0x5D;
            break;
        default:
            state->transmit_buffer[wc++] = buffer[rc];

        }
    }
    state->transmit_length = wc;
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
            char next = state->read_buffer[state->read_cursor+1];
            if (next == 0x5D) {
                state->write_buffer[state->write_cursor] = 0x7D;
            } else if (next == 0x5E) {
                state->write_buffer[state->write_cursor] = 0x7E;
            } else {
                return 0;
            }

            state->read_cursor += 2;
            state->write_cursor += 1;

            return 2;
        default:
            state->write_buffer[state->write_cursor] = state->read_buffer[state->read_cursor];
            state->read_cursor += 1;
            state->write_cursor += 1;
            return 1;
    }
}

int read_packet(state_t *state) {
    // now get the size, need to read enough bytes to make sure

    ssize_t ret;

    while (state->data_end < 6) {
        ret = acfr_sensor_read(state->sensor, state->read_buffer + state->data_end, 6 - state->data_end);
        printf("Read bytes... %ld\n", ret);
        // any error, bail out
        if (ret <= 0) return 0;
        state->data_end += ret; // add bytes read
    }

    printf("Checking for SOF character\n");

    // check that packet starts with what we want it to
    if (state->read_buffer[0] != 0x7E) {
        return 0;
    }

    // in case we have already read some data
    if (state->read_cursor == 0) {
        state->read_cursor = 1;
    }

    if (state->write_cursor == 0) {
        state->write_cursor = 1;
        state->write_buffer[0] = 0x7E;
    }

    // cleanse the length bytes
    if (cleanse_next(state) == 0) return 0;
    if (cleanse_next(state) == 0) return 0;

    printf("Cleansing header characters\n");

    // lets get to it,
    uint16_t length = *(uint16_t *)(state->write_buffer + 1);

    printf("Packet length: %d\n", length);

    while (state->write_cursor < length + 1) {
        size_t next_read = 1 + length - state->write_cursor - state->data_end + state->read_cursor;
        ret = acfr_sensor_read(state->sensor, state->read_buffer + state->data_end, next_read);
        state->data_end += ret;

        while (state->read_cursor < state->data_end) {
            if (cleanse_next(state) == 0) return 0;
        }
    }

    // at this point we should have a valid packet
    // check the checksum...
    uint16_t crc = gen_crc16(state->write_buffer + 1, state->write_cursor - 1);

    if (crc != 0)
    {
        printf("CRC failure\n");
        return 0;
    }

    // ran the gauntlet and made it - valid packet has been found
    return 1;
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
        // always reread from the start
        // but there may alreadt be data in the read buffer
        // so don't discard it
        state->read_cursor = 0;
        state->write_cursor = 0;

        if (read_packet(state)) {
            // deal with the packet now
            printf("Got a packet!\n");
            handle_packet(&state->glider, state->write_buffer, state->write_cursor);
            if (state->glider.reply_length > 0)
            {
                // we have a message to send
                escape_packet(state, state->glider.buffer, state->glider.reply_length);
                send_message(state);
            }
            state->data_end = 0; // reset to start of buffer
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
    glider_state_init(&state.glider);
    pthread_mutex_init(&state.queue_lock, 0);

    // install the signal handler
	program_exit = 0;
    signal(SIGINT, signal_handler);
    state.lcm = lcm_create(NULL);

    // read the config file
    char rootkey[64];
    sprintf(rootkey, "acfr.%s", basename(argv[0]));

    state.sensor = acfr_sensor_create(state.lcm, rootkey);
    if (state.sensor == NULL)
        return 0;

    acfr_sensor_noncanonical(state.sensor, 0, 1);

    char key[64];
    /*char *device;
    sprintf(key, "%s.device", rootkey);
    device = bot_param_get_str_or_fail(state.sensor->param, key);

    int baud;
    sprintf(key, "%s.baud", rootkey);
    baud = bot_param_get_int_or_fail(state.sensor->param, key);*/

    // lets start LCM
    senlcm_usbl_fix_short_t_subscribe(state.lcm, "USBL_FIX.*", &usbl_callback, &state.queue[0]);
    acfrlcm_auv_status_short_t_subscribe(state.lcm, "AUVSTAT.*", &auv_status_callback, &state.queue[1]);
    acfrlcm_ship_status_t_subscribe(state.lcm, "SHIP_STATUS.W.*", &ship_status_callback, &state.queue[2]);

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
