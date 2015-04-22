#include "messages.h"
#include "checksum.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

void glider_state_init(glider_state_t *glider)
{
    memset(glider, 0, sizeof(glider_state_t));
    glider->task_id = 0x13;
    glider->board_id = 0x37;
    glider->device_type = 0x3412;
    glider->poll_period = 30;
}

#define ACK_REQUIRED 0x8000

void handle_enumerate(glider_state_t *glider, enumerate_t *enumeration, int8_t suppress);
void handle_status(glider_state_t* glider, request_status_t *request_status, int8_t suppress);
void handle_request_queued(glider_state_t *glider, request_queued_message_t *request, int8_t suppress_ack);
void handle_queued_response(glider_state_t* glider, response_queued_message_t *response, int8_t suppress_ack);

// handle a complete and validated message
void handle_packet(glider_state_t *glider, char *message, size_t length)
{
    // convert to make it easier to read
    header_t *header = (header_t *)message;

    int8_t suppress_ack = 0;

    if (header->message_type & ACK_REQUIRED)
    {
        suppress_ack = 1;
        // remove the bit to prevent confusing the switch block
        header->message_type &= ~ACK_REQUIRED;
    }

    FILE *f = fopen("/tmp/messagelog.bin", "ab");
    fwrite(message, 1, length, f);
    fclose(f);

    printf("%d: Packet Type (0x%X)\n", time(0), header->message_type);
    switch (header->message_type)
    {
        case 0x0010:
            // request id/enumerate (broadcast)
            printf("%d: received enumeration request\n", time(0));
            enumerate_t *enumeration = (enumerate_t *)message;
            handle_enumerate(glider, enumeration, suppress_ack);
            break;
        case 0x0015:
            // telemetry (broadcast)
            printf("%d: received telemetry notifcation\n", time(0));
            telemetry_t *telemetry = (telemetry_t *)message;
            break;
        case 0x0022:
            // request status
            printf("%d: received status request\n", time(0));
            request_status_t *request_status = (request_status_t *)message;
            handle_status(glider, request_status, suppress_ack);
            break;
        case 0x0024:
            // send/forward message (broadcast)
            
            break;
        case 0x0030:
            // power status and control (broadcast)
            printf("%d: received power status message\n", time(0));
            break;
        case 0x0040:
            // request queued message
            printf("%d: request for queued message\n", time(0));
            request_queued_message_t *request = (request_queued_message_t *)message;
            handle_request_queued(glider, request, suppress_ack);
            break;
        case 0x0041:
            // ack/nak queued message
            printf("%d: received response to queued message\n", time(0));
            response_queued_message_t *response = (response_queued_message_t *)message;
            handle_queued_response(glider, response, suppress_ack);
            break;
        default:
            // no idea what type of message this is
            printf("%d: unknown message type.\n", time(0));
            break;
    }


    // now check if we are sending a message, and if there is another message to send
    if (glider->pending_data_length > 0 && glider->output_length)
    {
        printf("%d: Indicated presence of queued message to C&C\n", time(0));
        // flag that there is a message pending
        glider->output_data[12] |= 0x80;
        // and recalculate the checksum
        *((uint16_t *)(glider->output_data + glider->output_length-2)) = gen_crc16(glider->output_data + 1, glider->output_length - 3);
    }
}

void handle_enumerate(glider_state_t *glider, enumerate_t *enumeration, int8_t suppress_ack)
{
    char *next_chunk = glider->output_data;

    enumeration_ack_t *response_header = (enumeration_ack_t *)next_chunk;
    next_chunk += sizeof(enumeration_ack_t);

    device_info_block_t *device_info = (device_info_block_t *)next_chunk;
    next_chunk += sizeof(device_info_block_t);

    uint16_t *checksum = (uint16_t *)next_chunk;
    next_chunk += sizeof(uint16_t);

    ssize_t length = next_chunk - glider->output_data;

    response_header->header.sof = 0x7e;
    response_header->header.length = length - 1; // exclude sof char
    response_header->header.destination_task = enumeration->header.source_task;
    response_header->header.destination_board = enumeration->header.source_board;
    response_header->header.source_task = 0x13;
    response_header->header.source_board = 0x37;
    response_header->header.transaction_id = enumeration->header.transaction_id;
    response_header->header.message_type = 1;

    response_header->source_message_type = enumeration->header.message_type;
    response_header->responding_devices = 1;
    response_header->devices_present = 1;

    // device info
    uint8_t serial[] = {1, 1, 2, 3, 5, 8};
    char *description = "ACFR                ";

    device_info->format = 1;
    device_info->type = glider->device_type;
    device_info->task_id = glider->task_id;
    device_info->board_id = glider->board_id;
    memcpy(device_info->serial, serial, 6);
    device_info->port = 0;
    device_info->poll_period = glider->poll_period; // 5 minutes
    device_info->info_flags = 0x02; // want power information ie shutdown I think
    device_info->firmware_major = 1;
    device_info->firmware_minor = 0;
    device_info->firmware_revision = 1;
    memcpy(device_info->description, description, 20);
    memset(device_info->padding, 0, 8);


    // finally, the checksum
    *checksum = gen_crc16(glider->output_data + 1, length - 3);

    glider->output_length = length;
}

void handle_status(glider_state_t *glider, request_status_t *request_status, int8_t suppress_ack)
{
    char *next_chunk = glider->output_data;

    status_ack_t *response_header = (status_ack_t *)next_chunk;
    next_chunk += sizeof(status_ack_t);

    status_block_t *status = (status_block_t *)next_chunk;
    next_chunk += sizeof(status_block_t);

    uint16_t *checksum = (uint16_t *)next_chunk;
    next_chunk += sizeof(uint16_t);

    ssize_t length = next_chunk - glider->output_data;

    response_header->header.sof = 0x7e;
    response_header->header.length = length - 1; // exclude sof char
    response_header->header.destination_task = request_status->header.source_task;
    response_header->header.destination_board = request_status->header.source_board;
    response_header->header.source_task = glider->task_id;
    response_header->header.source_board = glider->board_id;
    response_header->header.transaction_id = request_status->header.transaction_id;
    response_header->header.message_type = 1;

    response_header->source_message_type = request_status->header.message_type;
    response_header->total_responses = 1;
    response_header->responses_present = 1;

    status->version = 1;
    status->task_id = glider->task_id;
    status->board_id = glider->board_id;
    memset(&status->alarm_flags, 0, sizeof(status_block_t) - 4);

    *checksum = gen_crc16(glider->output_data + 1, length - 3);

    glider->output_length = length;
}

void handle_request_queued(glider_state_t *glider, request_queued_message_t *request, int8_t suppress_ack)
{
    char *next_chunk = glider->output_data;

    ack_queued_message_t *response_header = (ack_queued_message_t *)next_chunk;
    next_chunk += sizeof(ack_queued_message_t);

    memcpy(next_chunk, glider->pending_data, glider->pending_data_length); 
    next_chunk += glider->pending_data_length;
    glider->pending_data_length = 0;

    uint16_t *checksum = (uint16_t *)next_chunk;
    next_chunk += sizeof(uint16_t);

    ssize_t length = next_chunk - glider->output_data;

    response_header->header.sof = 0x7e;
    response_header->header.length = length - 1; // exclude sof char
    response_header->header.destination_task = 0x00;
    response_header->header.destination_board = 0x02;
    response_header->header.source_task = glider->task_id;
    response_header->header.source_board = glider->board_id;
    response_header->header.transaction_id = request->header.transaction_id;
    response_header->header.message_type = 1;

    response_header->source_message_type = request->header.message_type;
    response_header->message_format = 1;

    // already copied in the message

    *checksum = gen_crc16(glider->output_data + 1, length - 3);

    glider->output_length = length;
}


// we only ever talk to the iridium/ XBee
void handle_queued_response(glider_state_t* glider, response_queued_message_t *response, int8_t suppress_ack)
{
    char *next_chunk = glider->output_data;

    general_ack_t *response_header = (general_ack_t *)next_chunk;
    next_chunk += sizeof(ack_queued_message_t);

    uint16_t *checksum = (uint16_t *)next_chunk;
    next_chunk += sizeof(uint16_t);

    ssize_t length = next_chunk - glider->output_data;

    response_header->header.sof = 0x7e;
    response_header->header.length = length - 1; // exclude sof char
    response_header->header.destination_task = response->header.source_task;
    response_header->header.destination_board = response->header.source_board;
    response_header->header.source_task = glider->task_id;
    response_header->header.source_board = glider->board_id;
    response_header->header.transaction_id = response->header.transaction_id;
    response_header->header.message_type = 1;

    response_header->source_message_type = response->header.message_type;
    response_header->command_format = 1;

    // already copied in the message

    *checksum = gen_crc16(glider->output_data + 1, length - 3);

    glider->output_length = length;
}
