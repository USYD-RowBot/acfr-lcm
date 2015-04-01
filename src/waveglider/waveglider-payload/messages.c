#include "messages.h"
#include "checksum.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void glider_state_init(glider_state_t *glider)
{
    memset(glider, 0, sizeof(glider_state_t));
    glider->task_id = 0x13;
    glider->board_id = 0x37;
    glider->device_type = 0x3412;
    glider->poll_period = 60;
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

    if (glider->reply_length > 0)
    {
        printf("Should have already send reply to glider.\n");
    }

    if (header->message_type & ACK_REQUIRED)
    {
        suppress_ack = 1;
        // remove the bit to prevent confusing the switch block
        header->message_type &= ~ACK_REQUIRED;
    }

    switch (header->message_type)
    {
        case 0x0010:
            // request id/enumerate (broadcast)
            printf("Received enumeration request\n");
            enumerate_t *enumeration = (enumerate_t *)message;
            handle_enumerate(glider, enumeration, suppress_ack);
            break;
        case 0x0015:
            // telemetry (broadcast)
            printf("Received telemetry notifcation\n");
            telemetry_t *telemetry = (telemetry_t *)message;
            break;
        case 0x0022:
            // request status
            printf("Received status request\n");
            request_status_t *request_status = (request_status_t *)message;
            handle_status(glider, request_status, suppress_ack);
            break;
        case 0x0024:
            // send/forward message (broadcast)
            
            break;
        case 0x0030:
            // power status and control (broadcast)
            printf("Received power status message\n");
            break;
        case 0x0040:
            // request queued message
            printf("Request queued message\n");
            request_queued_message_t *request = (request_queued_message_t *)message;
            handle_request_queued(glider, request, suppress_ack);
            break;
        case 0x0041:
            // ack/nak queued message
            printf("Response to queued message\n");
            response_queued_message_t *response = (response_queued_message_t *)message;
            handle_queued_response(glider, response, suppress_ack);
            break;
        default:
            // no idea what type of message this is
            break;
    }


    // now check if we are sending a message
    if (glider->reply_length > 0)
    {
        // flag that there is a message pending
        glider->buffer[12] |= 0x80;
    }
}

void handle_enumerate(glider_state_t *glider, enumerate_t *enumeration, int8_t suppress_ack)
{
    char *next_chunk = glider->buffer;

    enumeration_ack_t *response_header = (enumeration_ack_t *)next_chunk;
    next_chunk += sizeof(enumeration_ack_t);

    device_info_block_t *device_info = (device_info_block_t *)next_chunk;
    next_chunk += sizeof(device_info_block_t);

    uint16_t *checksum = (uint16_t *)next_chunk;
    next_chunk += sizeof(uint16_t);

    ssize_t length = next_chunk - glider->buffer;

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
    char *description = "ACFR Payload WGlider";

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
    *checksum = gen_crc16(glider->buffer + 1, length - 3);

    glider->reply_length = length;
}

void handle_status(glider_state_t *glider, request_status_t *request_status, int8_t suppress_ack)
{
    char *next_chunk = glider->buffer;

    status_ack_t *response_header = (status_ack_t *)next_chunk;
    next_chunk += sizeof(status_ack_t);

    status_block_t *status = (status_block_t *)next_chunk;
    next_chunk += sizeof(status_block_t);

    uint16_t *checksum = (uint16_t *)next_chunk;
    next_chunk += sizeof(uint16_t);

    ssize_t length = next_chunk - glider->buffer;

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

    *checksum = gen_crc16(glider->buffer + 1, length - 3);

    glider->reply_length = length;
}

void handle_request_queued(glider_state_t *glider, request_queued_message_t *request, int8_t suppress_ack)
{
    char *next_chunk = glider->buffer;

    ack_queued_message_t *response_header = (ack_queued_message_t *)next_chunk;
    next_chunk += sizeof(ack_queued_message_t);

    memcpy(next_chunk, glider->queued_message_buffer, glider->queued_message_length); 
    next_chunk += glider->queued_message_length;

    uint16_t *checksum = (uint16_t *)next_chunk;
    next_chunk += sizeof(uint16_t);

    ssize_t length = next_chunk - glider->buffer;

    response_header->header.sof = 0x7e;
    response_header->header.length = length - 1; // exclude sof char
    response_header->header.destination_task = 0x04;
    response_header->header.destination_board = 0x00;
    response_header->header.source_task = glider->task_id;
    response_header->header.source_board = glider->board_id;
    response_header->header.transaction_id = request->header.transaction_id;
    response_header->header.message_type = 1;

    response_header->source_message_type = request->header.message_type;
    response_header->message_format = 1;

    // already copied in the message

    *checksum = gen_crc16(glider->buffer + 1, length - 3);

    glider->reply_length = length;
}


// we only ever talk to the iridium/ XBee
void handle_queued_response(glider_state_t* glider, response_queued_message_t *response, int8_t suppress_ack)
{
    char *next_chunk = glider->buffer;

    general_ack_t *response_header = (general_ack_t *)next_chunk;
    next_chunk += sizeof(ack_queued_message_t);

    uint16_t *checksum = (uint16_t *)next_chunk;
    next_chunk += sizeof(uint16_t);

    ssize_t length = next_chunk - glider->buffer;

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

    *checksum = gen_crc16(glider->buffer + 1, length - 3);

    glider->reply_length = length;
}
