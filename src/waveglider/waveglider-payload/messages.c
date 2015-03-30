#include "messages.h"
#include <stdio.h>

enum {
        INITIALISED = 0x01,
        PENDING_MESSAGES = 0x02,
        WAITING_QUEUE_RESPONSE = 0x04,
        SHUTTING_DOWN = 0x08
};

typedef struct _glider_state {
    int8_t flags; // initialised = 0x01, notified_queued_message = 0x02


} glider_state_t;

#define ACK_REQUIRED 0x8000


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

    switch (header->message_type)
    {
        case 0x0010:
            // request id/enumerate (broadcast)
            printf("Received enumeration request\n");
            enumerate_t *enumeration = (enumerate_t *)message;
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
            break;
        case 0x0024:
            // send/forward message (broadcast)
            
            break;
        case 0x0030:
            // power status and control (broadcast)
            break;
        case 0x0040:
            // request queued message
            break;
        case 0x0041:
            // ack/nak queued message
            break;
        default:
            // no idea what type of message this is
            break;
    }
}
