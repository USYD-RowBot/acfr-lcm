#ifndef __MESSAGES_H__
#define __MESSAGES_H__

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _glider_state glider_state_t;

typedef struct {
    uint8_t sof;
    uint16_t length;
    uint8_t destination_task;
    uint8_t destination_board;
    uint8_t source_task;
    uint8_t source_board;
    uint16_t transaction_id;
    uint16_t message_type;
} header_t;

// Enumerate 0x0010
typedef struct {
    header_t header;
    uint16_t protocols;
    uint16_t command_formats;
} enumerate_t;

// ACK 0x0001
typedef struct {
    header_t header;
    uint16_t source_message_type;
    uint16_t responses_total;
    uint16_t responses_present;
} general_ack_t;

typedef struct {
    uint16_t format;
    uint16_t type;
    uint16_t device_address;
    uint8_t serial[6];
    uint8_t port;
    uint32_t poll_period; // seconds
    uint8_t info_flags;
    uint8_t firmware_major;
    uint8_t firmware_mainor;
    uint16_t firmware_revision;
    uint8_t desciption[20];
    uint8_t padding[8]; // set to zero ?!? not actually used.
} device_info_block_t;


// version and address must be real, but all else can be 0's.
typedef struct {
    uint16_t version;
    uint16_t device_address;
    uint16_t alarm_flags;
    uint8_t leak_sensor1;
    uint8_t leak_sensor2;
    uint8_t temperature_humidity; // \deg C from humidity sensor
    uint8_t humidity; // per cent
    uint8_t temperature_pressure; // \deg C from pressure sensor
    uint8_t pressue; // kPa
} status_block_t;


// request queued message 0x0040
typedef struct {
    header_t header;
} request_queued_message_t;

// nak response to request queued message
// ie the data disappeared...
typedef struct {
    header_t header;
    uint16_t source_message_type;
    uint8_t invalid_flag; // == 0x02
    uint16_t length; // == 0x0000
    uint16_t checksum;
} nak_queued_message_t;

// ack response to request queued message
// follow directly with data & checksum
typedef struct {
    header_t header;
    uint16_t source_message_type;
} ack_queued_message;

// ACK/NAK Queued Message 0x0041
// immediately followed by response message (may be none)
// then the checksum
typedef struct {
    header_t header;
    uint16_t command_format; // == 0x0001
    uint16_t response_code; // 0 = ACK
    // 1 = unknown command
    // 2 = invalid data
    // 3 = unable to complete
    // 4 = unknown format
} response_queued_message_t;

// C&C telemetry 0x0015
// followed by at most 544 bytes and then checksum
typedef struct {
    header_t header;
    uint8_t structure_id;
} telemetry_t;

// Request Status 0x0022
typedef struct {
    header_t header;
    uint8_t time_utc[8]; // second, minute, hour, day of month, month, years since 2000
    uint32_t latitude; // decimal minutes
    uint32_t longitude; // decimal minutes
    uint8_t fix_valid; // 'Y' or 'N' depending on GPS validity
} request_status_t;

// Send/forward message 0x0024
// followed by date (<= 541 bytes) and checksum
typedef struct {
    header_t header;
    uint8_t flag; // 0 = forward, 1 = send
    uint16_t command_format; // 1 = vehicle to remote
} sendforward_message_t;

#ifdef __cplusplus
};
#endif

#endif // __MESSAGES_H__
