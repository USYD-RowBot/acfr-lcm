#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <semaphore.h>
#include <errno.h>
#include <zlib.h>
#include "base91.h"
#include "perls-common/timestamp.h"
#include "perls-lcmtypes/senlcm_evologics_usbl_t.h"
#include "perls-lcmtypes/senlcm_evologics_modem_t.h"
#include "perls-lcmtypes/senlcm_evologics_config_t.h"
#include "perls-lcmtypes/acfrlcm_auv_status_t.h"


#ifndef EVOLOGICS_H
#define EVOLOGICS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LCM_AUV_STATUS = 1,
    LCM_AUV_IMAGE,
    LCM_USBL_FIX,
    LCM_TASK_COMMAND
} lcm_mesg_t;

typedef enum
{
    PARSE_ERROR,
    PARSE_USBL,
    PARSE_DATA,
    PARSE_IM,
    PARSE_NONE
} parse_message_t;

typedef enum
{
    IO_SERIAL,
    IO_ENET
} interface_t;

typedef struct 
{
    lcm_t *lcm;
    int fd;
    int current_target;
    int channel_ready;
    int ping_semaphore[8];
    int targets[8];
    int num_targets;
    int sending_data;
    interface_t interface;
    senlcm_evologics_usbl_t *usbl;
} el_state_t;


int send_ping(int target, el_state_t *state);
int send_evologics_data(char *data, int size, int target, el_state_t *state);
parse_message_t parse_evologics_message(char *data, int len, el_state_t *state, int64_t timestamp);
int readline(int fd, char *buf, int max_len);
int send_evologics_command(char *data, char *ret_str, int size, el_state_t *state);
int chop_string(char *data, char **tokens);

#ifdef __cplusplus
}
#endif


#endif
