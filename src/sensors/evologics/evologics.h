#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <semaphore.h>
#include <errno.h>
#include "base91.h"
#include "perls-lcmtypes/senlcm_evologics_usbl_t.h"
#include "perls-lcmtypes/senlcm_evologics_modem_t.h"
#include "perls-lcmtypes/senlcm_evologics_config_t.h"
#include "perls-lcmtypes/acfrlcm_auv_status_t.h"
#include "perls-lcmtypes/senlcm_novatel_t.h"
#include "perls-lcmtypes/senlcm_usbl_fix_t.h"


typedef enum
{
    LCM_AUV_STATUS = 1,
    LCM_AUV_IMAGE
} lcm_mesg_t;
 
typedef enum 
{
    NOVATEL,
    INTERNAL
} attitude_source_t;  
 
enum {io_socket, io_serial};   

typedef struct
{
    lcm_t *lcm;
    int fd;
    int ping_counter;
    int current_target;
    int targets[8];
    int num_targets;
    char term;
    int ping_sem[8];
    int channel_ready;
    
    attitude_source_t attitude_source;
    senlcm_novatel_t *novatel;
} state_t;

int send_ping(int target, state_t *state);
int send_evologics_data(char *data, int size, int target, state_t *state);
int parse_evologics_message(char *data, int len, state_t *state, int64_t timestamp);
int readline(int fd, char *buf, int max_len);
int send_evologics_command(char *data, char *ret_str, int size, state_t *state);


