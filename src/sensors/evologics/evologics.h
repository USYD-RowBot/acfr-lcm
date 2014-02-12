#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include "base91.h"
#include "perls-lcmtypes/senlcm_evologics_usbl_t.h"
#include "perls-lcmtypes/senlcm_evologics_modem_t.h"
#include "perls-lcmtypes/senlcm_evologics_config_t.h"
#include "perls-lcmtypes/acfrlcm_auv_status_t.h"

typedef enum
{
    LCM_AUV_STATUS = 1,
    LCM_AUV_IMAGE
} lcm_mesg_t;
 
enum {io_socket, io_serial};   

typedef struct
{
    lcm_t *lcm;
    int fd;
    int ping_period;
    int ping_counter;
    int current_target;
    int io;
} state_t;

int send_ping(int target, state_t *state);
int send_evologics_data(char *data, int size, int target, state_t *state);
int parse_evologics_message(char *data, int len, state_t *state, int64_t timestamp);
int readline(int fd, char *buf, int max_len);



