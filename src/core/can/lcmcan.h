#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
 
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <time.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>

#include <bot_param/param_client.h>
#include "perls-common/timestamp.h"

typedef struct
{
    char *device;
    canid_t id;
    int socket;
} canlcm_drive;

canlcm_driver *canlcm_create(lcm_t *lcm, char *rootkey);
int canlcm_destroy(canlcm_driver *cld;
int canlcm_write(canlcm_driver *cld, unsigned char *d, int len);
int canlcm_read(canlcm_driver *cld, unsigned char *d, int len, int64_t *timestamp);
int canlcm_read_timeout(canlcm_driver *cld, unsigned char *d, int len, int timeout_s, int64_t *timestamp);

