
#include <stdio.h>
#include "perls-lcmtypes/senlcm_raw_ascii_t.h"
#include <perls-common/timestamp.h>

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        printf("Usage: send-string [LCM channel] [string]\n");
        return 0;
    }
    
    lcm_t *lcm = lcm_create(NULL);
    senlcm_raw_ascii_t msg;
    msg.utime = timestamp_now();
    msg.msg = argv[2];
    senlcm_raw_ascii_t_publish(lcm, argv[1], &msg);
    lcm_destroy(lcm);
    
    return 1;
}
    
    
